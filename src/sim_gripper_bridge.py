# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""UDP bridge for the simulated RBY1 gripper.

The real RBY1 gripper is driven by Dynamixel motors over ``/dev/rby1_gripper``
via ``rby1_sdk.DynamixelBus``. To let user scripts target the simulator without
emulating a serial device, this module exposes:

* :class:`SimDynamixelBus`  — drop-in replacement for ``rby1_sdk.DynamixelBus``
  on the user (SDK) side. Sends commands over UDP and receives state back.
* :class:`SimGripperServer` — backwards-compatible alias for
  :class:`gripper_servers.rb_gripper.RbGripperServer` on the Isaac Sim side.

Protocol (UDP, little-endian)
-----------------------------
Default ports: cmd 5007 (bus → sim), state 5008 (sim → bus). Override via the
``RBY1_SIM_GRIPPER_*`` environment variables (see :mod:`config`).

Command packet (12 bytes)::

    magic:u16 = 0x4743 ('GC')
    flags:u8       # bit0 = homed, bit1 = TE left, bit2 = TE right
    reserved:u8
    closeness_l:f32   # 0.0 (open) ~ 1.0 (closed)
    closeness_r:f32

State packet (16 bytes)::

    magic:u16 = 0x4753 ('GS')
    reserved:u16
    closeness_l:f32
    closeness_r:f32
    timestamp:f32     # sim time [s]

Values are dimensionless closeness so the bus side owns all calibration.
"""
from __future__ import annotations

import socket
import struct
import threading
from typing import Dict, List, Optional, Tuple

from config import (
    GRIPPER_CMD_PORT,
    GRIPPER_HOST,
    GRIPPER_STATE_PORT,
)
from gripper_servers.rb_gripper import (
    RB_GRIPPER_HOME_MAX_RAD,
    RB_GRIPPER_HOME_MIN_RAD,
    RB_GRIPPER_HOMING_STEP_RAD,
    RbGripperServer,
    closeness_to_finger_meter,
    finger_meter_to_closeness,
)


# ============================================================
# Protocol constants
# ============================================================

_CMD_MAGIC = 0x4743    # 'GC'
_STATE_MAGIC = 0x4753  # 'GS'

_CMD_FMT = "<HBBff"     # 12 bytes
_STATE_FMT = "<HHfff"   # 16 bytes

# Gripper motor IDs (rby1-sdk convention).
GRIPPER_IDS: Tuple[int, int] = (0, 1)

# DynamixelBus class constants (match the real SDK).
_TORQUE_DISABLE = 0
_TORQUE_ENABLE = 1
_CURRENT_CONTROL_MODE = 0
_CURRENT_BASED_POSITION_CONTROL_MODE = 5

# Fake model number (XM430-W350 = 1020) returned by ping.
_FAKE_MODEL_NUMBER = 1020


# ============================================================
# Internal motor state
# ============================================================

class _MotorState:
    __slots__ = (
        "torque_enable", "op_mode",
        "goal_position_rad", "goal_current_a", "goal_torque_nm",
        "present_position_rad",
        "p_gain", "i_gain", "d_gain",
    )

    def __init__(self) -> None:
        self.torque_enable: int = 0
        self.op_mode: int = _CURRENT_BASED_POSITION_CONTROL_MODE
        self.goal_position_rad: float = 0.0
        self.goal_current_a: float = 0.0
        self.goal_torque_nm: float = 0.0
        self.present_position_rad: float = 0.0
        self.p_gain: int = 800
        self.i_gain: int = 0
        self.d_gain: int = 0


class _FakeMotorState:
    """``rby1_sdk.DynamixelBus.MotorState`` compatible attribute container."""
    __slots__ = ("torque_enable", "position", "velocity", "current", "torque", "temperature")

    def __init__(self, torque_enable, position, velocity, current, torque, temperature):
        self.torque_enable = torque_enable
        self.position = position
        self.velocity = velocity
        self.current = current
        self.torque = torque
        self.temperature = temperature


# ============================================================
# SimDynamixelBus — shim used by user scripts (SDK consumer side)
# ============================================================

class SimDynamixelBus:
    """Simulation replacement for ``rby1_sdk.DynamixelBus``.

    Methods called by the SDK ``Gripper`` example are exposed with identical
    signatures. Commands (closeness, 0=open ~ 1=closed) are forwarded to Isaac
    Sim over UDP.

    The ``dev_name`` argument is accepted for API compatibility and ignored.
    Thread-safe: ``homing`` runs on the main thread, the position-control loop
    runs on a background thread; all state access is guarded by ``_lock``.
    """

    # --- Constants matching the real DynamixelBus ----------------------
    ProtocolVersion = 2.0
    DefaultBaudrate = 2_000_000
    TorqueEnable = _TORQUE_ENABLE
    TorqueDisable = _TORQUE_DISABLE
    CurrentControlMode = _CURRENT_CONTROL_MODE
    CurrentBasedPositionControlMode = _CURRENT_BASED_POSITION_CONTROL_MODE
    AddrTorqueEnable = 64
    AddrOperatingMode = 11
    AddrGoalPosition = 116
    AddrGoalCurrent = 102
    AddrPresentCurrent = 126
    AddrPresentVelocity = 128
    AddrPresentPosition = 132
    AddrCurrentTemperature = 146
    AddrPositionPGain = 84
    AddrPositionIGain = 82
    AddrPositionDGain = 80

    def __init__(
        self,
        dev_name: str = "sim",
        *,
        sim_host: str = GRIPPER_HOST,
        cmd_port: int = GRIPPER_CMD_PORT,
        state_port: int = GRIPPER_STATE_PORT,
    ) -> None:
        self._dev_name = dev_name
        self._sim_addr = (sim_host, cmd_port)
        self._state_port = state_port

        self._lock = threading.Lock()
        self._motors: Dict[int, _MotorState] = {i: _MotorState() for i in GRIPPER_IDS}
        self._torque_constants: List[float] = [1.0, 1.0]

        self._send_sock: Optional[socket.socket] = None
        self._recv_sock: Optional[socket.socket] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._running = False

        # Tracks the direction in which to ramp present_position during homing.
        self._homing_dir: Dict[int, int] = {i: 0 for i in GRIPPER_IDS}  # +1, -1, 0

    # ---------------- Lifecycle ----------------

    def open_port(self) -> bool:
        if self._send_sock is not None:
            return True
        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._recv_sock.bind(("0.0.0.0", self._state_port))
        except OSError:
            # Same-host setups may already hold the state port — ignore and skip recv.
            self._recv_sock.close()
            self._recv_sock = None
        else:
            self._running = True
            self._recv_thread = threading.Thread(
                target=self._recv_loop, name="SimDxlBus-recv", daemon=True
            )
            self._recv_thread.start()
        return True

    def close_port(self) -> None:
        self._running = False
        for sock_name in ("_send_sock", "_recv_sock"):
            sock = getattr(self, sock_name)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
                setattr(self, sock_name, None)
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=0.5)
            self._recv_thread = None

    def set_baud_rate(self, baudrate: int) -> bool:  # noqa: ARG002
        return True

    def set_torque_constant(self, torque_constant: List[float]) -> None:
        with self._lock:
            self._torque_constants = list(torque_constant)

    # ---------------- Single-motor R/W ----------------

    def ping(self, id: int) -> Optional[int]:
        # Real SDK returns model_number or None (truthy/falsy).
        return _FAKE_MODEL_NUMBER if id in self._motors else None

    def send_torque_enable(self, id: int, onoff: int) -> None:
        with self._lock:
            m = self._motors.get(id)
            if m is None:
                return
            m.torque_enable = 1 if onoff else 0
        self._send_cmd_packet()

    def send_operating_mode(self, id: int, operating_mode: int) -> None:
        with self._lock:
            m = self._motors.get(id)
            if m is None:
                return
            m.op_mode = int(operating_mode)

    def send_goal_position(self, id: int, goal_position: float) -> None:
        with self._lock:
            m = self._motors.get(id)
            if m is None:
                return
            m.goal_position_rad = float(goal_position)
        self._send_cmd_packet()

    def send_goal_current(self, id: int, current: float) -> None:
        with self._lock:
            m = self._motors.get(id)
            if m is None:
                return
            m.goal_current_a = float(current)
            # During homing, current sign acts as a direction signal.
            if m.op_mode == _CURRENT_CONTROL_MODE:
                self._homing_dir[id] = _sign(current)

    def send_current(self, id: int, current: float) -> None:
        # SDK alias.
        self.send_goal_current(id, current)

    def send_torque(self, id: int, torque: float) -> None:
        with self._lock:
            m = self._motors.get(id)
            if m is None:
                return
            m.goal_torque_nm = float(torque)
            if m.op_mode == _CURRENT_CONTROL_MODE:
                self._homing_dir[id] = _sign(torque)

    def read_encoder(self, id: int) -> Optional[float]:
        with self._lock:
            m = self._motors.get(id)
            if m is None:
                return None
            self._advance_homing_position_locked(id)
            return m.present_position_rad

    def read_operating_mode(self, id: int, use_cache: bool = False) -> Optional[int]:  # noqa: ARG002
        with self._lock:
            m = self._motors.get(id)
            return None if m is None else m.op_mode

    def read_torque_enable(self, id: int) -> Optional[int]:
        with self._lock:
            m = self._motors.get(id)
            return None if m is None else m.torque_enable

    def read_temperature(self, id: int) -> Optional[int]:
        # Stub: real motor temperature is not modelled; report a safe room value.
        return 25 if id in self._motors else None

    # ---------------- PID gain stubs ----------------

    def set_position_p_gain(self, id: int, p_gain: int) -> None:
        self._set_gain(id, "p_gain", p_gain)

    def set_position_i_gain(self, id: int, i_gain: int) -> None:
        self._set_gain(id, "i_gain", i_gain)

    def set_position_d_gain(self, id: int, d_gain: int) -> None:
        self._set_gain(id, "d_gain", d_gain)

    def set_position_pid_gain(self, id: int, p_gain=None, i_gain=None, d_gain=None) -> None:
        # Single-overload form only (PIDGain struct not supported).
        with self._lock:
            m = self._motors.get(id)
            if m is None:
                return
            if p_gain is not None:
                m.p_gain = int(p_gain)
            if i_gain is not None:
                m.i_gain = int(i_gain)
            if d_gain is not None:
                m.d_gain = int(d_gain)

    def get_position_p_gain(self, id: int) -> Optional[int]:
        return self._get_gain(id, "p_gain")

    def get_position_i_gain(self, id: int) -> Optional[int]:
        return self._get_gain(id, "i_gain")

    def get_position_d_gain(self, id: int) -> Optional[int]:
        return self._get_gain(id, "d_gain")

    def get_position_pid_gain(self, id: int) -> Optional[Tuple[int, int, int]]:
        with self._lock:
            m = self._motors.get(id)
            return None if m is None else (m.p_gain, m.i_gain, m.d_gain)

    def _set_gain(self, id: int, attr: str, value: int) -> None:
        with self._lock:
            m = self._motors.get(id)
            if m is not None:
                setattr(m, attr, int(value))

    def _get_gain(self, id: int, attr: str) -> Optional[int]:
        with self._lock:
            m = self._motors.get(id)
            return None if m is None else getattr(m, attr)

    # ---------------- Group ops ----------------

    def group_fast_sync_read(
        self, ids: List[int], addr: int, length: int  # noqa: ARG002
    ) -> Optional[List[Tuple[int, int]]]:
        # Default to zero — call patterns are too varied to model precisely.
        return [(i, 0) for i in ids if i in self._motors]

    def group_fast_sync_read_encoder(
        self, ids: List[int]
    ) -> Optional[List[Tuple[int, float]]]:
        out: List[Tuple[int, float]] = []
        with self._lock:
            for i in ids:
                m = self._motors.get(i)
                if m is None:
                    continue
                self._advance_homing_position_locked(i)
                out.append((i, m.present_position_rad))
        return out if out else None

    def group_fast_sync_read_operating_mode(
        self, ids: List[int], use_cache: bool = False  # noqa: ARG002
    ) -> Optional[List[Tuple[int, int]]]:
        with self._lock:
            return [(i, self._motors[i].op_mode) for i in ids if i in self._motors] or None

    def group_fast_sync_read_torque_enable(
        self, ids: List[int]
    ) -> Optional[List[Tuple[int, int]]]:
        with self._lock:
            return [(i, self._motors[i].torque_enable) for i in ids if i in self._motors] or None

    def get_motor_states(self, ids: List[int]):
        """Returns a list compatible with ``rby1_sdk.DynamixelBus.MotorState``."""
        out = []
        with self._lock:
            for i in ids:
                m = self._motors.get(i)
                if m is None:
                    continue
                out.append((i, _FakeMotorState(
                    torque_enable=bool(m.torque_enable),
                    position=m.present_position_rad,
                    velocity=0.0,
                    current=m.goal_current_a,
                    torque=m.goal_torque_nm,
                    temperature=25,
                )))
        return out if out else None

    def group_sync_write_torque_enable(self, *args) -> None:
        """Supports both overloads::

            group_sync_write_torque_enable([(id, on), ...])
            group_sync_write_torque_enable([id, ...], on)
        """
        if len(args) == 1:
            pairs = args[0]
        elif len(args) == 2:
            ids, enable = args
            pairs = [(i, enable) for i in ids]
        else:
            raise TypeError("group_sync_write_torque_enable: bad arity")
        with self._lock:
            for id_, on in pairs:
                m = self._motors.get(id_)
                if m is not None:
                    m.torque_enable = 1 if on else 0
        self._send_cmd_packet()

    def group_sync_write_operating_mode(self, pairs) -> None:
        with self._lock:
            for id_, mode in pairs:
                m = self._motors.get(id_)
                if m is not None:
                    m.op_mode = int(mode)

    def group_sync_write_send_position(self, pairs) -> None:
        with self._lock:
            for id_, pos in pairs:
                m = self._motors.get(id_)
                if m is not None:
                    m.goal_position_rad = float(pos)
        self._send_cmd_packet()

    def group_sync_write_send_torque(self, pairs) -> None:
        with self._lock:
            for id_, tau in pairs:
                m = self._motors.get(id_)
                if m is None:
                    continue
                m.goal_torque_nm = float(tau)
                if m.op_mode == _CURRENT_CONTROL_MODE:
                    self._homing_dir[id_] = _sign(tau)

    def send_vibration(self, id: int, vibration: int) -> None:  # noqa: ARG002
        # Leader-arm haptics — no-op in simulation.
        return

    def read_button_status(self, id: int):  # noqa: ARG002
        # Leader-arm trigger — not modelled in simulation.
        return None

    # ---------------- Internal helpers ----------------

    def _advance_homing_position_locked(self, id: int) -> None:
        """Ramp ``present_position`` one step toward the fake homing limit."""
        m = self._motors[id]
        if m.op_mode != _CURRENT_CONTROL_MODE:
            return
        d = self._homing_dir[id]
        if d > 0:
            m.present_position_rad = min(
                m.present_position_rad + RB_GRIPPER_HOMING_STEP_RAD,
                RB_GRIPPER_HOME_MAX_RAD,
            )
        elif d < 0:
            m.present_position_rad = max(
                m.present_position_rad - RB_GRIPPER_HOMING_STEP_RAD,
                RB_GRIPPER_HOME_MIN_RAD,
            )

    def _send_cmd_packet(self) -> None:
        """Snapshot under the lock, build the packet, then send outside the lock."""
        if self._send_sock is None:
            return

        with self._lock:
            m0, m1 = self._motors[0], self._motors[1]
            te0, te1 = m0.torque_enable, m1.torque_enable
            # The fake calibration is fixed, so goal_position always lies in
            # [RB_GRIPPER_HOME_MIN_RAD, RB_GRIPPER_HOME_MAX_RAD] after homing.
            c0 = _closeness_from_rad(m0.goal_position_rad)
            c1 = _closeness_from_rad(m1.goal_position_rad)

        homed = 1  # always homed (fake calibration is fixed)
        flags = (homed & 0x1) | ((te0 & 0x1) << 1) | ((te1 & 0x1) << 2)
        pkt = struct.pack(_CMD_FMT, _CMD_MAGIC, flags, 0, c0, c1)
        try:
            self._send_sock.sendto(pkt, self._sim_addr)
        except OSError:
            pass

    def _recv_loop(self) -> None:
        sock = self._recv_sock
        if sock is None:
            return
        buf = bytearray(64)
        while self._running:
            try:
                n, _ = sock.recvfrom_into(buf)
            except OSError:
                break
            if n < struct.calcsize(_STATE_FMT):
                continue
            try:
                magic, _res, cl, cr, _ts = struct.unpack_from(_STATE_FMT, buf, 0)
            except struct.error:
                continue
            if magic != _STATE_MAGIC:
                continue
            # Inverse-transform sim closeness back to radians and update present.
            # Only meaningful in position control mode (homing uses its own ramp).
            with self._lock:
                for motor_id, c in ((0, cl), (1, cr)):
                    m = self._motors[motor_id]
                    if m.op_mode == _CURRENT_BASED_POSITION_CONTROL_MODE:
                        m.present_position_rad = _rad_from_closeness(c)


# Backwards-compatible Isaac Sim side server name. New code should import
# RbGripperServer through the gripper_servers package.
SimGripperServer = RbGripperServer

__all__ = [
    "SimDynamixelBus",
    "SimGripperServer",
    "closeness_to_finger_meter",
    "finger_meter_to_closeness",
]


# ============================================================
# Closeness / radian conversions
# ============================================================

def _sign(x: float) -> int:
    if x > 1e-9:
        return 1
    if x < -1e-9:
        return -1
    return 0


def _closeness_from_rad(rad: float) -> float:
    """Fake calibration: max_rad = open (closeness 0), min_rad = closed (1)."""
    span = RB_GRIPPER_HOME_MAX_RAD - RB_GRIPPER_HOME_MIN_RAD
    if span <= 0:
        return 0.0
    c = (RB_GRIPPER_HOME_MAX_RAD - float(rad)) / span
    return min(1.0, max(0.0, c))


def _rad_from_closeness(c: float) -> float:
    span = RB_GRIPPER_HOME_MAX_RAD - RB_GRIPPER_HOME_MIN_RAD
    return RB_GRIPPER_HOME_MAX_RAD - float(c) * span
