"""UDP command/state server for the Rainbow Robotics two-finger gripper."""
from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Dict, Final, Optional, Sequence

import numpy as np

from config import (
    GRIPPER_CMD_PORT,
    GRIPPER_HOST,
    GRIPPER_STATE_PORT,
)
from gripper_servers.base import BaseGripperServer


_CMD_MAGIC = 0x4743    # 'GC'
_STATE_MAGIC = 0x4753  # 'GS'

_CMD_FMT = "<HBBff"     # 12 bytes
_STATE_FMT = "<HHfff"   # 16 bytes

# Fake homing range for SimDynamixelBus (emulates motor radian span at ~+/-45 deg).
RB_GRIPPER_HOME_MIN_RAD: Final[float] = -0.785398
RB_GRIPPER_HOME_MAX_RAD: Final[float] = +0.785398
RB_GRIPPER_HOMING_STEP_RAD: Final[float] = 0.1

# URDF finger prismatic joint travel: lower=-0.05 m (open), upper=0.0 m (closed).
RB_GRIPPER_FINGER_TRAVEL_M: Final[float] = 0.05


def closeness_to_finger_meter(closeness: float) -> float:
    """0 (open) -> -0.05 m, 1 (closed) -> 0.0 m."""
    c = max(0.0, min(1.0, float(closeness)))
    return -RB_GRIPPER_FINGER_TRAVEL_M * (1.0 - c)


def finger_meter_to_closeness(meter: float) -> float:
    """Inverse of :func:`closeness_to_finger_meter`."""
    return max(0.0, min(1.0, 1.0 + float(meter) / RB_GRIPPER_FINGER_TRAVEL_M))


class RbGripperServer(BaseGripperServer):
    """Receive rb_gripper commands from ``SimDynamixelBus`` over UDP."""

    name = "rb_gripper"
    JOINT_NAMES = ("gripper_finger_l1", "gripper_finger_r1")

    def __init__(
        self,
        *,
        cmd_bind_host: str = "0.0.0.0",
        cmd_bind_port: int = GRIPPER_CMD_PORT,
        state_send_host: str = GRIPPER_HOST,
        state_send_port: int = GRIPPER_STATE_PORT,
    ) -> None:
        self._cmd_addr = (cmd_bind_host, cmd_bind_port)
        self._state_addr = (state_send_host, state_send_port)

        self._lock = threading.Lock()
        self._left_closeness: float = 0.0
        self._right_closeness: float = 0.0
        self._left_torque_enable: bool = False
        self._right_torque_enable: bool = False
        self._homed: bool = False
        self._last_rx_time: float = 0.0

        self._recv_sock: Optional[socket.socket] = None
        self._send_sock: Optional[socket.socket] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._running = False

    def controlled_joint_names(self) -> Sequence[str]:
        return self.JOINT_NAMES

    def start(self) -> None:
        if self._running:
            return
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.bind(self._cmd_addr)
        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop, name="RbGripperServer-recv", daemon=True
        )
        self._recv_thread.start()
        print(f"[RbGripperServer] cmd recv <- {self._cmd_addr[0]}:{self._cmd_addr[1]}, "
              f"state send -> {self._state_addr[0]}:{self._state_addr[1]}")

    def close(self) -> None:
        self._running = False
        for sock in (self._recv_sock, self._send_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._recv_sock = None
        self._send_sock = None
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=0.5)
            self._recv_thread = None

    def get_targets(self) -> Dict[str, float]:
        """Latest command snapshot. Closeness values are clipped to [0, 1]."""
        with self._lock:
            return {
                "left": self._left_closeness,
                "right": self._right_closeness,
                "left_torque_enable": self._left_torque_enable,
                "right_torque_enable": self._right_torque_enable,
                "homed": self._homed,
                "last_rx_time": self._last_rx_time,
            }

    def get_target_positions(
        self,
        current_positions: Optional[np.ndarray],
        simulation_time: float,
    ) -> Optional[np.ndarray]:
        cmd = self.get_targets()
        if not cmd["homed"]:
            return None
        left_m = closeness_to_finger_meter(cmd["left"])
        right_m = closeness_to_finger_meter(cmd["right"])
        return np.array([left_m, right_m], dtype=np.float64)

    def publish_state(self, current_positions: np.ndarray, simulation_time: float) -> None:
        if len(current_positions) < 2:
            return
        self.set_present(
            finger_meter_to_closeness(current_positions[0]),
            finger_meter_to_closeness(current_positions[1]),
            simulation_time,
        )

    def set_present(self, left_closeness: float, right_closeness: float, sim_time: float) -> None:
        if self._send_sock is None:
            return
        cl = float(np.clip(left_closeness, 0.0, 1.0))
        cr = float(np.clip(right_closeness, 0.0, 1.0))
        pkt = struct.pack(_STATE_FMT, _STATE_MAGIC, 0, cl, cr, float(sim_time))
        try:
            self._send_sock.sendto(pkt, self._state_addr)
        except OSError:
            pass

    def _recv_loop(self) -> None:
        sock = self._recv_sock
        if sock is None:
            return
        buf = bytearray(64)
        size = struct.calcsize(_CMD_FMT)
        while self._running:
            try:
                n, _ = sock.recvfrom_into(buf)
            except OSError:
                break
            if n < size:
                continue
            try:
                magic, flags, _res, cl, cr = struct.unpack_from(_CMD_FMT, buf, 0)
            except struct.error:
                continue
            if magic != _CMD_MAGIC:
                continue
            cl_c = float(np.clip(cl, 0.0, 1.0))
            cr_c = float(np.clip(cr, 0.0, 1.0))
            with self._lock:
                self._homed = bool(flags & 0x1)
                self._left_torque_enable = bool(flags & 0x2)
                self._right_torque_enable = bool(flags & 0x4)
                self._left_closeness = cl_c
                self._right_closeness = cr_c
                self._last_rx_time = time.monotonic()
