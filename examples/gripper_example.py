#!/usr/bin/env python3
"""
Gripper control sample (shared for simulation and real robot)
======================================
When Isaac Sim is running with the --sim-gripper flag, uses the sim gripper.
When a real robot (UPC) is connected, uses the actual Dynamixel bus.

Usage:
    # Simulation mode (run Isaac Sim first: ./docker/run.sh)
    python3 gripper_example.py --sim

    # Real robot mode
    python3 gripper_example.py
"""

import argparse
import logging
import os
import sys
import time
import threading

import numpy as np

# sim_gripper_bridge is in ../src/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
log = logging.getLogger(__name__)


# ──────────────────────────────────────────────────────────────────────────────
# DynamixelBus selection: --sim flag → SimDynamixelBus, otherwise real SDK
# ──────────────────────────────────────────────────────────────────────────────
def _make_bus(sim: bool):
    if sim:
        from sim_gripper_bridge import SimDynamixelBus
        return SimDynamixelBus("sim")
    else:
        try:
            import rby1_sdk as rby
            return rby.DynamixelBus(rby.upc.GripperDeviceName)
        except ImportError:
            raise SystemExit("rby1_sdk not found. Please use the --sim flag.")


# ──────────────────────────────────────────────────────────────────────────────
# Gripper class (same structure as real robot example 35)
# ──────────────────────────────────────────────────────────────────────────────
class Gripper:
    """
    Two-finger gripper (Dynamixel ID 0=left, 1=right).

    normalized_q convention:
        0.0 = fully open
        1.0 = fully closed
    """

    GRIPPER_IDS = [0, 1]

    def __init__(self, bus):
        self.bus = bus
        self.min_q = np.array([np.inf,  np.inf])
        self.max_q = np.array([-np.inf, -np.inf])
        self.target_q: np.ndarray | None = None
        self._running = False
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    def initialize(self) -> bool:
        """Open port + verify ping + enable torque."""
        self.bus.open_port()
        self.bus.set_baud_rate(2_000_000)
        self.bus.set_torque_constant([1, 1])

        ok = True
        for dev_id in self.GRIPPER_IDS:
            model = self.bus.ping(dev_id)
            if not model:
                log.error(f"  Dynamixel ID {dev_id} — ping failed")
                ok = False
            else:
                log.info(f"  Dynamixel ID {dev_id} — OK (model {model})")
        if ok:
            self.bus.group_sync_write_torque_enable(
                [(dev_id, 1) for dev_id in self.GRIPPER_IDS]
            )
        return ok

    # ------------------------------------------------------------------
    def _set_operating_mode(self, mode: int) -> None:
        self.bus.group_sync_write_torque_enable(
            [(dev_id, 0) for dev_id in self.GRIPPER_IDS]
        )
        self.bus.group_sync_write_operating_mode(
            [(dev_id, mode) for dev_id in self.GRIPPER_IDS]
        )
        self.bus.group_sync_write_torque_enable(
            [(dev_id, 1) for dev_id in self.GRIPPER_IDS]
        )

    # ------------------------------------------------------------------
    def homing(self) -> bool:
        """
        Probes both mechanical limits by pushing torque in each direction and sets min_q / max_q.
        Real robot: detects actual encoder limits
        Simulation:  SimDynamixelBus internally performs fake homing converging to ±0.785 rad
        """
        log.info("Starting homing...")
        CurrentControlMode = 0  # DynamixelBus.CurrentControlMode

        self._set_operating_mode(CurrentControlMode)

        q = np.zeros(2)
        prev_q = np.zeros(2)
        direction = 0
        counter = 0

        while direction < 2:
            torque = 0.3 * (1 if direction == 0 else -1)
            self.bus.group_sync_write_send_torque(
                [(dev_id, torque) for dev_id in self.GRIPPER_IDS]
            )
            rv = self.bus.group_fast_sync_read_encoder(self.GRIPPER_IDS)
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            self.min_q = np.minimum(self.min_q, q)
            self.max_q = np.maximum(self.max_q, q)

            if np.allclose(prev_q, q, atol=1e-6):
                counter += 1
            else:
                counter = 0
            prev_q = q.copy()

            if counter >= 30:
                direction += 1
                counter = 0
            time.sleep(0.05)

        log.info(f"  Homing complete — min={self.min_q}, max={self.max_q}")
        return True

    # ------------------------------------------------------------------
    def start(self) -> None:
        """Start the position control loop as a background thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.bus.close_port()

    def _loop(self) -> None:
        CurrentBasedPositionControlMode = 5
        self._set_operating_mode(CurrentBasedPositionControlMode)
        self.bus.group_sync_write_send_torque(
            [(dev_id, 5) for dev_id in self.GRIPPER_IDS]
        )
        while self._running:
            if self.target_q is not None:
                self.bus.group_sync_write_send_position(
                    [(dev_id, float(q)) for dev_id, q in enumerate(self.target_q)]
                )
            time.sleep(0.05)

    # ------------------------------------------------------------------
    def set_normalized(self, value: float) -> None:
        """
        Sets the gripper opening.

        Args:
            value: 0.0 (open) ~ 1.0 (closed)
        """
        if not np.isfinite(self.min_q).all() or not np.isfinite(self.max_q).all():
            log.warning("Homing not complete, ignoring command.")
            return
        value = float(np.clip(value, 0.0, 1.0))
        # Same as real robot (GRIPPER_DIRECTION=False): 1.0=closed(min_q), 0.0=open(max_q)
        self.target_q = (1.0 - value) * (self.max_q - self.min_q) + self.min_q

    def set_open(self)  -> None: self.set_normalized(0.0)
    def set_closed(self) -> None: self.set_normalized(1.0)


# ──────────────────────────────────────────────────────────────────────────────
# Demo sequence
# ──────────────────────────────────────────────────────────────────────────────
def demo(gripper: Gripper) -> None:
    log.info("=== Demo sequence start ===")

    steps = [
        (0.0, "open"),
        (1.0, "closed"),
        (0.5, "half"),
        (1.0, "closed"),
        (0.0, "open"),
    ]
    for normalized, label in steps:
        log.info(f"  → {label} ({normalized:.1f})")
        gripper.set_normalized(normalized)
        time.sleep(1.5)

    log.info("=== Demo complete ===")


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(description="RBY1 gripper control sample")
    parser.add_argument(
        "--sim", action="store_true",
        help="Use SimDynamixelBus (communicates with Isaac Sim --sim-gripper)"
    )
    parser.add_argument(
        "--cmd-port", type=int, default=5007,
        help="Gripper command send port (simulation only, default 5007)  * 0.0=open, 1.0=closed"
    )
    parser.add_argument(
        "--state-port", type=int, default=5008,
        help="Gripper state receive port (simulation only, default 5008)"
    )
    args = parser.parse_args()

    bus = _make_bus(args.sim)

    # Override ports in simulation mode
    if args.sim:
        bus._cmd_port   = args.cmd_port
        bus._state_port = args.state_port

    gripper = Gripper(bus)

    log.info(f"{'[sim]' if args.sim else '[real robot]'} Initializing...")
    if not gripper.initialize():
        raise SystemExit("Initialization failed")

    gripper.homing()
    gripper.start()

    try:
        demo(gripper)
        log.info("Press Ctrl+C to quit, or enter interactive input:")
        while True:
            try:
                raw = input("Enter opening (0.0~1.0, q=quit): ").strip()
            except EOFError:
                break
            if raw.lower() in ("q", "quit", "exit"):
                break
            try:
                gripper.set_normalized(float(raw))
            except ValueError:
                log.warning("Please enter a valid number.")
    except KeyboardInterrupt:
        pass
    finally:
        gripper.stop()
        log.info("Exiting.")


if __name__ == "__main__":
    main()
