"""Skeleton command/state server for a user-provided gripper."""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from gripper_servers.base import BaseGripperServer


class CustomGripperServer(BaseGripperServer):
    """Placeholder adapter for a custom gripper protocol.

    Users can copy this class and fill in the protocol-specific parts:
    controlled joint names, command reception, target conversion, and optional
    state publishing. Asset attachment is handled separately by ``gripper.json``.
    """

    name = "custom_gripper"
    JOINT_NAMES = ()

    def __init__(self, **_kwargs) -> None:
        self._running = False

    def controlled_joint_names(self) -> Sequence[str]:
        return self.JOINT_NAMES

    def start(self) -> None:
        self._running = True
        print("[CustomGripperServer] Protocol is not implemented yet; no commands will be applied.")

    def close(self) -> None:
        self._running = False

    def get_target_positions(
        self,
        current_positions: Optional[np.ndarray],
        simulation_time: float,
    ) -> Optional[np.ndarray]:
        return None

    def publish_state(self, current_positions: np.ndarray, simulation_time: float) -> None:
        return
