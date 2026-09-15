"""Common interface for simulated gripper command/state servers."""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


class BaseGripperServer:
    """Base class for gripper-specific command/state adapters."""

    name = "base"

    def start(self) -> None:
        """Start any background communication resources."""

    def close(self) -> None:
        """Release any communication resources."""

    def controlled_joint_names(self) -> Sequence[str]:
        """Return articulation DOF names controlled by this server."""
        return ()

    def resolve_joint_indices(self, articulation_view) -> list[int]:
        """Resolve controlled DOF names against an Isaac articulation view."""
        joint_indices: list[int] = []
        for name in self.controlled_joint_names():
            idx = articulation_view.get_dof_index(name)
            if idx < 0:
                dof_names = getattr(articulation_view, "dof_names", "<unavailable>")
                raise RuntimeError(
                    f"[{self.__class__.__name__}] controlled joint '{name}' was not found. "
                    f"dof_names: {list(dof_names) if dof_names != '<unavailable>' else dof_names}"
                )
            joint_indices.append(idx)
        return joint_indices

    def get_target_positions(
        self,
        current_positions: Optional[np.ndarray],
        simulation_time: float,
    ) -> Optional[np.ndarray]:
        """Return latest position targets for controlled joints, or None."""
        return None

    def publish_state(self, current_positions: np.ndarray, simulation_time: float) -> None:
        """Publish the current controlled joint state, if the protocol has state output."""
