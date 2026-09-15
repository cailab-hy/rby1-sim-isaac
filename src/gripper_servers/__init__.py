"""Gripper server adapters used by the Isaac Sim task."""
from __future__ import annotations

from .base import BaseGripperServer
from .custom_gripper import CustomGripperServer
from .rb_gripper import RbGripperServer


def create_gripper_server(gripper_name: str, **kwargs) -> BaseGripperServer:
    """Create the server adapter for a gripper asset name."""
    if gripper_name == "rb_gripper":
        return RbGripperServer(**kwargs)
    if gripper_name == "custom_gripper":
        return CustomGripperServer(**kwargs)
    raise ValueError(f"Unsupported gripper server for gripper '{gripper_name}'.")


__all__ = [
    "BaseGripperServer",
    "CustomGripperServer",
    "RbGripperServer",
    "create_gripper_server",
]
