"""The contract a physics backend meets to drive the simulated robot.

The device sims (motors, sensors, controller) only see the robot through
drivetrain.DrivetrainState (pose, speeds, each drive motor's state) and
field_objects.Field (the blocks on the field and in the intake). Without a
backend, robot.py integrates drivetrain.py and field_objects.py itself (the
builtin physics). A backend replaces both:

  physics_mujoco.MujocoWorld    --physics mujoco (contact, traction)

A Gazebo or Isaac Sim backend would implement the same three calls, stepping
its simulator in lock-step with the robot sim's 1 ms tick (both have a
"step N" control; real time comes from the robot sim's own clock, which
follows QEMU).
"""
from __future__ import annotations

from typing import Protocol, Sequence

from .drivetrain import DrivetrainState, Pose
from .field_objects import Field
from .motor import Drive


class PhysicsWorld(Protocol):
    def step(self, left: Sequence[Drive], right: Sequence[Drive], dt: float,
             limits_a: Sequence[float] | None = None, intake_rad_s: float = 0.0,
             capacity: int = 0) -> DrivetrainState:
        """Advance dt seconds with one drive (volts, None = coast) per drive
        motor, each side's motors in port order, and the intake's shaft speed
        (+ = inward) with how many blocks it can hold. Returns the robot."""

    def field(self) -> Field:
        """The blocks on the field and the colours held, newest last."""

    def pose(self) -> Pose:
        """Where the robot is (sim frame: m from the centre, CCW from +x)."""
