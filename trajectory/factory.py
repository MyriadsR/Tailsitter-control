"""Trajectory construction shared by simulation, training, and evaluation."""

from trajectory.test import (
    CircularTrajectory,
    CircularTransitionTrajectory,
    DifferentialThrustTurnTrajectory,
    KnifeEdgeTransitionTrajectory,
    LemniscateTrajectory,
    TrajectoryGenerator,
)


TRAJECTORY_CHOICES = (
    "segments",
    "lemniscate",
    "knife-edge-transition",
    "circle-coordinated",
    "circle-knife-edge",
    "circular-knife-edge",
    "transition-to-forward",
    "transition-to-hover",
    "differential-turn",
)


def make_trajectory(name, laps=None, speed=None, lap_time=None):
    if name == "lemniscate":
        return LemniscateTrajectory(
            speed=6.0 if speed is None else speed,
            lap_time=7.0 if lap_time is None else lap_time,
            laps=8 if laps is None else laps,
        )
    if name in {"knife-edge-transition", "circle-knife-edge"}:
        return KnifeEdgeTransitionTrajectory(
            speed=6.0 if speed is None else speed,
            lap_time=6.25 if lap_time is None else lap_time,
            laps=8 if laps is None else laps,
        )
    if name in {"circle-coordinated", "circular-knife-edge"}:
        return CircularTrajectory(
            mode="coordinated" if name == "circle-coordinated" else "knife_edge",
            speed=8.1 if speed is None else speed,
            laps=4 if laps is None else laps,
        )
    if name in {"transition-to-forward", "transition-to-hover"}:
        transition_duration = 3.0
        target_speed = 8.1 if speed is None else speed
        return CircularTransitionTrajectory(
            transition="to_forward" if name == "transition-to-forward" else "to_hover",
            tangential_acceleration=target_speed / transition_duration,
            duration=transition_duration,
        )
    if name == "differential-turn":
        return DifferentialThrustTurnTrajectory(speed=7.0 if speed is None else speed)
    if name == "segments":
        return TrajectoryGenerator()
    raise ValueError(f"未知轨迹类型: {name}")
