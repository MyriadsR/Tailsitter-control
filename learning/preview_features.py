"""Body-frame state and future-reference observations for the residual policy."""

import numpy as np

from environment.closed_loop import zxy_euler_from_quaternion


STATE_DIM = 16
PREVIEW_CHANNELS = 11
DEFAULT_PREVIEW_POINTS = 12
DEFAULT_PREVIEW_SECONDS = 0.6


def angle_difference(lhs, rhs):
    return (lhs - rhs + np.pi) % (2.0 * np.pi) - np.pi


def build_preview_observation(
    simulator,
    preview_points=DEFAULT_PREVIEW_POINTS,
    preview_seconds=DEFAULT_PREVIEW_SECONDS,
):
    """Build normalized current-state and future-reference branches."""
    state = simulator.state
    pos = state[0:3]
    vel = state[3:6]
    q = state[6:10]
    omega = state[10:13]
    R_b_to_i = simulator.dyn._quat_to_rot_matrix(q)
    R_i_to_b = R_b_to_i.T
    x_ref, v_ref, a_ref = simulator.trajectory.sample(simulator.time)[:3]

    state_features = np.concatenate(
        [
            R_i_to_b @ (x_ref - pos) / 5.0,
            R_i_to_b @ (v_ref - vel) / 5.0,
            R_i_to_b @ (a_ref - simulator.a_lpf_tilde) / 10.0,
            omega / 10.0,
            simulator.omega_rotors / max(simulator.cfg.omega_max, 1.0),
            simulator.delta_flaps / max(simulator.cfg.delta_max, 1e-6),
        ]
    )

    yaw = zxy_euler_from_quaternion(q)[0]
    future_times = simulator.time + np.linspace(
        0.0, preview_seconds, preview_points
    )
    preview = []
    for future_time in future_times:
        x_future, v_future, a_future, _, psi_future, _ = (
            simulator.trajectory.sample(future_time)
        )
        yaw_error = angle_difference(psi_future, yaw)
        preview.append(
            np.concatenate(
                [
                    R_i_to_b @ (x_future - pos) / 10.0,
                    R_i_to_b @ v_future / 10.0,
                    R_i_to_b @ a_future / 20.0,
                    [np.sin(yaw_error), np.cos(yaw_error)],
                ]
            )
        )
    preview_features = np.asarray(preview, dtype=np.float32).T
    if state_features.shape != (STATE_DIM,):
        raise RuntimeError(f"状态观测维度错误: {state_features.shape}")
    if preview_features.shape != (PREVIEW_CHANNELS, preview_points):
        raise RuntimeError(f"预瞄观测维度错误: {preview_features.shape}")
    return {
        "state": np.clip(state_features, -10.0, 10.0).astype(np.float32),
        "preview": np.clip(preview_features, -10.0, 10.0),
    }
