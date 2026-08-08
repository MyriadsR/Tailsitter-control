"""Reusable closed-loop tailsitter simulation stepped at the control rate."""

from dataclasses import dataclass

import numpy as np

from controller.INDI_acc_ctrl import INDILinearController
from controller.actuator_ctrl import ControlAllocator
from controller.att_ctrl import (
    FlatnessAttitudeController,
    INDIAngularController,
    PDAttitudeController,
)
from controller.pos_ctrl import PositionController
from environment.aerodynamics import Aerodynamics
from environment.dynamics import UAVDynamics
from utils.filters import SecondOrderButterworthFilter
from utils.math_help import zxy_euler_to_quaternion


def zxy_euler_from_quaternion(q):
    """Convert a wxyz quaternion to ZXY Euler angles [psi, phi, theta]."""
    qw, qx, qy, qz = q
    sin_phi = np.clip(2.0 * (qw * qx + qy * qz), -1.0, 1.0)
    phi = np.arcsin(sin_phi)
    theta = np.arctan2(
        2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qx**2 + qy**2)
    )
    psi = np.arctan2(
        2.0 * (qw * qz - qx * qy), 1.0 - 2.0 * (qx**2 + qz**2)
    )
    return np.array([psi, phi, theta])


def rk4_step(dynamics_model, state, f_alpha, moments, R_b_alpha, dt):
    def derivative(value):
        return dynamics_model.compute_state_derivatives(
            0.0,
            value,
            f_alpha,
            np.zeros(3),
            moments,
            np.zeros(3),
            R_b_alpha,
        )

    k1 = derivative(state)
    k2 = derivative(state + 0.5 * dt * k1)
    k3 = derivative(state + 0.5 * dt * k2)
    k4 = derivative(state + dt * k3)
    next_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    next_state[6:10] /= np.linalg.norm(next_state[6:10])
    return next_state


@dataclass
class SimulationSample:
    time: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    position_ref: np.ndarray
    velocity_ref: np.ndarray
    acceleration_ref: np.ndarray
    psi_ref: float
    euler_deg: np.ndarray
    euler_command_deg: np.ndarray
    angular_velocity: np.ndarray
    angular_velocity_ref: np.ndarray
    angular_acceleration: np.ndarray
    angular_acceleration_lpf: np.ndarray
    angular_acceleration_command: np.ndarray
    rotor_speed: np.ndarray
    flap_deg: np.ndarray
    residual_acceleration_body: np.ndarray
    applied_residual_acceleration_inertial: np.ndarray
    residual_jerk_inertial: np.ndarray


class TailsitterClosedLoop:
    """The existing controller and vehicle model with an optional acceleration residual."""

    def __init__(
        self,
        config,
        trajectory,
        residual_feedforward_consistency=False,
        residual_filter_cutoff_hz=10.0,
    ):
        self.cfg = config
        self.trajectory = trajectory
        self.dt = 1.0 / config.control_rate_hz
        self.steps = int(trajectory.total_duration / self.dt) + 1
        self.step_index = 0
        self.residual_feedforward_consistency = bool(
            residual_feedforward_consistency
        )
        self.residual_filter_cutoff_hz = float(residual_filter_cutoff_hz)
        if self.residual_filter_cutoff_hz <= 0.0:
            raise ValueError("residual_filter_cutoff_hz 必须大于 0")
        self._filtered_residual_acceleration_i = np.zeros(3)

        self.dyn = UAVDynamics(config.mass, config.inertia_matrix, config.g)
        self.aero = Aerodynamics(config)
        self.pos_ctrl = PositionController(config)
        self.indi_lin = INDILinearController(config)
        self.att_flat = FlatnessAttitudeController(config)
        self.att_pd = PDAttitudeController(config)
        self.indi_ang = INDIAngularController(config)
        self.allocator = ControlAllocator(config)

        ca0, sa0 = np.cos(config.alpha_0), np.sin(config.alpha_0)
        self.R_b_to_alpha = np.array(
            [[ca0, 0.0, sa0], [0.0, 1.0, 0.0], [-sa0, 0.0, ca0]]
        )
        self.R_alpha_to_b = self.R_b_to_alpha.T
        self._initialize_state()
        self._initialize_filters()

    def _consistent_residual_reference(self, residual_acceleration_i):
        """Filter the acceleration residual and estimate its inertial jerk."""
        if not self.residual_feedforward_consistency:
            return residual_acceleration_i, np.zeros(3)
        alpha = 1.0 - np.exp(
            -2.0 * np.pi * self.residual_filter_cutoff_hz * self.dt
        )
        previous = self._filtered_residual_acceleration_i.copy()
        self._filtered_residual_acceleration_i += alpha * (
            residual_acceleration_i - self._filtered_residual_acceleration_i
        )
        residual_jerk_i = (
            self._filtered_residual_acceleration_i - previous
        ) / self.dt
        return self._filtered_residual_acceleration_i.copy(), residual_jerk_i

    @property
    def time(self):
        return self.step_index * self.dt

    @property
    def done(self):
        return self.step_index >= self.steps

    def _initialize_state(self):
        cfg = self.cfg
        mode = self.trajectory.initial_state_mode
        initial_flap = 0.0
        if mode == "reference":
            pos, vel, acc, jerk, psi, psi_dot = self.trajectory.sample(0.0)
            q_seed = zxy_euler_to_quaternion(psi, 0.0, 0.0)
            f_ref = cfg.mass * (acc - np.array([0.0, 0.0, cfg.g]))
            initial_flap = -0.27
            q = q_seed
            for _ in range(12):
                q, thrust, _, _ = self.att_flat.compute_desired_attitude_and_thrust(
                    f_ref,
                    vel,
                    2.0 * initial_flap,
                    psi,
                    q,
                    return_angles=True,
                )
                v_b = self.dyn._quat_to_rot_matrix(q).T @ vel
                v_alpha_x = (
                    np.cos(cfg.alpha_0) * v_b[0]
                    + np.sin(cfg.alpha_0) * v_b[2]
                )
                flap_force_slope = (
                    cfg.c_LT_delta
                    * np.cos(cfg.alpha_0 + cfg.alpha_T)
                    * thrust
                    + 2.0 * cfg.c_LV_delta * np.linalg.norm(vel) * v_alpha_x
                )
                if abs(flap_force_slope) > 1e-8:
                    trim = np.clip(
                        cfg.c_mu_T * thrust / (cfg.l_dx * flap_force_slope),
                        -cfg.delta_max,
                        cfg.delta_max,
                    )
                    initial_flap = 0.5 * initial_flap + 0.5 * trim
            q, thrust, phi, theta_bar = (
                self.att_flat.compute_desired_attitude_and_thrust(
                    f_ref,
                    vel,
                    2.0 * initial_flap,
                    psi,
                    q,
                    return_angles=True,
                )
            )
            omega = self.att_flat.compute_feedforward_angular_velocity(
                vel,
                acc,
                jerk,
                psi,
                psi_dot,
                f_ref,
                phi,
                theta_bar,
                2.0 * initial_flap,
            )
            rotor_speed = np.sqrt(max(thrust, 0.0) / (2.0 * cfg.c_T))
            rotors = np.full(2, rotor_speed)
        elif mode == "hover":
            pos, _, _, _, psi, _ = self.trajectory.sample(0.0)
            vel = np.zeros(3)
            omega = np.zeros(3)
            q_seed = zxy_euler_to_quaternion(psi, 0.0, np.pi / 2.0)
            f_hover = -cfg.mass * np.array([0.0, 0.0, cfg.g])
            initial_flap = -0.27
            q, thrust = self.att_flat.compute_desired_attitude_and_thrust(
                f_hover, vel, 2.0 * initial_flap, psi, q_seed
            )
            rotors = np.full(2, np.sqrt(max(thrust, 0.0) / (2.0 * cfg.c_T)))
        elif mode == "ground":
            pos = np.zeros(3)
            vel = np.zeros(3)
            q = zxy_euler_to_quaternion(0.0, 0.0, np.pi / 2.0)
            omega = np.zeros(3)
            rotors = np.zeros(2)
        else:
            raise ValueError(f"未知初始状态模式: {mode}")

        self.state = np.concatenate([pos, vel, q, omega])
        self.omega_rotors = rotors
        self.delta_flaps = np.full(2, initial_flap)

        if mode != "ground":
            v_b = self.dyn._quat_to_rot_matrix(q).T @ vel
            self.f_alpha_lpf, self.m_lpf, _ = self.aero.compute_forces_and_moments(
                v_b,
                rotors[0],
                rotors[1],
                initial_flap,
                initial_flap,
            )
            state_dot = self.dyn.compute_state_derivatives(
                0.0,
                self.state,
                self.f_alpha_lpf,
                np.zeros(3),
                self.m_lpf,
                np.zeros(3),
                self.R_alpha_to_b,
            )
            self.a_lpf = state_dot[3:6].copy()
            self.a_lpf_tilde = self.a_lpf.copy()
        else:
            self.a_lpf = np.array([0.0, 0.0, cfg.g])
            self.a_lpf_tilde = self.a_lpf.copy()
            self.f_alpha_lpf = np.zeros(3)
            self.m_lpf = np.zeros(3)

    def _initialize_filters(self):
        cfg = self.cfg
        kwargs = (cfg.indi_lpf_cutoff_hz, cfg.control_rate_hz)
        self.acceleration_filter = SecondOrderButterworthFilter(
            *kwargs, channels=3, filter_type="lowpass", initial_value=self.a_lpf
        )
        self.angular_velocity_filter = SecondOrderButterworthFilter(
            *kwargs,
            channels=3,
            filter_type="lowpass",
            initial_value=self.state[10:13],
        )
        self.rotor_filter = SecondOrderButterworthFilter(
            *kwargs,
            channels=2,
            filter_type="lowpass",
            initial_value=self.omega_rotors,
        )
        self.flap_filter = SecondOrderButterworthFilter(
            *kwargs,
            channels=2,
            filter_type="lowpass",
            initial_value=self.delta_flaps,
        )
        self.flap_hpf_filter = SecondOrderButterworthFilter(
            cfg.flap_hpf_cutoff_hz,
            cfg.control_rate_hz,
            channels=2,
            filter_type="highpass",
            initial_value=self.delta_flaps,
        )
        self.omega_lpf = self.state[10:13].copy()
        self.omega_dot_lpf = np.zeros(3)
        self.delta_flaps_without_transient = self.delta_flaps.copy()

    def step(self, residual_acceleration_body=None):
        if self.done:
            raise RuntimeError("仿真已经结束")
        residual = np.zeros(3) if residual_acceleration_body is None else np.asarray(
            residual_acceleration_body, dtype=float
        )
        if residual.shape != (3,) or not np.all(np.isfinite(residual)):
            raise ValueError("residual_acceleration_body 必须是有限的三维向量")

        cfg = self.cfg
        t = self.time
        pos = self.state[0:3].copy()
        vel = self.state[3:6].copy()
        q_curr = self.state[6:10].copy()
        omega_curr = self.state[10:13].copy()
        R_b_to_i = self.dyn._quat_to_rot_matrix(q_curr)
        R_i_to_b = R_b_to_i.T
        x_ref, v_ref, a_ref, j_ref, psi_ref, psi_dot_ref = (
            self.trajectory.sample(t)
        )
        residual_acceleration_i, residual_jerk_i = (
            self._consistent_residual_reference(R_b_to_i @ residual)
        )

        a_c = self.pos_ctrl.compute_acceleration_command(
            x_ref,
            v_ref,
            a_ref,
            pos,
            vel,
            self.a_lpf_tilde,
            R_b_to_i,
        )
        a_c += residual_acceleration_i
        f_c_i = self.indi_lin.compute_force_command(
            a_c,
            self.a_lpf_tilde,
            self.f_alpha_lpf,
            R_b_to_i,
            self.R_alpha_to_b,
        )
        delta_sum = np.sum(self.delta_flaps_without_transient)
        q_c, thrust_c = self.att_flat.compute_desired_attitude_and_thrust(
            f_c_i, vel, delta_sum, psi_ref, q_curr
        )

        if self.residual_feedforward_consistency:
            a_ref_for_attitude = a_ref + residual_acceleration_i
            j_ref_for_attitude = j_ref + residual_jerk_i
        else:
            a_ref_for_attitude = a_ref
            j_ref_for_attitude = j_ref
        f_ref_i = cfg.mass * (
            a_ref_for_attitude - np.array([0.0, 0.0, cfg.g])
        )
        _, _, phi_ref, theta_bar_ref = (
            self.att_flat.compute_desired_attitude_and_thrust(
                f_ref_i,
                v_ref,
                delta_sum,
                psi_ref,
                q_curr,
                return_angles=True,
            )
        )
        omega_ref = self.att_flat.compute_feedforward_angular_velocity(
            v_ref,
            a_ref_for_attitude,
            j_ref_for_attitude,
            psi_ref,
            psi_dot_ref,
            f_ref_i,
            phi_ref,
            theta_bar_ref,
            delta_sum,
        )
        omega_dot_c = self.att_pd.compute_angular_acceleration_command(
            q_curr, q_c, self.omega_lpf, omega_ref
        )
        moment_c = self.indi_ang.compute_moment_command(
            omega_dot_c, self.omega_dot_lpf, self.m_lpf
        )
        v_b = R_i_to_b @ vel
        omega_cmd, delta_cmd = self.allocator.allocate_controls(
            thrust_c, moment_c, v_b
        )

        motor_alpha = 1.0 - np.exp(-self.dt / cfg.motor_time_constant)
        servo_alpha = 1.0 - np.exp(-self.dt / cfg.servo_time_constant)
        self.omega_rotors += motor_alpha * (omega_cmd - self.omega_rotors)
        self.omega_rotors = np.clip(
            self.omega_rotors, cfg.omega_min, cfg.omega_max
        )
        delta_step = servo_alpha * (delta_cmd - self.delta_flaps)
        max_delta_step = cfg.servo_rate_max * self.dt
        self.delta_flaps += np.clip(
            delta_step, -max_delta_step, max_delta_step
        )
        self.delta_flaps = np.clip(
            self.delta_flaps, -cfg.delta_max, cfg.delta_max
        )

        f_real, m_real, _ = self.aero.compute_forces_and_moments(
            v_b,
            self.omega_rotors[0],
            self.omega_rotors[1],
            self.delta_flaps[0],
            self.delta_flaps[1],
        )
        state_dot = self.dyn.compute_state_derivatives(
            t,
            self.state,
            f_real,
            np.zeros(3),
            m_real,
            np.zeros(3),
            self.R_alpha_to_b,
        )
        self.state = rk4_step(
            self.dyn, self.state, f_real, m_real, self.R_alpha_to_b, self.dt
        )
        if self.state[2] > 0.0:
            self.state[2] = 0.0
            if self.state[5] > 0.0:
                self.state[5] = 0.0

        self.a_lpf = self.acceleration_filter.update(state_dot[3:6])
        omega_previous = self.omega_lpf.copy()
        self.omega_lpf = self.angular_velocity_filter.update(omega_curr)
        self.omega_dot_lpf = (self.omega_lpf - omega_previous) / self.dt
        rotors_lpf = self.rotor_filter.update(self.omega_rotors)
        flaps_lpf = self.flap_filter.update(self.delta_flaps)
        flaps_hpf = self.flap_hpf_filter.update(flaps_lpf)
        self.delta_flaps_without_transient = flaps_lpf - flaps_hpf

        self.f_alpha_lpf, _, _ = self.aero.compute_forces_and_moments(
            v_b,
            rotors_lpf[0],
            rotors_lpf[1],
            self.delta_flaps_without_transient[0],
            self.delta_flaps_without_transient[1],
        )
        _, self.m_lpf, _ = self.aero.compute_forces_and_moments(
            v_b,
            rotors_lpf[0],
            rotors_lpf[1],
            flaps_lpf[0],
            flaps_lpf[1],
        )
        _, _, f_delta_hpf = self.aero.compute_forces_and_moments(
            v_b,
            rotors_lpf[0],
            rotors_lpf[1],
            flaps_hpf[0],
            flaps_hpf[1],
        )
        R_alpha_to_i = R_b_to_i @ self.R_alpha_to_b
        self.a_lpf_tilde = self.a_lpf - R_alpha_to_i @ f_delta_hpf / cfg.mass

        sample = SimulationSample(
            time=t,
            position=pos,
            velocity=vel,
            acceleration=state_dot[3:6].copy(),
            position_ref=x_ref,
            velocity_ref=v_ref,
            acceleration_ref=a_ref,
            psi_ref=float(psi_ref),
            euler_deg=np.rad2deg(zxy_euler_from_quaternion(q_curr)),
            euler_command_deg=np.rad2deg(zxy_euler_from_quaternion(q_c)),
            angular_velocity=omega_curr,
            angular_velocity_ref=omega_ref,
            angular_acceleration=state_dot[10:13].copy(),
            angular_acceleration_lpf=self.omega_dot_lpf.copy(),
            angular_acceleration_command=omega_dot_c,
            rotor_speed=self.omega_rotors.copy(),
            flap_deg=np.rad2deg(self.delta_flaps.copy()),
            residual_acceleration_body=residual.copy(),
            applied_residual_acceleration_inertial=(
                residual_acceleration_i.copy()
            ),
            residual_jerk_inertial=residual_jerk_i.copy(),
        )
        self.step_index += 1
        return sample
