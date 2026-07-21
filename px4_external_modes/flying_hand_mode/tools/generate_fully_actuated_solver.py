#!/usr/bin/env python3
"""Generate the paper-architecture fully actuated Flying Hand ACADOS solver."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

import casadi as ca
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GENERATED_DIR = PACKAGE_ROOT / "generated" / "flying_hand_fully_actuated_solver"
ACADOS_EXPECTED_REVISION = "59d93e17d2985fdd73fc58b8a83ed8f83a024171"
T_RENDERER_EXPECTED_SHA256 = (
    "1973ddd1b536dcd4a059a22f7259c3b0e6b6c878cbfbe7e9b962eba195f361d9"
)

STATE_DIMENSION = 17
CONTROL_DIMENSION = 10
PARAMETER_DIMENSION = 84
HORIZON_NODES = 100
GRAVITY_M_S2 = 9.80665
PAPER_POSITION_WEIGHT = 12.0
PAPER_ORIENTATION_WEIGHT = 10.0
PAPER_VELOCITY_WEIGHT = 0.1
PAPER_ARM_POSITION_WEIGHT = 0.1
PAPER_FORCE_WEIGHT = 0.03
PAPER_MOMENT_WEIGHT = 0.1
IMPLEMENTATION_ARM_COMMAND_WEIGHT = 0.03
QUATERNION_NORM_WEIGHT = 200.0


def quaternion_to_rotation(quaternion_wxyz: ca.SX) -> ca.SX:
    quaternion = quaternion_wxyz / ca.sqrt(
        ca.dot(quaternion_wxyz, quaternion_wxyz) + 1e-12
    )
    w, x, y, z = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    return ca.vertcat(
        ca.horzcat(1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        ca.horzcat(2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        ca.horzcat(2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def quaternion_derivative(quaternion_wxyz: ca.SX, angular_velocity_frd: ca.SX) -> ca.SX:
    w, x, y, z = quaternion_wxyz[0], quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3]
    wx, wy, wz = angular_velocity_frd[0], angular_velocity_frd[1], angular_velocity_frd[2]
    return 0.5 * ca.vertcat(
        -x * wx - y * wy - z * wz,
        w * wx + y * wz - z * wy,
        w * wy + z * wx - x * wz,
        w * wz + x * wy - y * wx,
    )


def homogeneous(rotation: ca.SX, translation: ca.SX) -> ca.SX:
    return ca.vertcat(
        ca.horzcat(rotation, translation),
        ca.horzcat(0, 0, 0, 1),
    )


def standard_dh(theta: ca.SX, d: ca.SX, a: ca.SX, alpha: ca.SX) -> ca.SX:
    cosine_theta = ca.cos(theta)
    sine_theta = ca.sin(theta)
    cosine_alpha = ca.cos(alpha)
    sine_alpha = ca.sin(alpha)
    return ca.vertcat(
        ca.horzcat(
            cosine_theta,
            -sine_theta * cosine_alpha,
            sine_theta * sine_alpha,
            a * cosine_theta,
        ),
        ca.horzcat(
            sine_theta,
            cosine_theta * cosine_alpha,
            -cosine_theta * sine_alpha,
            a * sine_theta,
        ),
        ca.horzcat(0, sine_alpha, cosine_alpha, d),
        ca.horzcat(0, 0, 0, 1),
    )


def end_effector_pose_flu(
    joints: ca.SX,
    d_m: ca.SX,
    a_m: ca.SX,
    alpha_rad: ca.SX,
    theta_offset_rad: ca.SX,
    body_from_arm_translation_flu: ca.SX,
    body_from_arm_quaternion_flu: ca.SX,
) -> tuple[ca.SX, ca.SX]:
    transform = homogeneous(
        quaternion_to_rotation(body_from_arm_quaternion_flu),
        body_from_arm_translation_flu,
    )
    for joint in range(4):
        transform = ca.mtimes(
            transform,
            standard_dh(
                joints[joint] + theta_offset_rad[joint],
                d_m[joint],
                a_m[joint],
                alpha_rad[joint],
            ),
        )
    return transform[:3, 3], transform[:3, :3]


def orientation_error(reference: ca.SX, actual: ca.SX) -> ca.SX:
    skew_error = 0.5 * (ca.mtimes(reference.T, actual) - ca.mtimes(actual.T, reference))
    return ca.vertcat(skew_error[2, 1], skew_error[0, 2], skew_error[1, 0])


def nominal_allocation_inverse() -> np.ndarray:
    radius = 0.34
    tilt = math.radians(20.0)
    allocation = np.zeros((6, 6))
    for rotor in range(6):
        angle = rotor * math.pi / 3.0
        direction = 1.0 if rotor % 2 == 0 else -1.0
        position = np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
        axis = np.array(
            [
                -direction * math.sin(tilt) * math.sin(angle),
                direction * math.sin(tilt) * math.cos(angle),
                -math.cos(tilt),
            ]
        )
        moment = np.cross(position, axis) - direction * 0.02 * axis
        allocation[:3, rotor] = axis
        allocation[3:, rotor] = moment
    return np.linalg.inv(allocation)


def build_ocp():
    from acados_template import AcadosModel, AcadosOcp

    model = AcadosModel()
    model.name = "flying_hand_fully_actuated"

    state = ca.SX.sym("x", STATE_DIMENSION)
    state_dot = ca.SX.sym("xdot", STATE_DIMENSION)
    control = ca.SX.sym("u", CONTROL_DIMENSION)
    parameters = ca.SX.sym("p", PARAMETER_DIMENSION)

    position_ned = state[0:3]
    attitude_ned_frd = state[3:7]
    velocity_ned = state[7:10]
    angular_velocity_frd = state[10:13]
    arm_position = state[13:17]

    force_frd_n = control[0:3]
    moment_frd_nm = control[3:6]
    arm_position_command = control[6:10]

    target_position_ned = parameters[0:3]
    target_attitude_ned = parameters[3:7]
    mass_kg = parameters[7]
    inertia_frd = ca.reshape(parameters[8:17], 3, 3)
    dh_d_m = parameters[17:21]
    dh_a_m = parameters[21:25]
    dh_alpha_rad = parameters[25:29]
    dh_theta_offset_rad = parameters[29:33]
    arm_base_translation_flu = parameters[33:36]
    arm_base_quaternion_flu = parameters[36:40]
    arm_servo_delay_s = parameters[40:44]
    allocation_inverse = ca.reshape(parameters[44:80], 6, 6)
    arm_home_rad = parameters[80:84]

    rotation_ned_frd = quaternion_to_rotation(attitude_ned_frd)
    position_derivative = velocity_ned
    attitude_derivative = quaternion_derivative(attitude_ned_frd, angular_velocity_frd)
    velocity_derivative = ca.vertcat(0, 0, GRAVITY_M_S2) + ca.mtimes(
        rotation_ned_frd, force_frd_n
    ) / mass_kg
    angular_momentum = ca.mtimes(inertia_frd, angular_velocity_frd)
    angular_velocity_derivative = ca.solve(
        inertia_frd,
        moment_frd_nm - ca.cross(angular_velocity_frd, angular_momentum),
    )
    safe_servo_delay = ca.fmax(arm_servo_delay_s, 1e-3)
    arm_position_derivative = (arm_position_command - arm_position) / safe_servo_delay
    explicit_dynamics = ca.vertcat(
        position_derivative,
        attitude_derivative,
        velocity_derivative,
        angular_velocity_derivative,
        arm_position_derivative,
    )

    ee_position_flu, ee_rotation_flu = end_effector_pose_flu(
        arm_position,
        dh_d_m,
        dh_a_m,
        dh_alpha_rad,
        dh_theta_offset_rad,
        arm_base_translation_flu,
        arm_base_quaternion_flu,
    )
    frd_from_flu = ca.DM(np.diag([1.0, -1.0, -1.0]))
    ee_position_frd = ca.mtimes(frd_from_flu, ee_position_flu)
    ee_rotation_frd = ca.mtimes(frd_from_flu, ee_rotation_flu)
    ee_position_ned = position_ned + ca.mtimes(rotation_ned_frd, ee_position_frd)
    ee_rotation_ned = ca.mtimes(rotation_ned_frd, ee_rotation_frd)
    target_rotation_ned = quaternion_to_rotation(target_attitude_ned)

    quaternion_norm_error = ca.dot(attitude_ned_frd, attitude_ned_frd) - 1.0
    path_residual = ca.vertcat(
        ee_position_ned - target_position_ned,
        orientation_error(target_rotation_ned, ee_rotation_ned),
        velocity_ned,
        angular_velocity_frd,
        arm_position - arm_home_rad,
        force_frd_n,
        moment_frd_nm,
        arm_position_command - arm_position,
        quaternion_norm_error,
    )
    terminal_residual = ca.vertcat(
        ee_position_ned - target_position_ned,
        orientation_error(target_rotation_ned, ee_rotation_ned),
        velocity_ned,
        angular_velocity_frd,
        arm_position - arm_home_rad,
        quaternion_norm_error,
    )

    model.x = state
    model.xdot = state_dot
    model.u = control
    model.p = parameters
    model.f_expl_expr = explicit_dynamics
    model.f_impl_expr = state_dot - explicit_dynamics
    model.cost_y_expr = path_residual
    model.cost_y_expr_e = terminal_residual
    rotor_thrust_n = ca.mtimes(allocation_inverse, control[0:6])
    path_constraints = ca.vertcat(rotor_thrust_n, arm_position_derivative)
    model.con_h_expr_0 = path_constraints
    model.con_h_expr = path_constraints

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = HORIZON_NODES
    ocp.solver_options.tf = 2.5

    path_weights = np.concatenate(
        (
            np.full(3, PAPER_POSITION_WEIGHT),
            np.full(3, PAPER_ORIENTATION_WEIGHT),
            np.full(6, PAPER_VELOCITY_WEIGHT),
            np.full(4, PAPER_ARM_POSITION_WEIGHT),
            np.full(3, PAPER_FORCE_WEIGHT),
            np.full(3, PAPER_MOMENT_WEIGHT),
            np.full(4, IMPLEMENTATION_ARM_COMMAND_WEIGHT),
            np.array([QUATERNION_NORM_WEIGHT]),
        )
    )
    terminal_weights = np.concatenate(
        (
            np.full(3, PAPER_POSITION_WEIGHT),
            np.full(3, PAPER_ORIENTATION_WEIGHT),
            np.full(6, PAPER_VELOCITY_WEIGHT),
            np.full(4, PAPER_ARM_POSITION_WEIGHT),
            np.array([QUATERNION_NORM_WEIGHT]),
        )
    )
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.diag(path_weights)
    ocp.cost.W_e = np.diag(terminal_weights)
    ocp.cost.yref = np.zeros(path_weights.size)
    ocp.cost.yref_e = np.zeros(terminal_weights.size)

    ocp.constraints.idxbu = np.arange(6, 10)
    ocp.constraints.lbu = np.full(4, -math.pi)
    ocp.constraints.ubu = np.full(4, math.pi)
    ocp.constraints.idxbx = np.arange(7, 17)
    ocp.constraints.lbx = np.concatenate((np.full(6, -20.0), np.full(4, -math.pi)))
    ocp.constraints.ubx = np.concatenate((np.full(6, 20.0), np.full(4, math.pi)))
    ocp.constraints.idxbx_e = np.arange(7, 17)
    ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
    ocp.constraints.ubx_e = ocp.constraints.ubx.copy()
    ocp.constraints.lh = np.concatenate((np.zeros(6), np.full(4, -10.0)))
    ocp.constraints.uh = np.concatenate((np.full(6, 100.0), np.full(4, 10.0)))
    ocp.constraints.lh_0 = ocp.constraints.lh.copy()
    ocp.constraints.uh_0 = ocp.constraints.uh.copy()

    initial_state = np.zeros(STATE_DIMENSION)
    initial_state[3] = 1.0
    ocp.constraints.x0 = initial_state

    parameter_values = np.zeros(PARAMETER_DIMENSION)
    parameter_values[3] = 1.0
    parameter_values[7] = 5.0
    parameter_values[8:17] = np.diag([0.1, 0.1, 0.15]).flatten(order="F")
    parameter_values[17:21] = [0.0, 0.050, 0.0, 0.076]
    parameter_values[21:25] = [0.363, 0.441, 0.007, 0.200]
    parameter_values[25:29] = [0.10, -0.10, -1.578, 0.0]
    parameter_values[36] = 1.0
    parameter_values[40:44] = [0.66, 0.68, 0.81, 0.85]
    parameter_values[44:80] = nominal_allocation_inverse().flatten(order="F")
    ocp.parameter_values = parameter_values

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = 20
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.timeout_max_time = 0.008
    ocp.solver_options.timeout_heuristic = "ZERO"
    ocp.solver_options.print_level = 0
    ocp.code_gen_options.code_export_directory = str(GENERATED_DIR)
    return ocp


def main() -> None:
    global GENERATED_DIR
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=GENERATED_DIR)
    args = parser.parse_args()
    GENERATED_DIR = args.output.resolve()

    source_value = os.environ.get("ACADOS_SOURCE_DIR")
    if not source_value:
        raise SystemExit("ACADOS_SOURCE_DIR must point to the pinned acados source tree")
    acados_source = Path(source_value).resolve()
    if not (acados_source / "CMakeLists.txt").is_file():
        raise SystemExit(f"Invalid ACADOS_SOURCE_DIR: {acados_source}")

    def git_output(*arguments: str) -> str:
        return subprocess.check_output(
            ["git", "-C", str(acados_source), *arguments], text=True
        ).strip()

    revision = git_output("rev-parse", "HEAD")
    if revision != ACADOS_EXPECTED_REVISION:
        raise SystemExit(
            f"acados revision {revision} does not match pinned {ACADOS_EXPECTED_REVISION}"
        )
    status = git_output(
        "status", "--porcelain", "--untracked-files=all", "--ignore-submodules=none"
    )
    if status:
        raise SystemExit(f"Pinned acados source tree is dirty:\n{status}")

    template_path = acados_source / "interfaces" / "acados_template"
    renderer_path = acados_source / "bin" / "t_renderer"
    renderer_sha256 = hashlib.sha256(renderer_path.read_bytes()).hexdigest()
    if renderer_sha256 != T_RENDERER_EXPECTED_SHA256:
        raise SystemExit(
            f"t_renderer SHA256 {renderer_sha256} does not match pinned "
            f"{T_RENDERER_EXPECTED_SHA256}"
        )
    sys.path.insert(0, str(template_path))
    from acados_template import AcadosOcpSolver

    os.environ["ACADOS_SOURCE_DIR"] = str(acados_source)
    if GENERATED_DIR.exists():
        shutil.rmtree(GENERATED_DIR)
    GENERATED_DIR.parent.mkdir(parents=True, exist_ok=True)

    json_path = GENERATED_DIR.parent / "flying_hand_fully_actuated_ocp.json"
    AcadosOcpSolver.generate(build_ocp(), json_file=str(json_path), verbose=True)
    json_path.unlink()
    for generated_only in (
        "Makefile",
        "acados_solver.pxd",
        "acados_sim_solver_flying_hand_fully_actuated.c",
        "acados_sim_solver_flying_hand_fully_actuated.h",
        "main_flying_hand_fully_actuated.c",
        "main_sim_flying_hand_fully_actuated.c",
    ):
        (GENERATED_DIR / generated_only).unlink(missing_ok=True)
    print(f"Generated fully actuated Flying Hand solver in {GENERATED_DIR}")


if __name__ == "__main__":
    main()
