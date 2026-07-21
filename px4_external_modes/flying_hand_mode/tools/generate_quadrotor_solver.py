#!/usr/bin/env python3
"""Generate the repository-pinned Flying Hand quadrotor ACADOS solver."""

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
import pinocchio as pin


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GENERATED_DIR = PACKAGE_ROOT / "generated" / "flying_hand_quadrotor_solver"
CONTROL_URDF = PACKAGE_ROOT / "models" / "x500_arm2x_control.urdf"
ACADOS_EXPECTED_REVISION = "59d93e17d2985fdd73fc58b8a83ed8f83a024171"
T_RENDERER_EXPECTED_SHA256 = (
    "1973ddd1b536dcd4a059a22f7259c3b0e6b6c878cbfbe7e9b962eba195f361d9"
)

GRAVITY_M_S2 = 9.80665
ROTOR_ARM_M = 0.17688
ROTOR_MOMENT_RATIO_M = 0.01296242524483617
MAX_ROTOR_THRUST_N = 25.000097317747848
ARM_SERVO_TAU_S = np.array([0.08, 0.08, 0.08, 0.06])
ARM_HOME_RAD = np.array([-1.5708, 3.1415, 0.0, 0.0])

PIN_MODEL = pin.buildModelFromUrdf(str(CONTROL_URDF), pin.JointModelFreeFlyer())
ARM_JOINT_IDS = [PIN_MODEL.getJointId(f"joint_{index}") for index in range(1, 5)]
if any(joint_id == 0 for joint_id in ARM_JOINT_IDS) or any(
    PIN_MODEL.joints[joint_id].shortname() != "JointModelRZ"
    for joint_id in ARM_JOINT_IDS
):
    raise RuntimeError("Flying Hand control URDF must contain four revolute Z joints")

ARM_Q_INDICES = [PIN_MODEL.joints[joint_id].idx_q for joint_id in ARM_JOINT_IDS]
ARM_LOWER_RAD = PIN_MODEL.lowerPositionLimit[ARM_Q_INDICES].copy()
ARM_UPPER_RAD = PIN_MODEL.upperPositionLimit[ARM_Q_INDICES].copy()
ARM_MAX_VELOCITY_RAD_S = np.array(
    [PIN_MODEL.velocityLimit[PIN_MODEL.joints[joint_id].idx_v] for joint_id in ARM_JOINT_IDS]
)
ARM_JOINT_PLACEMENTS = tuple(PIN_MODEL.jointPlacements[joint_id] for joint_id in ARM_JOINT_IDS)
EE_PLACEMENT = PIN_MODEL.frames[PIN_MODEL.getFrameId("ee_link")].placement

PIN_Q_HOME = pin.neutral(PIN_MODEL)
PIN_Q_HOME[ARM_Q_INDICES] = ARM_HOME_RAD
PIN_DATA = PIN_MODEL.createData()
COM_FLU_M = np.asarray(pin.centerOfMass(PIN_MODEL, PIN_DATA, PIN_Q_HOME)).copy()
pin.ccrba(PIN_MODEL, PIN_DATA, PIN_Q_HOME, np.zeros(PIN_MODEL.nv))
MASS_KG = float(PIN_DATA.Ig.mass)
INERTIA_FLU_KG_M2 = np.asarray(PIN_DATA.Ig.inertia).copy()


def skew(vector: ca.SX) -> ca.SX:
    return ca.vertcat(
        ca.horzcat(0, -vector[2], vector[1]),
        ca.horzcat(vector[2], 0, -vector[0]),
        ca.horzcat(-vector[1], vector[0], 0),
    )


def quaternion_to_rotation(quaternion_wxyz: ca.SX) -> ca.SX:
    quaternion = quaternion_wxyz / ca.sqrt(ca.dot(quaternion_wxyz, quaternion_wxyz) + 1e-12)
    w, x, y, z = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    return ca.vertcat(
        ca.horzcat(1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        ca.horzcat(2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        ca.horzcat(2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def quaternion_multiply(left: ca.SX | ca.DM, right: ca.SX | ca.DM) -> ca.SX:
    lw, lx, ly, lz = left[0], left[1], left[2], left[3]
    rw, rx, ry, rz = right[0], right[1], right[2], right[3]
    return ca.vertcat(
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_conjugate(quaternion: ca.SX | ca.DM) -> ca.SX:
    return ca.vertcat(quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3])


def homogeneous(rotation: ca.SX | ca.DM, translation: tuple[float, float, float]) -> ca.SX:
    return ca.vertcat(
        ca.horzcat(rotation, ca.DM(translation)),
        ca.horzcat(0, 0, 0, 1),
    )


def arm_end_effector_pose_flu(joints: ca.SX) -> tuple[ca.SX, ca.SX]:
    transform = ca.SX.eye(4)
    quaternion = ca.DM([1.0, 0.0, 0.0, 0.0])
    for index, placement in enumerate(ARM_JOINT_PLACEMENTS):
        translation = tuple(float(value) for value in placement.translation)
        origin_rotation = ca.DM(np.asarray(placement.rotation))
        origin_quaternion_xyzw = pin.Quaternion(placement.rotation).coeffs()
        origin_quaternion = ca.DM(
            [
                origin_quaternion_xyzw[3],
                origin_quaternion_xyzw[0],
                origin_quaternion_xyzw[1],
                origin_quaternion_xyzw[2],
            ]
        )
        transform = ca.mtimes(transform, homogeneous(origin_rotation, translation))
        quaternion = quaternion_multiply(quaternion, origin_quaternion)
        cosine = ca.cos(joints[index])
        sine = ca.sin(joints[index])
        rotation_z = ca.vertcat(
            ca.horzcat(cosine, -sine, 0),
            ca.horzcat(sine, cosine, 0),
            ca.horzcat(0, 0, 1),
        )
        transform = ca.mtimes(transform, homogeneous(rotation_z, (0.0, 0.0, 0.0)))
        joint_quaternion = ca.vertcat(
            ca.cos(joints[index] / 2.0), 0.0, 0.0, ca.sin(joints[index] / 2.0)
        )
        quaternion = quaternion_multiply(quaternion, joint_quaternion)
    ee_translation = tuple(float(value) for value in EE_PLACEMENT.translation)
    ee_rotation = ca.DM(np.asarray(EE_PLACEMENT.rotation))
    ee_quaternion_xyzw = pin.Quaternion(EE_PLACEMENT.rotation).coeffs()
    ee_quaternion = ca.DM(
        [
            ee_quaternion_xyzw[3],
            ee_quaternion_xyzw[0],
            ee_quaternion_xyzw[1],
            ee_quaternion_xyzw[2],
        ]
    )
    transform = ca.mtimes(transform, homogeneous(ee_rotation, ee_translation))
    quaternion = quaternion_multiply(quaternion, ee_quaternion)
    quaternion /= ca.sqrt(ca.dot(quaternion, quaternion) + 1e-12)
    return transform[:3, 3], quaternion


def quaternion_derivative(quaternion_wxyz: ca.SX, angular_velocity_frd: ca.SX) -> ca.SX:
    w, x, y, z = quaternion_wxyz[0], quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3]
    wx, wy, wz = angular_velocity_frd[0], angular_velocity_frd[1], angular_velocity_frd[2]
    return 0.5 * ca.vertcat(
        -x * wx - y * wy - z * wz,
        w * wx + y * wz - z * wy,
        w * wy + z * wx - x * wz,
        w * wz + x * wy - y * wx,
    )


def rotor_allocation_frd(center_of_mass_flu: ca.SX) -> ca.SX:
    rotor_position_flu = ca.DM(
        [
            [ROTOR_ARM_M, -ROTOR_ARM_M, 0.0071418],
            [-ROTOR_ARM_M, ROTOR_ARM_M, 0.0071418],
            [ROTOR_ARM_M, ROTOR_ARM_M, 0.0071418],
            [-ROTOR_ARM_M, -ROTOR_ARM_M, 0.0071418],
        ]
    )
    arm_flu = rotor_position_flu - ca.repmat(center_of_mass_flu.T, 4, 1)
    direction = ca.DM([[1.0, 1.0, -1.0, -1.0]])
    return ca.vertcat(
        ca.DM.ones(1, 4),
        arm_flu[:, 1].T,
        arm_flu[:, 0].T,
        direction * ROTOR_MOMENT_RATIO_M,
    )


def build_ocp():
    from acados_template import AcadosModel, AcadosOcp

    model = AcadosModel()
    model.name = "flying_hand_quadrotor"

    state = ca.SX.sym("x", 17)
    state_dot = ca.SX.sym("xdot", 17)
    control = ca.SX.sym("u", 8)
    parameters = ca.SX.sym("p", 37)

    position_ned = state[0:3]
    attitude_ned_frd = state[3:7]
    velocity_ned = state[7:10]
    angular_velocity_frd = state[10:13]
    arm_position = state[13:17]

    collective_thrust_n = control[0]
    moment_frd_nm = control[1:4]
    arm_position_command = control[4:8]

    target_position_ned = parameters[0:3]
    target_attitude_ned_ee = parameters[3:7]
    force_disturbance_frd_n = parameters[7:10]
    moment_disturbance_frd_nm = parameters[10:13]
    arm_disturbance_rad_s = parameters[13:17]
    previous_control = parameters[17:25]
    center_of_mass_flu = parameters[25:28]
    inertia_flu = ca.reshape(parameters[28:37], 3, 3)

    rotation_ned_frd = quaternion_to_rotation(attitude_ned_frd)
    frd_from_flu = ca.DM(np.diag([1.0, -1.0, -1.0]))
    inertia_frd = ca.mtimes(frd_from_flu, ca.mtimes(inertia_flu, frd_from_flu))

    thrust_force_frd = ca.vertcat(0, 0, -collective_thrust_n)
    position_derivative = velocity_ned
    attitude_derivative = quaternion_derivative(attitude_ned_frd, angular_velocity_frd)
    velocity_derivative = ca.vertcat(0, 0, GRAVITY_M_S2) + ca.mtimes(
        rotation_ned_frd,
        thrust_force_frd + force_disturbance_frd_n,
    ) / MASS_KG
    angular_momentum = ca.mtimes(inertia_frd, angular_velocity_frd)
    angular_velocity_derivative = ca.solve(
        inertia_frd,
        moment_frd_nm + moment_disturbance_frd_nm
        - ca.cross(angular_velocity_frd, angular_momentum),
    )
    arm_position_derivative = (
        (arm_position_command - arm_position) / ca.DM(ARM_SERVO_TAU_S)
        + arm_disturbance_rad_s
    )
    explicit_dynamics = ca.vertcat(
        position_derivative,
        attitude_derivative,
        velocity_derivative,
        angular_velocity_derivative,
        arm_position_derivative,
    )

    arm_ee_position_flu, arm_ee_attitude_flu = arm_end_effector_pose_flu(arm_position)
    ee_position_frd_from_com = ca.mtimes(
        frd_from_flu, arm_ee_position_flu - center_of_mass_flu
    )
    ee_position_ned = position_ned + ca.mtimes(rotation_ned_frd, ee_position_frd_from_com)
    frd_from_flu_attitude = ca.DM([0.0, 1.0, 0.0, 0.0])
    ee_attitude_ned = quaternion_multiply(
        attitude_ned_frd,
        quaternion_multiply(frd_from_flu_attitude, arm_ee_attitude_flu),
    )
    ee_attitude_ned /= ca.sqrt(ca.dot(ee_attitude_ned, ee_attitude_ned) + 1e-12)
    normalized_target_attitude = target_attitude_ned_ee / ca.sqrt(
        ca.dot(target_attitude_ned_ee, target_attitude_ned_ee) + 1e-12
    )
    attitude_error = quaternion_multiply(
        quaternion_conjugate(normalized_target_attitude), ee_attitude_ned
    )
    attitude_error = ca.if_else(attitude_error[0] >= 0.0, attitude_error, -attitude_error)
    attitude_error_vector = attitude_error[1:4]
    attitude_error_norm = ca.sqrt(ca.dot(attitude_error_vector, attitude_error_vector) + 1e-16)
    attitude_error_angle = 2.0 * ca.atan2(attitude_error_norm, attitude_error[0] + 1e-12)
    orientation_error = attitude_error_angle * attitude_error_vector / attitude_error_norm
    tilt_error = rotation_ned_frd[0:2, 2]
    quaternion_norm_error = ca.dot(attitude_ned_frd, attitude_ned_frd) - 1.0
    hover_thrust_n = MASS_KG * GRAVITY_M_S2

    path_residual = ca.vertcat(
        ee_position_ned - target_position_ned,
        orientation_error,
        velocity_ned,
        angular_velocity_frd,
        tilt_error,
        arm_position - ca.DM(ARM_HOME_RAD),
        collective_thrust_n - hover_thrust_n,
        moment_frd_nm,
        arm_position_command - arm_position,
        control - previous_control,
        quaternion_norm_error,
    )
    terminal_residual = ca.vertcat(
        ee_position_ned - target_position_ned,
        orientation_error,
        velocity_ned,
        angular_velocity_frd,
        tilt_error,
        arm_position - ca.DM(ARM_HOME_RAD),
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
    rotor_thrust_n = ca.solve(rotor_allocation_frd(center_of_mass_flu), control[0:4])
    model.con_h_expr = ca.vertcat(rotation_ned_frd[2, 2], rotor_thrust_n)
    model.con_h_expr_e = ca.vertcat(rotation_ned_frd[2, 2])

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = 100
    ocp.solver_options.tf = 2.5

    path_weights = np.concatenate(
        (
            np.full(3, 160.0),
            np.full(3, 80.0),
            np.full(3, 3.0),
            np.full(3, 1.5),
            np.full(2, 30.0),
            np.full(4, 0.15),
            np.array([0.015]),
            np.full(3, 0.08),
            np.full(4, 0.6),
            np.full(8, 0.2),
            np.array([200.0]),
        )
    )
    terminal_weights = np.concatenate(
        (
            np.full(3, 400.0),
            np.full(3, 200.0),
            np.full(3, 8.0),
            np.full(3, 4.0),
            np.full(2, 60.0),
            np.full(4, 0.2),
            np.array([400.0]),
        )
    )
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.diag(path_weights)
    ocp.cost.W_e = np.diag(terminal_weights)
    ocp.cost.yref = np.zeros(path_weights.size)
    ocp.cost.yref_e = np.zeros(terminal_weights.size)

    maximum_collective = 4.0 * MAX_ROTOR_THRUST_N
    maximum_roll_pitch = 2.0 * ROTOR_ARM_M * MAX_ROTOR_THRUST_N
    maximum_yaw = 2.0 * ROTOR_MOMENT_RATIO_M * MAX_ROTOR_THRUST_N
    ocp.constraints.idxbu = np.arange(8)
    ocp.constraints.lbu = np.concatenate(
        ([0.0, -maximum_roll_pitch, -maximum_roll_pitch, -maximum_yaw], ARM_LOWER_RAD)
    )
    ocp.constraints.ubu = np.concatenate(
        ([maximum_collective, maximum_roll_pitch, maximum_roll_pitch, maximum_yaw], ARM_UPPER_RAD)
    )

    constraint_c = np.zeros((4, 17))
    constraint_d = np.zeros((4, 8))
    for joint in range(4):
        constraint_c[joint, 13 + joint] = -1.0
        constraint_d[joint, 4 + joint] = 1.0
    ocp.constraints.C = constraint_c
    ocp.constraints.D = constraint_d
    ocp.constraints.lg = -ARM_MAX_VELOCITY_RAD_S * ARM_SERVO_TAU_S
    ocp.constraints.ug = ARM_MAX_VELOCITY_RAD_S * ARM_SERVO_TAU_S

    ocp.constraints.idxbx = np.arange(13, 17)
    ocp.constraints.lbx = ARM_LOWER_RAD
    ocp.constraints.ubx = ARM_UPPER_RAD
    ocp.constraints.idxbx_e = np.arange(13, 17)
    ocp.constraints.lbx_e = ARM_LOWER_RAD
    ocp.constraints.ubx_e = ARM_UPPER_RAD
    maximum_tilt_rad = math.radians(40.0)
    ocp.constraints.lh = np.concatenate(
        ([math.cos(maximum_tilt_rad)], np.zeros(4))
    )
    ocp.constraints.uh = np.concatenate(
        ([1.0], np.full(4, MAX_ROTOR_THRUST_N))
    )
    ocp.constraints.lh_e = np.array([math.cos(maximum_tilt_rad)])
    ocp.constraints.uh_e = np.array([1.0])

    initial_state = np.zeros(17)
    initial_state[3] = 1.0
    initial_state[13:17] = ARM_HOME_RAD
    ocp.constraints.x0 = initial_state

    parameter_values = np.zeros(37)
    parameter_values[3] = 1.0
    parameter_values[17] = hover_thrust_n
    parameter_values[21:25] = ARM_HOME_RAD
    parameter_values[25:28] = COM_FLU_M
    parameter_values[28:37] = INERTIA_FLU_KG_M2.flatten(order="F")
    ocp.parameter_values = parameter_values

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = 20
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 2
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
    acados_source_value = os.environ.get("ACADOS_SOURCE_DIR")
    if not acados_source_value:
        raise SystemExit("ACADOS_SOURCE_DIR must point to the pinned acados source tree")
    acados_source = Path(acados_source_value).resolve()
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
    submodule_status = git_output("submodule", "status", "--recursive")
    if any(line.startswith(("-", "+", "U")) for line in submodule_status.splitlines()):
        raise SystemExit(
            "Initialize every pinned acados submodule at its recorded revision:\n"
            f"{submodule_status}"
        )

    template_path = acados_source / "interfaces" / "acados_template"
    if not (template_path / "acados_template" / "__init__.py").is_file():
        raise SystemExit(f"Pinned acados_template was not found in {template_path}")
    renderer_path = acados_source / "bin" / "t_renderer"
    if not renderer_path.is_file() or not os.access(renderer_path, os.X_OK):
        raise SystemExit(
            "Install the ACADOS v0.5.5 Linux x86_64 t_renderer in "
            f"{renderer_path} before code generation"
        )
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

    ocp = build_ocp()
    json_path = GENERATED_DIR.parent / "flying_hand_quadrotor_ocp.json"
    AcadosOcpSolver.generate(ocp, json_file=str(json_path), verbose=True)
    json_path.unlink()
    for generated_only in (
        "Makefile",
        "acados_solver.pxd",
        "acados_sim_solver_flying_hand_quadrotor.c",
        "acados_sim_solver_flying_hand_quadrotor.h",
        "main_flying_hand_quadrotor.c",
        "main_sim_flying_hand_quadrotor.c",
    ):
        (GENERATED_DIR / generated_only).unlink(missing_ok=True)
    print(f"Generated Flying Hand quadrotor solver in {GENERATED_DIR}")


if __name__ == "__main__":
    main()
