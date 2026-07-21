# Flying Hand Fully Actuated External Mode

The `flying_hand_mode` fully actuated controller reconstructs the low-level architecture
described by the Flying Hand paper for a calibrated, tilted-rotor hexarotor with
a 4-DoF arm. It is an independent implementation from public paper details. It
is not a source port, and the official Flying Hand repository does not provide
the MPC or MuJoCo controller implementation used by the paper.

UMI-on-Air reuses this low-level controller architecture. Its policy, EADP
pipeline, model weights, and checkpoints are outside this package.

## Control Pipeline

```text
EE pose target
  -> 17-state, 10-input ACADOS SQP-RTI MPC
  -> 6D UAV and 4-joint L1 correction
  -> exact six-rotor thrust-box projection
  -> PX4-compatible normalized 6D wrench
  -> PX4 control allocator and ACETele arm commands
```

The MPC state is `[position, attitude quaternion, linear velocity, angular
velocity, arm joints]`. Its control is `[force_xyz, moment_xyz,
arm_position_cmd]`. It uses a 2.5 s horizon, 100 shooting intervals, 25 ms ERK
nodes, SQP-RTI, and a nominal 100 Hz receding-horizon rate. The rate can be set
to 50 Hz for UMI-on-Air-style operation without changing the horizon.

The objective tracks the end-effector SE(3) pose using the paper's published
position, orientation, velocity, joint, force, and moment weights. The arm
command regularizer is explicitly an implementation stability term because its
weight was not published. The nominal model contains Newton-Euler UAV dynamics,
the published 4-DoF DH chain, and first-order servo delay. Arm-to-base coupling
is handled as a disturbance by L1 rather than duplicated inside the MPC.

## Profiles

Run the complete controller without execution outputs:

```bash
ros2 launch flying_hand_mode \
  real_flying_hand_fully_actuated_shadow.launch.py
```

Closed-loop launch has no default configuration. It requires an explicit,
calibrated YAML with both safety gates enabled:

```bash
ros2 launch flying_hand_mode \
  real_flying_hand_fully_actuated.launch.py \
  config_file:=/absolute/path/to/calibrated_fully_actuated.yaml
```

```yaml
flying_hand_fully_actuated:
  ros__parameters:
    closed_loop: true
    calibration_confirmed: true
```

The checked-in shadow YAML contains nominal geometry only and must not be used
as a calibrated real-aircraft profile.

## Calibration Contract

Identify and verify these parameters on the target aircraft:

- `uav.mass_kg` and `uav.inertia_frd_kg_m2`
- `rotor.position_frd_m` and `rotor.axis_frd`
- `rotor.moment_ratio_m`, thrust limits, and `rotor.thrust_curve_kappa`
- the four DH parameter arrays, arm base transform, servo delay, joint limits,
  and velocity limits

Rotor position and axis arrays are rotor-major and contain 18 values. Other
rotor arrays contain 6 values. Startup rejects malformed or non-finite arrays,
a rank-deficient 6x6 effectiveness matrix, an excessive condition number, and
invalid limits. A conventional vertical hexarotor is rank deficient for 6D
wrench control and is deliberately rejected.

The geometry, axis sign, moment ratio, thrust curve, and actuator ordering must
match PX4's `CA_ROTOR*` configuration. The package projects the requested wrench
into rotor thrust limits and reproduces PX4 mixer normalization, but PX4 remains
the owner of final actuator allocation.

## ROS 2 Interface

The mode consumes PX4 odometry, IMU, vehicle status, land state, and
`ControlAllocatorStatus`, plus ACETele follower state/status. It publishes full
three-axis `VehicleThrustSetpoint` and `VehicleTorqueSetpoint` messages and uses
the existing `/ace_leader/*` command protocol.

Mode-specific topics are:

- `/flying_hand_fully_actuated/ee_pose_setpoint`
- `/flying_hand_fully_actuated/ee_pose`
- `/flying_hand_fully_actuated/wrench_nominal`
- `/flying_hand_fully_actuated/wrench_adaptive`
- `/flying_hand_fully_actuated/wrench_applied`
- `/flying_hand_fully_actuated/status`

The quadrotor and fully actuated modes share `/ace_leader/*` and
`/ace_follower/*`; do not run them concurrently.

## Safety And Reproduction Boundary

Shadow mode executes MPC, L1, wrench projection, and diagnostics but never
publishes PX4 thrust/torque or arm execution commands. Closed loop additionally
requires fresh allocator status. Missing allocation, unachieved wrench,
persistent saturation, stale state, repeated solver overruns, or invalid output
faults the mode and requests a return to PX4 Position mode.

This implementation has unit, solver-replay, and ROS shadow coverage. It does
not claim closed-loop flight validation. Environment and self-collision
constraints whose precise forms were not published are intentionally omitted
rather than guessed.

## Build And Test

```bash
git submodule update --init --recursive third_party/acados_vendor/vendor/acados
colcon build --packages-select flying_hand_mode
colcon test --packages-select flying_hand_mode --event-handlers console_direct+
```

The generated solver is checked in. Regeneration is an explicit development
operation using the pinned ACADOS source and
`tools/requirements-fully-actuated-codegen.txt` and
`tools/generate_fully_actuated_solver.py`; normal deployment does not require Python,
CasADi, or the ACADOS template package.
