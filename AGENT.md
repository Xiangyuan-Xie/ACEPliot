# ACEPliot Agent Guide

This guide defines repository-level expectations for coding agents and
maintainers. Keep it environment-independent: project rules must not depend on a
username, home directory, IDE session, timezone, or one machine's installed
packages.

## Project Snapshot

ACEPliot is a ROS 2 Humble colcon workspace for PX4 aerial-manipulator control,
simulation, and real-flight deployment. The target baseline is Ubuntu 22.04,
PX4 v1.14 or newer, C++17, and Python 3.10.

The control architecture has five layers:

1. `libraries/` contains reusable code without ROS node ownership.
2. `px4_external_modes/` contains controllers that own a PX4 External Mode
   lifecycle.
3. `commanders/` contains ordinary ROS 2 deployment nodes for upper policies.
4. `trajectory_generators/` contains composable reference sources.
5. `benchmarks/` contains repeatable Python workloads, metrics, and reports.

Do not blur these boundaries. A commander must not become a `NodeWithMode`, a
trajectory source must not start PX4 or ACESim, a benchmark must not own the
simulator lifecycle, and a reusable policy library must not declare ROS
parameters.

Real-flight dependencies are not implied by a successful source checkout.
Verify ROS, PX4 interfaces, submodules, ONNX Runtime, ACADOS,
`MicroXRCEAgent`, and `mavlink-routerd` before treating an environment failure
as a code defect.

## Repository Map

- `libraries/policy_inference/` - ROS-free ONNX policy API, tensor maps, and
  optional recurrent-state handling. Callers own parameters and logging.
- `px4_external_modes/am_position_mode/` - the single `AM Position` PX4 mode.
  It owns vehicle state, observations, inference, RNN state, flight logging,
  Offboard references, and rates/thrust publication.
- `px4_external_modes/flying_hand_mode/` - one ROS package containing shared
  runtime plus isolated quadrotor and fully-actuated controllers, generated
  ACADOS solvers, launch files, configs, and tests.
- `commanders/am_ee_pose_commander/` - ACELab AM EE Pose upper-policy
  deployment. It publishes references for PX4 AM Offboard and drives the
  ACETele arm handshake; it is not an External Mode.
- `trajectory_generators/figure8_trajectory/` - Python PX4 position Figure-8
  references and AM EE Pose five-frame previews.
- `benchmarks/benchmark_reporting/` - Python alignment, metrics, CSV/JSON
  serialization, and headless PNG plots.
- `benchmarks/velocity_tracking_benchmark/` - scripted PX4 Offboard velocity
  tracking workload.
- `benchmarks/arm_motion_benchmark/` - 100 Hz ACETele arm motion workload whose
  report covers only aerial-base position drift.
- `benchmarks/aerial_manipulation_benchmark/` - launch-only composition of the
  velocity and arm-motion workloads.
- `benchmarks/figure8_tracking_benchmark/` - PX4 and EE Figure-8 tracking
  launch composition.
- `px4_state_converter/` - PX4 / ROS 2 state, IMU, visual odometry, and ground
  truth conversion.
- `tools/airlink/` - AirLink and `mavlink-routerd` configuration.
- `tests/` - repository-level Python tests for package layout and launch
  contracts. Package-local tests stay in each package's `test/` directory.
- `third_party/` - submodules and vendor packages such as `px4_msgs`, the PX4
  ROS 2 interface library, ONNX Runtime, ACADOS, and mavlink-router.

Generated and local output is not source of truth. Do not base behavior on
`build/`, `install/`, `log/`, IDE metadata, agent state, or local caches.

## Setup

Initialize dependencies from the workspace root:

```bash
cd <ACEPliot_ROOT>
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

Build and source the workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Install the repository-pinned MAVLink router when AirLink tooling is needed:

```bash
git submodule update --init --recursive third_party/mavlink-router
./tools/airlink/install_mavlink_router.sh
```

PX4-side `uxrce_dds_client` and companion-side `MicroXRCEAgent` are distinct.
Real launch files default to not starting the agent. Prefer one manually managed
instance:

```bash
MicroXRCEAgent udp4 -p 8888
```

Only use `start_micro_xrce_agent:=true` when launch-owned lifecycle is an
explicit requirement.

## Common Commands

Build the refactored control packages:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  policy_inference am_position_mode flying_hand_mode am_ee_pose_commander \
  figure8_trajectory benchmark_reporting velocity_tracking_benchmark \
  arm_motion_benchmark aerial_manipulation_benchmark figure8_tracking_benchmark
```

Run repository-level tests:

```bash
python3 -m unittest discover -s tests
```

Run package tests and inspect their aggregate result:

```bash
colcon test --packages-select \
  policy_inference am_position_mode flying_hand_mode am_ee_pose_commander \
  figure8_trajectory benchmark_reporting velocity_tracking_benchmark \
  arm_motion_benchmark aerial_manipulation_benchmark figure8_tracking_benchmark \
  --event-handlers console_direct+
colcon test-result --verbose
```

Launch AM Position:

```bash
ros2 launch am_position_mode sim_am_position.launch.py
ros2 launch am_position_mode real_am_position.launch.py
```

Launch Figure-8 references:

```bash
ros2 launch figure8_trajectory px4_figure8_trajectory.launch.py
ros2 launch figure8_trajectory ee_figure8_trajectory.launch.py
```

Launch the deployment commander and Python benchmarks:

```bash
ros2 launch am_ee_pose_commander sim_am_ee_pose_commander.launch.py \
  upper_model_path:=<ABSOLUTE_UPPER_ONNX>
ros2 launch velocity_tracking_benchmark sim_velocity_tracking_benchmark.launch.py
ros2 launch arm_motion_benchmark sim_arm_motion_benchmark.launch.py
ros2 launch aerial_manipulation_benchmark sim_aerial_manipulation_benchmark.launch.py
ros2 launch figure8_tracking_benchmark sim_px4_figure8_tracking_benchmark.launch.py
```

Launch Flying Hand profiles:

```bash
ros2 launch flying_hand_mode sim_flying_hand_quadrotor.launch.py
ros2 launch flying_hand_mode real_flying_hand_quadrotor_shadow.launch.py
ros2 launch flying_hand_mode real_flying_hand_fully_actuated_shadow.launch.py
```

The fully-actuated closed-loop launch intentionally requires an explicit
calibrated file:

```bash
ros2 launch flying_hand_mode real_flying_hand_fully_actuated.launch.py \
  config_file:=<ABSOLUTE_CALIBRATED_YAML>
```

Start ACESim separately before an ACEPliot sim launch. ACEPliot does not own the
simulator lifecycle:

```bash
ros2 launch acesim_ros2 linux.launch.py ace_follower:=auto
```

Simulation defaults use `/acesim/clock`, `/acesim/vehicle/odometry`, and the
`/ace_follower/*` handshake topics.

## Development Conventions

- Read the package's `package.xml`, `CMakeLists.txt`, launch, config, and tests
  before changing behavior.
- Keep changes scoped to established package boundaries, C++17 deployment
  code, and Python 3.10 benchmark code.
- Preserve `-Wall -Wextra -Wpedantic -Werror` compatibility where enabled.
- Use structured parsers for YAML, XML, JSON, and CMake data when a suitable
  parser already exists. Avoid broad string rewrites.
- Keep topic defaults absolute, including the leading `/`.
- Preserve simulation and real-flight differences. Never turn a private IP,
  serial device, username, or local model path into a repository default.
- Do not edit `third_party/` unless dependency synchronization is explicitly
  requested.

AM Position contract:

- The only executable and display mode is `am_position_mode` / `AM Position`.
- The only accepted deployment semantics is `body_rate_thrust_raw` with FLU
  policy rates, FRD publication, and `sigmoid_2x` collective preprocessing.
- The thrust scale parameter is `collective_scale`.
- Publish only `RatesSetpointType` body rates and thrust. Do not reintroduce
  direct-motor output or variant inheritance.
- Keep ONNX Runtime details in `policy_inference`; ROS parameters, vehicle
  state, command mapping, and flight logs belong to the mode.

AM EE Pose contract:

- The commander runs only the upper policy. PX4 firmware owns the AM Offboard
  low-level policy and actuator output.
- Keep five preview poses at offsets `0`, `0.02`, `0.04`, `0.06`, and `1.0 s`;
  63 upper observations; 8 actions; 50 Hz upper and arm updates; 64-value GRU
  state where the deployed model declares it.
- With no trajectory preview, hold the initial EE pose.
- A model using ONNX external data requires its referenced `.data` file. Do not
  download or invent weights in code.
- Do not claim reproduction of ACELab self-collision termination when deployment
  telemetry provides no equivalent contact signal.

Command ownership:

- Only one process may publish `/fmu/in/offboard_control_mode` and
  `/fmu/in/trajectory_setpoint` for a vehicle.
- Use `ee_figure8_trajectory` beside `am_ee_pose_commander`; do not run the
  direct PX4 Figure-8 trajectory at the same time.
- ACETele gripper state is a normalized public value, not raw `joint_5` radians.
- Flying Hand variants and arm-motion workloads share `/ace_leader/*` and
  `/ace_follower/*`; do not run competing publishers.

Figure-8 contract:

- Use physical parameters `period_s`, `amplitude_x_m`, `amplitude_y_m`,
  `max_linear_speed_m_s`, `transition_time_s`, and `loops_to_run`.
- Geometry is `x=x0+Ax*sin(phase)`, `y=y0+Ay*sin(2*phase)`; Y amplitude is the
  actual peak.
- Validate `omega*sqrt(Ax^2+4*Ay^2)` against the speed limit.
- EE ramp-in changes phase speed with smooth5. Do not ramp amplitude.
- Count finite loops from actual accumulated phase and hold the origin when
  complete.
- Publish `/figure8_trajectory/{px4,ee}/status` using `waiting`, `running`,
  `finished`, or `failed`; finite runs must return to the origin and exit.

Benchmark contract:

- Benchmark workloads and reporting are Python 3 / `ament_python`; deployment
  commanders, PX4 External Modes, and their control libraries remain C++.
- Simulation workloads advance only from `/acesim/clock`; do not add wall-time
  fallback to a simulation benchmark.
- Reports use bounded timestamp interpolation without extrapolation and write
  CSV/JSON plus headless PNG/SVG/PDF figure bundles before launch shutdown.
- Spatial trajectory figures use an equal-scale ENU 3D hero panel with XY,
  XZ, and YZ projections. Keep raw samples unsmoothed and SVG text editable.
- Default artifacts belong in `benchmarks/<package>/logs/<UTC_TIMESTAMP>/`;
  keep package-local `logs/` directories untracked.
- `arm_motion_benchmark` drives the arm but reports only base position drift.
  Do not subscribe to follower joint or gripper state from its reporter.

Flying Hand contract:

- Keep shared runtime and both controllers in `flying_hand_mode`, but preserve
  separate solver, controller, calibration, and generated-symbol boundaries.
- PX4 vehicle and wrench state uses NED/FRD. Fully-actuated rotor arrays are
  rotor-major.
- Reject invalid, rank-deficient, or poorly conditioned allocation matrices.
- Shadow mode must never publish execution commands.
- Never bypass `closed_loop` and `calibration_confirmed` gates.
- Generated ACADOS code is checked-in deployment input. Change its generator
  and regenerate against the pinned vendor; do not hand-edit only generated C.
- The fully-actuated implementation is a paper-based reconstruction, not an
  official source port or hardware-validation claim.

## Commit Notes

- Do not create a commit unless the user explicitly asks.
- Inspect `git status --short` and `git diff` before staging.
- Preserve unrelated user changes in a dirty worktree.
- Keep each commit focused on one logical change and give every non-root change
  an informative scope.
- Match the message to the actual diff. Examples:

```text
refactor(am-position): merge policy runtime into mode
refactor(flying-hand): consolidate controller packages
feat(figure8): publish end-effector target previews
fix(airlink): preserve qgc udp forwarding
docs(repo): document package ownership
```

- Run the narrowest relevant tests before committing and report failures or
  missing dependencies.
- For a submodule change, commit inside the submodule first, then commit the
  parent gitlink. Never commit only the parent's view of a dirty submodule.
- Do not commit model weights, flight logs, rosbags, build products, or personal
  IDE and agent configuration unless repository policy explicitly requires it.

## Subagent Use

Subagents may be used freely for independent work. The primary agent remains
responsible for integration and review.

- Give each subagent a concrete, bounded task and disjoint write ownership.
- Never assign two agents to the same file, CMake target, launch/config pair, or
  generated solver.
- Tell subagents not to revert unrelated changes in the shared worktree.
- Prefer parallel package implementation, read-only audits, test triage, and
  documentation consistency checks.
- Require a summary of changed paths and verification results.
- Re-check all conclusions that affect real flight, PX4 topics, frame
  conversion, AirLink, allocator output, or `MicroXRCEAgent` lifecycle.

## Testing Notes

Scale tests with the changed behavior. A launch rename needs static and
`--show-args` checks; shared inference or controller changes need focused unit
tests plus dependent-package builds; real-flight safety changes need human
hardware validation.

Useful checks:

```bash
python3 -m unittest discover -s tests
colcon build --packages-select <CHANGED_PACKAGES>
colcon test --packages-select <CHANGED_PACKAGES> --event-handlers console_direct+
colcon test-result --verbose
pre-commit run --all-files
```

Some checks require ROS 2, PX4 interfaces, ONNX Runtime, ACADOS,
`MicroXRCEAgent`, `mavlink-routerd`, or network access. Report a missing
dependency accurately instead of masking it as a source failure.

Never claim real-flight validation unless the command ran on the relevant
vehicle, companion computer, and ground-station setup.

## Documentation Notes

- Update `README.md` and this guide when package paths, launch files, public
  parameters, topic names, ports, frame conventions, model contracts, or
  real-flight procedures change.
- Keep the root README Chinese-first and organized in the ACESim-style
  Best-README-Template rhythm.
- Keep the distinction between PX4-side `uxrce_dds_client` and companion-side
  `MicroXRCEAgent` explicit.
- Use `<ACEPliot_ROOT>` and placeholder absolute paths. Never preserve local
  usernames, home directories, IDE state, or temporary analysis in docs.
