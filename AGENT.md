# ACEPliot Agent Guide

This guide is for coding agents and maintainers working in the ACEPliot
repository. It summarizes the project shape, common commands, and local
conventions so a new agent can make focused changes without rediscovering the
basics.

## Project Snapshot

ACEPliot is a ROS 2 / PX4 control workspace for aerial manipulation research,
simulation validation, and real-flight deployment. The codebase is organized as
a colcon workspace and should remain a set of ROS packages, not a standalone
CMake project or loose script collection.

Core capabilities:

- PX4 External Flight Modes for reinforcement-learning control.
- Shared RL runtime code for robot state, observations, ONNX Runtime inference,
  RNN state, and flight logging.
- Motor and CTBR position-control entries for the aerial-manipulator workflow.
- PX4 / ROS 2 state, IMU, and odometry conversion nodes.
- Figure-8 Offboard reference generation.
- AirLink / MAVLink ground-station routing utilities.
- Optional uXRCE-DDS communication through a manually managed
  `MicroXRCEAgent`.

Target baseline:

- Ubuntu 22.04.
- ROS 2 Humble.
- PX4 v1.14 or newer.
- C++17.

These are project-level target assumptions, not proof that the current machine
has every dependency installed. Before reporting a code issue, verify whether
the local ROS, PX4, `MicroXRCEAgent`, `mavlink-routerd`, submodule, or system
dependency is missing.

## Repository Map

- `px4_external_modes/rl_base_mode/` - shared PX4 external-mode runtime for RL
  controllers, including state containers, ONNX Runtime inference helpers,
  observation history, RNN state, and logging.
- `px4_external_modes/am_position_mode/` - aerial-manipulator position-control
  mode, including Motor and CTBR executables, launch files, YAML configs, tests,
  and default policy-weight path conventions.
- `px4_state_converter/` - bidirectional PX4 / ROS 2 bridge for visual
  odometry, ground-truth odometry, IMU, and related frame conversions.
- `trajectory_generators/figure8_trajectory_mode/` - figure-8 trajectory
  generation, PX4 Offboard reference conversion, and testable trajectory logic.
- `tools/airlink/` - AirLink configuration scripts, scenario YAML files, and
  `mavlink-routerd` templates for real-flight ground-station links.
- `third_party/` - vendored or submodule-based dependencies such as `px4_msgs`,
  the PX4 ROS 2 interface library, `onnxruntime_vendor`, and `mavlink-router`.
- `tests/` - repository-level Python tests for cross-package behavior that does
  not belong to a single ROS package.

Generated output and local state are not source of truth. Avoid relying on
`build/`, `install/`, `log/`, IDE metadata, agent metadata, analysis output, or
local cache files when changing project behavior.

## Setup

Initialize submodules and install ROS dependencies from the workspace root:

```bash
cd <ACEPliot_ROOT>
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

Build the whole workspace:

```bash
cd <ACEPliot_ROOT>
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Build a focused package when the change is local to one package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select am_position_mode
```

AirLink tooling depends on `mavlink-routerd`; install it through the repository
script:

```bash
git submodule update --init --recursive third_party/mavlink-router
./tools/airlink/install_mavlink_router.sh
```

Real PX4 / ROS 2 communication requires a PX4-side `uxrce_dds_client` and a
companion-computer-side `MicroXRCEAgent`. The true real-flight launch files
default to not starting the agent automatically. Prefer running one agent in a
separate terminal:

```bash
MicroXRCEAgent udp4 -p 8888
```

Only use `start_micro_xrce_agent:=true` when explicitly restoring the older
launch-owned agent lifecycle.

## Common Commands

Run all package tests:

```bash
source /opt/ros/humble/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Run targeted tests for the real launch agent behavior and AirLink config:

```bash
python3 -m unittest tests.test_micro_xrce_agent_launch tools.airlink.test_configure_airlink
```

Run the RL real-flight Motor mode:

```bash
ros2 launch am_position_mode real_am_position_motor.launch.py
```

Run the RL real-flight CTBR mode:

```bash
ros2 launch am_position_mode real_am_position_ctbr.launch.py
```

Publish external odometry to PX4:

```bash
ros2 launch px4_state_converter generic_odometry.launch.py
```

Run figure-8 Offboard references:

```bash
ros2 launch figure8_trajectory_mode figure8_position_mode.launch.py
ros2 launch figure8_trajectory_mode figure8_velocity_mode.launch.py
```

Configure the MAVLink ground-station link:

```bash
./tools/airlink/configure_airlink.sh
sudo journalctl -u mavlink-router -f
```

Run code-quality hooks when available:

```bash
pre-commit run --all-files
```

## Development Conventions

- Prefer small, scoped changes that respect existing ROS package boundaries.
- Read the relevant `package.xml`, `CMakeLists.txt`, launch files, configs, and
  tests before editing a package.
- Keep C++ changes compatible with the existing CMake baseline: C++17 with
  `-Wall -Wextra -Wpedantic -Werror` where enabled.
- Avoid changing PX4 / ROS topic names, frame conventions, timestamps, QoS,
  launch defaults, real-flight ports, or model paths unless the task explicitly
  requires it.
- Preserve simulation and real-flight configuration differences. Do not turn a
  local serial device, private IP address, username, or absolute model path into
  a project default.
- Use structured parsers or existing helper code for YAML, XML, TOML, and CMake
  data where practical. Avoid brittle ad hoc string edits.
- Do not edit `third_party/` unless the task explicitly involves vendored or
  upstream dependency synchronization.
- Treat `third_party/px4_msgs/` and the PX4 ROS 2 interface library as interface
  material. Message, service, or interface changes require rebuild notes.
- Do not commit large model weights, flight logs, rosbag files, generated build
  products, or personal IDE / agent configuration unless the repository
  convention explicitly allows it.

## Commit Notes

- Do not create commits unless the user explicitly asks for a commit.
- Keep each commit focused on one logical change. Do not mix documentation,
  launch behavior, generated files, dependency updates, and unrelated cleanup in
  the same commit.
- Before staging, inspect `git status --short` and `git diff`. Stage only files
  that belong to the requested change, and do not stage unrelated user edits.
- Before committing, run the narrowest relevant verification commands and record
  any failures, skipped checks, or missing dependencies in the final response.
- Prefer clear, scoped commit messages, for example:

```text
docs: align README with ACESim style
fix(px4-state): make MicroXRCEAgent launch opt-in
test(airlink): cover UDP config rendering
feat(trajectory): add figure-eight velocity option
```

- If a change touches `third_party/px4_msgs/`,
  `third_party/px4-ros2-interface-lib/`, `third_party/mavlink-router/`, or any
  other submodule, commit inside that submodule first. Then return to the
  ACEPliot parent repository and commit the updated gitlink.
- Do not commit only the parent repository's view of a dirty submodule working
  tree. Other users cannot reproduce that state.
- If the user asks for a PR summary, include purpose, main files or ROS packages
  changed, tests run, environment assumptions, and whether topics, ports,
  coordinate frames, config formats, or real-flight behavior changed.

## Subagent Use

Subagents are allowed and encouraged when they make the work safer or faster.
The main agent remains responsible for integration and review.

- Good subagent tasks include independent read-only investigations, package
  audits, test-result triage, documentation consistency checks, and bounded
  edits with disjoint file ownership.
- Do not let multiple subagents edit the same file, CMake target, launch entry,
  YAML config, or generated artifact.
- Tell every subagent that it is not alone in the repository and must not revert
  or overwrite unrelated changes.
- Require subagent summaries to cite repository-relative paths and concrete
  project facts.
- Do not preserve local usernames, private paths, session sandbox details, IDE
  tab state, or temporary analysis output as long-term project rules.
- Re-check subagent conclusions before reporting changes that affect real
  flight, PX4 topics, AirLink, `MicroXRCEAgent`, or control behavior.

## Testing Notes

Useful targeted checks:

```bash
colcon build --packages-select rl_base_mode
colcon build --packages-select am_position_mode
colcon build --packages-select px4_state_converter
colcon build --packages-select figure8_trajectory_mode
```

```bash
colcon test --packages-select am_position_mode --event-handlers console_direct+
colcon test --packages-select px4_state_converter --event-handlers console_direct+
colcon test --packages-select figure8_trajectory_mode --event-handlers console_direct+
colcon test-result --verbose
```

Some checks require ROS 2, PX4 interface packages, `onnxruntime_vendor`,
`MicroXRCEAgent`, `mavlink-routerd`, or network access for ROS XML schema
validation. If a dependency is missing, report the missing environment
requirement instead of masking the failure.

Real-flight changes need explicit human validation on the target hardware.
Never claim hardware validation unless the command was run on the relevant
vehicle, companion computer, and ground-station setup.

## Documentation Notes

- The root `README.md` follows the Best-README-Template rhythm used by ACESim
  while keeping ACEPliot-specific Chinese content.
- Update `README.md` when adding or removing ROS packages, launch files, config
  files, user-facing commands, default topics, ports, model paths, dependencies,
  or real-flight procedures.
- Keep the distinction clear between PX4-side `uxrce_dds_client` and
  companion-computer-side `MicroXRCEAgent`.
- Document changes to AirLink, MAVLink ports, PX4 external modes, trajectory
  generation, state conversion, and test commands in the same change that
  modifies the behavior.
- Keep this guide environment-independent. Project rules should not depend on a
  current username, shell, timezone, editor state, local IP address, or agent
  runtime implementation.
