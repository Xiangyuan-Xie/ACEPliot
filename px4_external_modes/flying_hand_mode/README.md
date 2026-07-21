# Flying Hand External Modes

`flying_hand_mode` contains two PX4 ROS 2 External Modes that share one runtime
while retaining separate control models and ACADOS solvers:

- **Flying Hand Quadrotor** is the existing underactuated x500 adaptation.
- **Flying Hand Fully Actuated** reconstructs the paper architecture for a
  calibrated tilted-rotor hexarotor.

The common runtime owns PX4 mode lifecycle, safety checks, ACETele handshake,
diagnostics, fallback, and wrench publication. The controller implementations
remain isolated under `quadrotor` and `fully_actuated`; generated solver symbols
and numerical behavior are unchanged by the package merge.

## Launch

```bash
ros2 launch flying_hand_mode sim_flying_hand_quadrotor.launch.py
ros2 launch flying_hand_mode real_flying_hand_quadrotor_shadow.launch.py
ros2 launch flying_hand_mode real_flying_hand_quadrotor.launch.py
ros2 launch flying_hand_mode real_flying_hand_fully_actuated_shadow.launch.py
ros2 launch flying_hand_mode real_flying_hand_fully_actuated.launch.py \
  config_file:=/absolute/path/to/calibrated_fully_actuated.yaml
```

The fully actuated closed-loop launch has no default configuration. It requires
an explicitly supplied YAML with both `closed_loop: true` and
`calibration_confirmed: true`. The checked-in fully actuated configuration is
shadow-only and contains nominal geometry that is not a flight calibration.

Both modes use the same `/ace_leader/*` and `/ace_follower/*` protocol and must
not run concurrently. Their mode-specific topics remain under
`/flying_hand_quadrotor/*` and `/flying_hand_fully_actuated/*`.

## Build And Test

```bash
colcon build --packages-select flying_hand_mode
colcon test --packages-select flying_hand_mode --event-handlers console_direct+
```

Detailed controller, safety, frame, and code-generation notes are in
[`docs/quadrotor.md`](docs/quadrotor.md) and
[`docs/fully_actuated.md`](docs/fully_actuated.md).
