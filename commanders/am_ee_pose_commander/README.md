# AM EE Pose Commander

`am_ee_pose_commander` deploys the upper policy from ACELab task
`Isaac-AM-EE-Pose-v0` as a normal ROS 2 command node. It is not a PX4 External
Mode and does not publish motor commands.

The execution chain is:

```text
EE target preview -> AM EE Pose upper policy -> velocity + arm commands
                                           |-> PX4 AM Offboard
                                           `-> ACETele follower
```

PX4 firmware owns the 100 Hz AM Position low-level policy. This package only
loads the 63-observation, 8-action upper ONNX model at 50 Hz. The first four
actions become heading-frame velocity commands for `/fmu/in/*`; the last four
actions become accumulated `joint_1..joint_4` targets after the ACETele
Leader/Follower handshake reaches `tracking`.

## Launch

Start PX4 with its `am_pos_control` module, switch to the firmware-native
`AM Offboard` mode, and run one commander:

```bash
ros2 launch am_ee_pose_commander sim_am_ee_pose_commander.launch.py \
  upper_model_path:=<ABSOLUTE_UPPER_ONNX>
```

`upper_model_path` is required and must be absolute. ROS launch arguments do not
expand `~`; use `$HOME/policy.onnx` from a shell instead. This repository does
not bundle policy weights. If an ONNX export uses external tensor data, keep its
`.data` sidecar beside the model with the original filename. Missing sidecars are
reported directly through the ONNX Runtime load error.

Use `real_am_ee_pose_commander.launch.py` for real flight. It does not start
`MicroXRCEAgent` unless `start_micro_xrce_agent:=true` is explicitly provided.

The target input `/am_ee_pose/trajectory_preview` is a
`geometry_msgs/msg/PoseArray` with exactly five world-frame ENU poses at
`t`, `t+0.02`, `t+0.04`, `t+0.06`, and `t+1.0 s`. The companion Figure-8 target
publisher is:

```bash
ros2 launch figure8_trajectory ee_figure8_trajectory.launch.py
```

The commander does not require a separate target publisher to hold position.
Once PX4 and follower states are available, it locks the current world-frame EE
pose and repeats it across all five preview samples. A fresh external preview
temporarily overrides that startup hold target; if the preview stops, the
commander returns to the locked startup pose.

The Figure-8 is horizontal in the world ENU `x-y` plane: `z` and EE orientation
remain fixed at their startup values.

The Figure-8 target follows `/acesim/clock` by default. For a real-time target
source, pass `use_sim_clock:=false`.

Do not run `px4_figure8_trajectory` or another process that publishes
`/fmu/in/offboard_control_mode` and `/fmu/in/trajectory_setpoint` at the same
time. `px4_figure8_trajectory` is a direct PX4 reference publisher;
`ee_figure8_trajectory` is the target source intended for this commander.
