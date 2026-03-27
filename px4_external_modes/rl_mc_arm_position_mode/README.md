# rl_mc_arm_position_mode

该包提供了两个机械臂跟踪飞行模式节点，二者共享同一套观测与推理流程，但控制输出不同。

## 节点与用途

- `rl_mc_arm_position_direct_actuators_mode`
  - 类型：直接电机推力输出
  - 对应模式类：`RlMCArmPositionDirectActuatorsMode`
  - 节点名：`rl_mc_arm_position_direct_actuators`
- `rl_mc_arm_position_rates_thrust_mode`
  - 类型：三轴角速率 + 统一推力输出
  - 对应模式类：`RlMCArmPositionRatesThrustMode`
  - 节点名：`rl_mc_arm_position_rates_thrust`

## 关键参数

通用参数：
- `model_path`：ONNX 模型路径
- `use_sim_time`：是否使用仿真时钟；为 `true` 时节点将订阅 `sim_clock_topic`
- `sim_clock_topic`：仿真时钟话题，默认 `/acesim/clock`
- `use_ros2_odom`：是否使用外部 ROS2 里程计
- `cmd_vel_topic`：外部速度命令话题（`geometry_msgs/TwistStamped`，默认 `/rl_arm_position/cmd_vel`）
- `cmd_vel_timeout_s`：外部速度命令超时秒数（默认 `0.5`）

命令输入来源固定为外部 `cmd_vel`。当外部命令超时后，模式会进入悬停锁定（锁当前位置与偏航，速度命令归零）。

仅 `rl_mc_arm_position_rates_thrust_mode` 使用：
- `max_body_rate_rad_s`：角速率缩放上限（rad/s）
- `max_collective_thrust`：统一推力缩放上限

## 启动示例

直接电机推力版本：

```bash
ros2 run rl_mc_arm_position_mode rl_mc_arm_position_direct_actuators_mode --ros-args -p model_path:=/abs/path/policy.onnx
```

角速率+统一推力版本：

```bash
ros2 run rl_mc_arm_position_mode rl_mc_arm_position_rates_thrust_mode --ros-args -p model_path:=/abs/path/policy.onnx
```

直接电机推力 launch：

```bash
ros2 launch rl_mc_arm_position_mode sim_mc_arm_position_direct_actuators.launch.py
```

角速率+统一推力 launch：

```bash
ros2 launch rl_mc_arm_position_mode sim_mc_arm_position_rates_thrust.launch.py
```

这两个仿真 launch 会默认传入：

```bash
use_sim_time:=true sim_clock_topic:=/acesim/clock
```

节点不再依赖把 `/clock` 重映射到 `/acesim/clock`。

外部速度命令示例（机体系，`x前/y左/z上`，`angular.z` 为偏航角速度）：

```bash
ros2 topic pub /rl_arm_position/cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 20
```
