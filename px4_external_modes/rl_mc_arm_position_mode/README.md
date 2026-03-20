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
- `use_sim_time`：是否使用仿真时钟
- `use_ros2_odom`：是否使用外部 ROS2 里程计

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
