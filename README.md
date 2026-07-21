<a id="readme-top"></a>

<div align="center">

<p align="center">
  <a href="https://ubuntu.com/"><img src="https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu" alt="Ubuntu 22.04" /></a>
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS%202-Humble-brightgreen.svg?logo=ros" alt="ROS 2 Humble" /></a>
  <a href="https://px4.io/"><img src="https://img.shields.io/badge/PX4-Autopilot-021A60.svg" alt="PX4 Autopilot" /></a>
  <img src="https://img.shields.io/badge/C%2B%2B-17-orange.svg" alt="C++17" />
  <img src="https://img.shields.io/badge/Python-3.10-blue.svg" alt="Python 3.10" />
</p>

<h1 align="center">ACEPliot</h1>

<p align="center">
  面向 PX4、ROS 2 与空中机械臂的控制、仿真和部署工作区。
</p>

</div>

<details>
  <summary>目录</summary>
  <ol>
    <li><a href="#项目简介">项目简介</a></li>
    <li><a href="#技术栈">技术栈</a></li>
    <li><a href="#快速开始">快速开始</a></li>
    <li><a href="#使用">使用</a></li>
    <li><a href="#贡献">贡献</a></li>
    <li><a href="#许可证">许可证</a></li>
    <li><a href="#联系">联系</a></li>
    <li><a href="#致谢">致谢</a></li>
  </ol>
</details>

## 项目简介

ACEPliot 是一个 ROS 2 Humble colcon workspace，用于组织 PX4 External
Modes、策略部署节点、轨迹源、可重复 benchmark、状态转换和真机通信工具。当前控制代码按
五层划分：

```text
libraries/               ROS 无关的通用 C++ 库
px4_external_modes/      由 PX4 External Mode 生命周期管理的低层控制模式
commanders/              普通 ROS 2 策略部署节点
trajectory_generators/   可独立组合的参考轨迹源
benchmarks/              可复现实验 workload、指标统计和报告生成
```

这个边界决定了进程所有权：External Mode 才能接管 PX4 模式生命周期；commander
只部署普通 ROS 控制节点；轨迹源只生成参考；benchmark 负责测试 workload 和报告。它们都
不负责启动 PX4、ACESim、ACETele 或 `MicroXRCEAgent`。

主要能力包括：

- `AM Position` body-rate/thrust 强化学习外部模式。
- Flying Hand 四旋翼和全驱动控制器，共享同一个 ROS package。
- ACELab AM EE Pose 上层策略部署与 ACETele 机械臂同步。
- PX4 速度跟踪、机械臂运动扰动和组合 benchmark。
- PX4 位置 8 字轨迹与 AM EE Pose 五帧目标预览。
- 自动生成 CSV、JSON 和 PNG 的轨迹与误差报告。
- PX4 / ROS 2 状态、IMU、外部里程计和 Ground Truth 转换。
- AirLink / MAVLink 地面站路由配置。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 技术栈

- Ubuntu 22.04
- ROS 2 Humble
- PX4 v1.14 或更高版本
- C++17、Python 3.10、CMake、ament_python、colcon
- ONNX Runtime
- ACADOS
- mavlink-router
- Micro XRCE-DDS

主要包：

| 层 | 包 | 用途 |
| --- | --- | --- |
| Library | `policy_inference` | ROS 无关的 ONNX 推理、张量与 RNN 状态管理 |
| PX4 mode | `am_position_mode` | 唯一的 AM Position body-rate/thrust 外部模式 |
| PX4 mode | `flying_hand_mode` | Flying Hand quadrotor 与 fully-actuated 控制器 |
| Commander | `am_ee_pose_commander` | ACELab AM EE Pose 上层策略与机械臂同步 |
| Trajectory | `figure8_trajectory` | PX4 位置 8 字和 EE 五帧预览 |
| Benchmark | `velocity_tracking_benchmark` | 分段 PX4 Offboard 速度跟踪 |
| Benchmark | `arm_motion_benchmark` | 机械臂运动时的飞机 base 稳定性 |
| Benchmark | `aerial_manipulation_benchmark` | 组合速度跟踪与机械臂运动 |
| Benchmark | `figure8_tracking_benchmark` | PX4 与 EE Figure-8 跟踪评测 |
| Benchmark | `benchmark_reporting` | 时间对齐、指标、CSV/JSON/PNG 报告 |
| Bridge | `px4_state_converter` | PX4 / ROS 2 状态与里程计转换 |

第三方依赖位于 `third_party/`。`px4_msgs`、PX4 ROS 2 Interface、ONNX
Runtime vendor、ACADOS 和 mavlink-router 均应按各自子模块或 vendor 包管理。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 快速开始

克隆并初始化子模块：

```bash
git clone git@github.com:Xiangyuan-Xie/ACEPliot.git
cd ACEPliot
git submodule update --init --recursive
```

安装工作区依赖：

```bash
cd <ACEPliot_ROOT>
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

构建并加载环境：

```bash
cd <ACEPliot_ROOT>
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

只构建本次控制架构涉及的包：

```bash
colcon build --packages-select \
  policy_inference am_position_mode flying_hand_mode am_ee_pose_commander \
  figure8_trajectory benchmark_reporting velocity_tracking_benchmark \
  arm_motion_benchmark aerial_manipulation_benchmark figure8_tracking_benchmark
```

AirLink 工具需要 `mavlink-routerd`：

```bash
git submodule update --init --recursive third_party/mavlink-router
./tools/airlink/install_mavlink_router.sh
```

真机 ROS 2 通信还需要机载电脑端的 agent。建议单独启动且只运行一个实例：

```bash
MicroXRCEAgent udp4 -p 8888
```

PX4 固件端进程是 `uxrce_dds_client`，机载电脑端进程是
`MicroXRCEAgent`，两者不是同一个组件。真机 launch 默认不拥有 agent 生命周期；仅在
明确需要时传入 `start_micro_xrce_agent:=true`。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 使用

### ACESim 对接

ACEPliot 的 sim launch 不启动 ACESim。先启动 ACESim ROS 2 bridge，再启动所需
benchmark、commander 或 External Mode：

```bash
# ACESim 终端
ros2 launch acesim_ros2 linux.launch.py ace_follower:=auto
```

仿真接口默认使用：

- `/acesim/clock`
- `/acesim/vehicle/odometry`
- `/ace_follower/arm/state`
- `/ace_follower/arm/sync_status`
- `/ace_follower/gripper/state`

Python trajectory 和 benchmark 的仿真入口只随 `/acesim/clock` 推进，不使用 wall-time
fallback。未收到仿真时钟时节点保持等待，不会提前消耗 profile 时间。

### AM Position

`am_position_mode` 只有一个 PX4 模式，显示名为 `AM Position`，输出完整的 body
rate 与 collective thrust，不再提供旧式控制变体或语义别名。

```bash
ros2 launch am_position_mode sim_am_position.launch.py \
  model_path:=<ABSOLUTE_POLICY_ONNX> \
  metadata_path:=<ABSOLUTE_POLICY_JSON>
```

```bash
ros2 launch am_position_mode real_am_position.launch.py \
  model_path:=<ABSOLUTE_POLICY_ONNX> \
  metadata_path:=<ABSOLUTE_POLICY_JSON>
```

部署 JSON 必须声明：

```json
{
  "action_semantics": "body_rate_thrust_raw",
  "body_frame": "FLU",
  "publish_frame": "FRD",
  "collective_preprocess": "sigmoid_2x",
  "max_body_rate_rad_s": 6.0
}
```

参数 `collective_scale` 控制归一化 collective 的缩放。旧元数据 token 不再接受；
ONNX 张量本身不会因为元数据字符串迁移而改变。

### Flying Hand

`flying_hand_mode` 包含两个相互独立的控制器，并复用包内 runtime：

```bash
ros2 launch flying_hand_mode sim_flying_hand_quadrotor.launch.py
ros2 launch flying_hand_mode real_flying_hand_quadrotor_shadow.launch.py
ros2 launch flying_hand_mode real_flying_hand_quadrotor.launch.py
```

全驱动模式默认从 shadow 开始：

```bash
ros2 launch flying_hand_mode real_flying_hand_fully_actuated_shadow.launch.py
```

闭环必须显式提供经过标定的 YAML：

```bash
ros2 launch flying_hand_mode real_flying_hand_fully_actuated.launch.py \
  config_file:=<ABSOLUTE_CALIBRATED_YAML>
```

全驱动配置需要同时满足 `closed_loop: true` 和
`calibration_confirmed: true`。旋翼几何、轴向、推力曲线、执行器顺序和 PX4
`CA_ROTOR*` 参数共同构成标定契约。该实现根据公开论文重建，不是官方源码移植，也不应
在没有硬件标定与 shadow 验证时直接闭环。

两个控制器共享 `/ace_leader/*` 和 `/ace_follower/*`，因此不能同时运行。

### Figure-8 Trajectory

向 PX4 发布位置 8 字参考：

```bash
ros2 launch figure8_trajectory px4_figure8_trajectory.launch.py
```

该轨迹源和对应的 `figure8_tracking_benchmark` 只发布位置参考并记录结果，不负责启动
ACESim、切换 PX4 到 AM Offboard 或构建 PX4 固件。使用 AM Offboard 跟踪前，需要先在
ACESim 中启动 PX4 并进入 AM Offboard；修改固件后应重新构建其 SITL：

```bash
cd <ACESim_ROOT>/components/upstream/PX4-Autopilot
make px4_sitl_default
```

AM Offboard 根据 `OffboardControlMode` 选择位置或速度输入；位置优先且不会把
`TrajectorySetpoint.velocity` 当作位置模式的策略前馈。

为 AM EE Pose 发布五帧目标预览：

```bash
ros2 launch figure8_trajectory ee_figure8_trajectory.launch.py
```

统一轨迹定义为：

```text
x = x0 + amplitude_x_m * sin(phase)
y = y0 + amplitude_y_m * sin(2 * phase)
```

`amplitude_y_m` 是实际 Y 峰值。启动时会校验理论最大速度
`omega * sqrt(Ax^2 + 4*Ay^2)` 不超过 `max_linear_speed_m_s`。EE 默认参数为
`period_s=20`、`transition_time_s=5`、`amplitude_x_m=0.5`、
`amplitude_y_m=0.25`；渐入作用于相位速度，避免额外的振幅变化速度峰值。有限循环结束后
参考回到原点并自动结束；`loops_to_run=0` 表示无限运行。

EE 预览继续发布到 `/am_ee_pose/trajectory_preview`，采样时刻为
`0/0.02/0.04/0.06/1.0 s`。

### AM EE Pose

`am_ee_pose_commander` 是普通 ROS 2 节点，不是 PX4 External Mode。它运行 ACELab
上层策略，向 PX4 固件内的 AM Offboard 低层模式发送参考，并通过 ACETele 协议控制机械臂。

```bash
ros2 launch am_ee_pose_commander sim_am_ee_pose_commander.launch.py \
  upper_model_path:=<ABSOLUTE_UPPER_ONNX>
```

```bash
ros2 launch am_ee_pose_commander real_am_ee_pose_commander.launch.py \
  upper_model_path:=<ABSOLUTE_UPPER_ONNX>
```

shell 不会展开参数值中间的字面量 `~`，因此模型路径应使用绝对路径。若 ONNX 使用
external data，模型引用的 `.onnx.data` 文件也必须与模型保持导出时的相对位置。
仓库不附带策略权重，也不会隐式下载外部权重。

没有 `/am_ee_pose/trajectory_preview` 输入时，commander 保持启动时的 EE 位置。使用
8 字目标进行测试时可直接使用 EE tracking benchmark。它会启动 EE 轨迹源、
`am_ee_pose_commander` 和 reporter，但不会启动 ACESim 或 PX4：

```bash
ros2 launch figure8_tracking_benchmark sim_ee_figure8_tracking_benchmark.launch.py \
  upper_model_path:="$HOME/policy.onnx"
```

### Benchmarks

PX4 分段速度命令：

```bash
ros2 launch velocity_tracking_benchmark sim_velocity_tracking_benchmark.launch.py
ros2 launch velocity_tracking_benchmark real_velocity_tracking_benchmark.launch.py
```

机械臂轨迹命令：

```bash
ros2 launch arm_motion_benchmark sim_arm_motion_benchmark.launch.py
ros2 launch arm_motion_benchmark real_arm_motion_benchmark.launch.py
```

机械臂 waypoint 使用 `segment_durations_s` 线性插值并按 100 Hz 发布。
`segment_durations_s[0]` 表示当前 Follower 状态到首个 waypoint，后续元素表示 waypoint
之间的时间。`loop_count=0` 表示无限循环，正整数表示执行次数。该 benchmark 只评价机械臂
动作期间的飞机 base 漂移，不报告关节或夹爪跟踪误差。

组合启动飞行速度与机械臂轨迹：

```bash
ros2 launch aerial_manipulation_benchmark sim_aerial_manipulation_benchmark.launch.py
ros2 launch aerial_manipulation_benchmark real_aerial_manipulation_benchmark.launch.py
```

PX4 和 EE Figure-8 跟踪入口：

```bash
ros2 launch figure8_tracking_benchmark sim_px4_figure8_tracking_benchmark.launch.py
ros2 launch figure8_tracking_benchmark real_px4_figure8_tracking_benchmark.launch.py
```

有限 benchmark 完成后会自动关闭并写出报告。`Ctrl-C` 结束无限测试时仍会生成
`incomplete` 报告。默认在工作区源码中的对应 benchmark 包内创建 `logs/`，每次运行使用
独立的 UTC 时间戳目录：

```text
benchmarks/<benchmark_package>/logs/<UTC_TIMESTAMP>/
```

可用 `output_dir:=<ABSOLUTE_OUTPUT_DIRECTORY>` 覆盖。报告包含 `summary.json`、对齐后的
CSV 样本，以及同名的 PNG、SVG、PDF 图组。空间轨迹图使用等物理尺度的 ENU 3D 主图和
XY/XZ/YZ 投影，每次运行还会生成 `overview` 汇总图。速度测试报告三轴和三维速度
RMSE/MAE/最大误差；机械臂测试报告 base 的逐轴、水平和三维漂移；Figure-8 报告位置、
速度以及 EE 姿态角误差。

### PX4 State Converter

向 PX4 发送外部里程计：

```bash
ros2 launch px4_state_converter generic_odometry.launch.py
```

导出 PX4 odometry 或 Ground Truth：

```bash
ros2 launch px4_state_converter odometry.launch.py
ros2 launch px4_state_converter gt_odometry.launch.py
```

这些真机 launch 默认不启动 `MicroXRCEAgent`。需要旧式 launch-owned 行为时才传入
`start_micro_xrce_agent:=true`。

### AirLink

配置 MAVLink 路由：

```bash
./tools/airlink/configure_airlink.sh
sudo journalctl -u mavlink-router -f
```

默认约定：

| 接口 | 默认值 |
| --- | --- |
| Micro XRCE-DDS Agent | UDP `8888` |
| PX4 MAVLink UDP listen | `14540` |
| QGroundControl UDP | `14550` |
| AirLink serial baud | `921600` |

MAVLink 和 uXRCE-DDS 是两条不同的数据链路。QGC 断连时应分别检查
`mavlink-routerd`、UDP `14540/14550` 流量，以及是否意外启动了多个 agent。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 贡献

从最新主分支创建功能分支，并让每个提交只覆盖一个逻辑变更。提交信息使用与历史一致的
Conventional Commit 风格和明确 scope，例如：

```text
refactor(am-position): merge policy runtime into mode
refactor(flying-hand): consolidate controller packages
feat(benchmark): add figure-eight tracking reports
fix(airlink): preserve qgc udp forwarding
docs(repo): document control-layer ownership
```

提交前至少运行相关包测试和仓库静态测试：

```bash
python3 -m unittest discover -s tests
colcon test --packages-select <CHANGED_PACKAGES> --event-handlers console_direct+
colcon test-result --verbose
pre-commit run --all-files
```

修改子模块时，先在子模块仓库内提交，再在 ACEPliot 中提交 gitlink。不要提交只有本机可见的
dirty submodule 状态、模型权重、rosbag、飞行日志、`build/`、`install/` 或 `log/`。

Pull Request 应说明行为变化、涉及包、验证命令，以及话题、坐标系、端口、配置格式和真机
安全边界是否改变。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 许可证

本项目当前主要面向科研与其他非商业用途开放。第三方依赖的授权条款以 `third_party/` 中
各模块的原始许可证与声明为准；实际使用前请结合具体场景评估合规要求。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 联系

项目维护者：Xiangyuan Xie

项目链接：<https://github.com/Xiangyuan-Xie/ACEPliot>

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 致谢

- [ROS 2](https://docs.ros.org/)
- [PX4 Autopilot](https://px4.io/)
- [PX4 ROS 2 Interface Library](https://github.com/PX4/px4-ros2-interface-lib)
- [ONNX Runtime](https://onnxruntime.ai/)
- [ACADOS](https://github.com/acados/acados)
- [mavlink-router](https://github.com/mavlink-router/mavlink-router)
- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- [ACESim](https://github.com/Xiangyuan-Xie/ACESim)

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>
