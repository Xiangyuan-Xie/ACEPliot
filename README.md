# ACEPliot

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](#) [![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg?logo=ros)](#) [![PX4](https://img.shields.io/badge/PX4-Autopilot-021A60.svg?logo=px4)](#)

ACEPliot 是一个面向 PX4 与 ROS 2 的工程化无人机控制工作区，当前主要包含三部分能力：

- 基于 PX4 External Flight Modes 的多旋翼强化学习控制模式
- PX4 与 ROS 2 之间的状态、IMU、外部里程计转换节点
- 真机链路相关的 `AirLink` 与 `MicroXRCEAgent` 启动入口

项目定位是给控制算法研究、仿真验证和真机部署提供一个统一的 ROS 2 工作区底座。

---

- [ACEPliot](#acepliot)
  - [环境要求](#环境要求)
  - [目录说明](#目录说明)
  - [额外依赖安装](#额外依赖安装)
  - [安装与构建](#安装与构建)
  - [RL 模式使用](#rl-模式使用)
  - [px4_state_converter 使用](#px4_state_converter-使用)
  - [配置与默认约定](#配置与默认约定)
  - [许可证](#许可证)

---

## 环境要求

本 README 默认你已经具备以下基础环境：

- Ubuntu 22.04
- ROS 2 Humble
- PX4 v1.14 或更新版本

如果你还没有安装 ROS 2 或 PX4，建议先完成各自官方安装流程，再继续下面的项目依赖安装和工作区构建。

## 目录说明

当前仓库中和使用最相关的目录如下：

```text
ACEPliot/
├── px4_external_modes/
│   ├── rl_base_mode/                  # RL 模式基类与共享机器人状态抽象
│   ├── rl_mode_utils/                 # ONNX 推理、日志、轨迹生成等工具库
│   └── am_position_mode/       # 多旋翼机械臂位置控制模式、launch、YAML、模型权重
├── px4_state_converter/               # PX4/ROS 2 状态、里程计、AirLink 相关节点与 launch
└── third_party/                       # px4_msgs、px4_ros2_cpp 等第三方依赖源码
```

## 额外依赖安装

除了 Ubuntu、ROS 2 和 PX4 基础环境外，这个工作区还依赖下面这些额外组件。

### 1. 安装系统依赖

先更新软件源并安装构建工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

如果你的机器还没有初始化 `rosdep`，先执行一次：

```bash
sudo rosdep init
rosdep update
```

### 2. 安装 `mavlink-routerd`

`AirLink` 现在通过仓库里的工具脚本配置 `mavlink-routerd`，但 `mavlink-router` 本身可以由用户自行安装。Ubuntu 22.04 上可直接执行：

```bash
sudo apt update
sudo apt install -y mavlink-router
```

安装完成后可以检查一下：

```bash
which mavlink-routerd
```

### 3. 安装 `MicroXRCEAgent`

本仓库多个 launch 会直接启动：

```bash
MicroXRCEAgent udp4 -p 8888
```

如果系统里没有 `MicroXRCEAgent`，需要先从源码安装：

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

安装完成后检查：

```bash
which MicroXRCEAgent
```

### 4. 安装工作区 ROS 依赖

`rl_mode_utils` 在编译时依赖 `onnxruntime_vendor`，此外还需要一组 ROS 2 包。推荐直接在工作区根目录用 `rosdep` 安装：

```bash
cd /path/to/ACEPliot
rosdep install --from-paths . --ignore-src -r -y
```

如果你已经 `source /opt/ros/humble/setup.bash`，`rosdep` 会自动尽量补齐例如 `Eigen3`、`cv_bridge`、`tf2_ros`、`message_filters`、`rosbag2_cpp`、`onnxruntime_vendor` 等依赖。

## 安装与构建

推荐按下面顺序完成首次构建。

### 1. 获取并更新子模块

仓库里的 `third_party` 包含工作区依赖源码，拉取后先同步：

```bash
cd /path/to/ACEPliot
git submodule update --init --recursive
```

### 2. 安装缺失依赖

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ACEPliot
rosdep install --from-paths . --ignore-src -r -y
```

### 3. 构建工作区

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ACEPliot
colcon build
```

### 4. 载入工作区环境

```bash
source /path/to/ACEPliot/install/setup.bash
```

## RL 模式使用

`am_position_mode` 当前提供 4 个 launch 入口，分别覆盖仿真/真机以及两种控制输出形式。

### 仿真

Motor 版本：

```bash
ros2 launch am_position_mode sim_am_position_motor.launch.py
```

CTBR 版本：

```bash
ros2 launch am_position_mode sim_am_position_ctbr.launch.py
```

这两个仿真 launch 默认读取各自对应的 YAML，并支持覆盖以下参数：

- `config_file`
- `model_path`
- `use_sim_time`
- `sim_clock_topic`
- `use_ros2_odom`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `offboard_setpoint_timeout_s`

例如覆盖模型路径和 Offboard 参考话题：

```bash
ros2 launch am_position_mode sim_am_position_motor.launch.py \
  model_path:=/absolute/path/to/policy.onnx \
  trajectory_setpoint_topic:=/fmu/in/trajectory_setpoint
```

### 真机

Motor 版本：

```bash
ros2 launch am_position_mode real_am_position_motor.launch.py
```

CTBR 版本：

```bash
ros2 launch am_position_mode real_am_position_ctbr.launch.py
```

真机 launch 会同时启动：

- RL 模式节点
- `MicroXRCEAgent`
- 预先配置好的 `mavlink-routerd`

真机使用前请确认：

- PX4 端已经启动 `uxrce_dds_client`
- 机载电脑上已经安装 `MicroXRCEAgent`
- 机载电脑上已经运行过 `./tools/airlink/configure_airlink.sh`

这两个真机 launch 支持覆盖以下参数：

- `config_file`
- `model_path`
- `use_ros2_odom`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `offboard_setpoint_timeout_s`

例如指定串口设备和模型路径：

```bash
ros2 launch am_position_mode real_am_position_ctbr.launch.py \
  model_path:=/absolute/path/to/policy.onnx
```

## px4_state_converter 使用

`px4_state_converter` 现在只负责状态和里程计转换节点。地面站链路改由 `tools/airlink/` 下的独立工具脚本维护。

## trajectory_generators 使用

`trajectory_generators/` 现在是父目录，下面拆成 3 个 ROS2 包：

- `trajectory_generator_utils`
- `figure8_trajectory_mode`
- `rc_manual_trajectory_mode`

其中 `trajectory_generator_utils` 提供共享的轨迹数据结构、PX4 Offboard 消息转换和状态读取工具；实际启动入口是后两个 mode 包。

### figure8_trajectory_mode

默认 launch：

```bash
ros2 launch figure8_trajectory_mode figure8_trajectory_mode.launch.py
```

默认会发布以下话题：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/trajectory_generators/path`

默认配置文件：

- `trajectory_generators/figure8_trajectory_mode/config/figure8_trajectory_mode.yaml`

公开参数为 `figure8.*` 以及公共发布/状态参数，例如：

- `publish_rate_hz`
- `use_ros2_odom`
- `odom_topic`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `path_topic`
- `publish_path`
- `figure8.period_s`
- `figure8.target_height`

### rc_manual_trajectory_mode

默认 launch：

```bash
ros2 launch rc_manual_trajectory_mode rc_manual_trajectory_mode.launch.py
```

默认会发布以下话题：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`

默认配置文件：

- `trajectory_generators/rc_manual_trajectory_mode/config/rc_manual_trajectory_mode.yaml`

公开参数为 `rc_manual.*` 以及公共发布/状态参数，例如：

- `publish_rate_hz`
- `use_ros2_odom`
- `odom_topic`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `rc_manual.v_xy`
- `rc_manual.v_up`
- `rc_manual.v_down`
- `rc_manual.acc_xy`
- `rc_manual.yaw_rate_max_deg`

### 1. AirLink 地面站链路

`tools/airlink/` 负责把 PX4 的 MAVLink 数据交给 `mavlink-routerd`，并以 UDP server 模式等待 QGroundControl 主动连接，不再使用广播。

先确保系统里已经安装 `mavlink-router`：

```bash
sudo apt update
sudo apt install -y mavlink-router
```

交互式选择仓库里的链路配置并写入 `/etc/mavlink-router/main.conf`：

```bash
./tools/airlink/configure_airlink.sh
```

仓库内当前提供两种场景配置：

- `tools/airlink/configs/serial.yaml`：机载电脑通过串口直连 PX4
- `tools/airlink/configs/udp.yaml`：PX4 通过 UDP 把 MAVLink 发到机载电脑
- `tools/airlink/configure_airlink.sh`：交互式选择配置，并重启 `mavlink-router` 服务

排障时可直接查看服务日志：

```bash
sudo journalctl -u mavlink-router -f
```

QGroundControl 侧请手动添加一个 UDP 链路，目标填机载电脑 IP，端口填 `14550`。

### 2. 外部里程计发送到 PX4

通用外部里程计入口：

```bash
ros2 launch px4_state_converter generic_odometry.launch.py
```

默认配置文件是 `nokov_mocap_config.yaml`。如需切换配置：

```bash
ros2 launch px4_state_converter generic_odometry.launch.py \
  config_file:=fast_lio_config.yaml
```

### 3. 从 PX4 导出里程计

```bash
ros2 launch px4_state_converter odometry.launch.py
```

默认配置文件是 `lidar_config.yaml`。如需切换配置：

```bash
ros2 launch px4_state_converter odometry.launch.py \
  config_file:=lidar_config.yaml
```

### 4. 从 PX4 导出 Ground Truth 里程计

```bash
ros2 launch px4_state_converter gt_odometry.launch.py
```

同样支持：

```bash
ros2 launch px4_state_converter gt_odometry.launch.py \
  config_file:=lidar_config.yaml
```

## 配置与默认约定

### RL 模式默认配置文件

4 个 RL launch 默认分别对应以下 YAML：

- `px4_external_modes/am_position_mode/config/sim_am_position_motor.yaml`
- `px4_external_modes/am_position_mode/config/sim_am_position_ctbr.yaml`
- `px4_external_modes/am_position_mode/config/real_am_position_motor.yaml`
- `px4_external_modes/am_position_mode/config/real_am_position_ctbr.yaml`

默认模型文件路径为：

```text
px4_external_modes/am_position_mode/weights/policy.onnx
```

`model_path` 支持通过 launch 参数传入绝对路径覆盖。

### 当前代码中的固定约定

以下行为当前由 launch 或源码固定：

- `cmd_vel_timeout_s = 0.5`
- `MicroXRCEAgent` 启动命令固定为 `udp4 -p 8888`
- `AirLink` 默认串口波特率固定为 `921600`
- `AirLink` 的 QGroundControl UDP 监听端口固定为 `14550`
- 当 `link_mode=udp` 时，PX4 UDP 监听端口固定为 `14540`

## 许可证

本项目开源用于科研及其他非商业用途。代码遵循有关协议使用，具体条款请参考 `third_party` 中相关第三方模块的原始开源协议以及本仓库后续相关的声明约定。使用前请仔细评估外部依赖所带有的合规与授权限制。
