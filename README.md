# ACEPliot

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](#)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg?logo=ros)](#)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-021A60.svg?logo=px4)](#)

ACEPliot 是一个面向 PX4 与 ROS 2 的工程化无人机控制工作区，面向控制算法研究、仿真验证与真机部署场景，提供统一的代码组织、依赖管理与运行入口。

当前仓库主要包含以下能力：

- 基于 PX4 External Flight Modes 的多旋翼强化学习控制模式
- PX4 与 ROS 2 之间的状态、IMU 与外部里程计转换节点
- 面向真机通信链路的 `AirLink` 与 `MicroXRCEAgent` 启动入口
- 基于轨迹生成器的 Offboard 参考指令发布能力

---

- [ACEPliot](#acepliot)
  - [项目结构](#项目结构)
  - [环境要求](#环境要求)
  - [依赖安装](#依赖安装)
  - [构建工作区](#构建工作区)
  - [使用说明](#使用说明)
  - [RL 控制模式](#rl-控制模式)
  - [trajectory_generators](#trajectory_generators)
  - [px4_state_converter](#px4_state_converter)
  - [AirLink 地面站链路](#airlink-地面站链路)
  - [配置与默认约定](#配置与默认约定)
  - [许可证](#许可证)

---

## 项目结构

当前仓库中与使用最相关的目录如下：

```text
ACEPliot/
├── px4_external_modes/
│   ├── rl_base_mode/                  # RL 模式基类与共享机器人状态抽象
│   ├── rl_mode_utils/                 # ONNX 推理、日志、轨迹生成等公共工具
│   └── am_position_mode/              # 多旋翼机械臂位置控制模式、launch、配置与模型权重
├── px4_state_converter/               # PX4/ROS 2 状态、里程计与 IMU 转换节点
├── trajectory_generators/
│   ├── trajectory_generator_utils/    # 轨迹生成相关公共数据结构与工具
│   └── figure8_trajectory_mode/       # 8 字轨迹位置/速度模式
├── tools/
│   └── airlink/                       # AirLink 链路配置脚本与预设配置
└── third_party/                       # px4_msgs、px4_ros2_cpp 等第三方依赖源码
```

## 环境要求

本文档默认系统已经具备以下基础环境：

- Ubuntu 22.04
- ROS 2 Humble
- PX4 v1.14 或更高版本

如果尚未完成 ROS 2 或 PX4 的安装，建议先按照各自官方文档完成基础环境部署，再继续进行本仓库的依赖安装与构建。

## 依赖安装

除基础环境外，首次使用前还需要补齐以下依赖。

### 1. 安装系统依赖

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

如果系统尚未初始化 `rosdep`，请先执行一次：

```bash
sudo rosdep init
rosdep update
```

### 2. 安装 `mavlink-routerd`

`AirLink` 相关工具依赖 `mavlink-routerd`。本仓库将官方 `mavlink-router` 固定为 `third_party/mavlink-router` 子模块，并通过源码构建安装；`mavlink-anywhere` 可作为安装思路参考，但不是本仓库的运行依赖。

首次 clone 仓库后请先初始化子模块：

```bash
git submodule update --init --recursive
```

随后运行 AirLink 安装脚本。该脚本会安装 `mavlink-router` 的构建依赖，初始化 `third_party/mavlink-router` 内部依赖，并执行 Meson/Ninja 源码安装：

```bash
./tools/airlink/install_mavlink_router.sh
```

安装完成后可执行以下命令确认 `mavlink-routerd` 是否可用：

```bash
which mavlink-routerd
```

### 3. 安装 `MicroXRCEAgent`

仓库中的多个 launch 文件会直接启动以下命令：

```bash
MicroXRCEAgent udp4 -p 8888
```

如果系统中尚未安装 `MicroXRCEAgent`，可按如下方式从源码安装：

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

安装完成后建议检查可执行文件是否已写入环境：

```bash
which MicroXRCEAgent
```

### 4. 安装工作区 ROS 依赖

`rl_mode_utils` 在编译阶段依赖 `onnxruntime_vendor`，此外工作区还依赖若干 ROS 2 包。建议直接在工作区根目录使用 `rosdep` 统一安装：

```bash
cd <ACEPliot_ROOT>
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

如果 ROS 2 环境已正确载入，`rosdep` 会尽量自动补齐例如 `Eigen3`、`cv_bridge`、`tf2_ros`、`message_filters`、`rosbag2_cpp`、`onnxruntime_vendor` 等依赖。

## 构建工作区

推荐按以下顺序完成首次构建。

### 1. 拉取并更新子模块

`third_party/` 中包含工作区依赖源码，拉取仓库后请先同步子模块：

```bash
cd <ACEPliot_ROOT>
git submodule update --init --recursive
```

### 2. 安装缺失依赖

```bash
cd <ACEPliot_ROOT>
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

### 3. 编译工作区

```bash
cd <ACEPliot_ROOT>
source /opt/ros/humble/setup.bash
colcon build
```

### 4. 载入工作区环境

```bash
source <ACEPliot_ROOT>/install/setup.bash
```

## 使用说明

本仓库当前主要包含四类使用入口：

- `am_position_mode`：强化学习控制模式
- `trajectory_generators`：轨迹生成与 Offboard 参考发布
- `px4_state_converter`：PX4 与 ROS 2 之间的状态/里程计转换
- `tools/airlink`：真机 MAVLink 地面站链路配置

下面按模块分别说明。

## RL 控制模式

`am_position_mode` 当前提供 4 个 launch 入口，覆盖仿真与真机两类场景，以及 `Motor` / `CTBR` 两种控制输出形式。

### 仿真

Motor 版本：

```bash
ros2 launch am_position_mode sim_am_position_motor.launch.py
```

CTBR 版本：

```bash
ros2 launch am_position_mode sim_am_position_ctbr.launch.py
```

这两个仿真 launch 默认读取各自对应的 YAML 配置，并支持覆盖以下参数：

- `config_file`
- `model_path`
- `use_sim_time`
- `sim_clock_topic`
- `use_ros2_odom`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `offboard_setpoint_timeout_s`

示例：覆盖模型路径与 Offboard 参考话题

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

使用真机模式前请确认：

- PX4 端已正确启动 `uxrce_dds_client`
- 机载电脑中已安装 `MicroXRCEAgent`
- 机载电脑上已经运行过 `./tools/airlink/configure_airlink.sh`

这两个真机 launch 支持覆盖以下参数：

- `config_file`
- `model_path`
- `use_ros2_odom`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `offboard_setpoint_timeout_s`

示例：覆盖模型路径

```bash
ros2 launch am_position_mode real_am_position_ctbr.launch.py \
  model_path:=/absolute/path/to/policy.onnx
```

## trajectory_generators

`trajectory_generators/` 当前包含两个 ROS 2 包：

- `trajectory_generator_utils`
- `figure8_trajectory_mode`

其中，`trajectory_generator_utils` 提供共享的轨迹数据结构、PX4 Offboard 消息转换与状态读取工具；`figure8_trajectory_mode` 提供位置模式与速度模式两个运行入口。

### `figure8_position_mode`

默认启动命令：

```bash
ros2 launch figure8_trajectory_mode figure8_position_mode.launch.py
```

默认发布以下话题：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/trajectory_generators/path`

默认配置文件：

- `trajectory_generators/figure8_trajectory_mode/config/figure8_position_mode.yaml`

该模式仅发布 `position` 与 `yaw`。公开参数包括 `figure8.*` 以及公共发布/状态参数，例如：

- `publish_rate_hz`
- `use_ros2_odom`
- `odom_topic`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `path_topic`
- `publish_path`
- `figure8.period_s`
- `figure8.target_height`

### `figure8_velocity_mode`

默认启动命令：

```bash
ros2 launch figure8_trajectory_mode figure8_velocity_mode.launch.py
```

默认发布以下话题：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/trajectory_generators/path`

默认配置文件：

- `trajectory_generators/figure8_trajectory_mode/config/figure8_velocity_mode.yaml`

该模式仅发布 `velocity` 与 `yaw_rate`。公开参数同样包括 `figure8.*` 以及公共发布/状态参数，例如：

- `publish_rate_hz`
- `use_ros2_odom`
- `odom_topic`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `path_topic`
- `publish_path`
- `figure8.period_s`
- `figure8.target_height`

## px4_state_converter

`px4_state_converter` 负责 PX4 与 ROS 2 之间的状态、IMU 与里程计转换。当前常用入口如下。

### 1. 向 PX4 发送外部里程计

通用外部里程计入口：

```bash
ros2 launch px4_state_converter generic_odometry.launch.py
```

默认配置文件为 `nokov_mocap_config.yaml`。如需切换配置，可执行：

```bash
ros2 launch px4_state_converter generic_odometry.launch.py \
  config_file:=fast_lio_config.yaml
```

### 2. 从 PX4 导出里程计

```bash
ros2 launch px4_state_converter odometry.launch.py
```

默认配置文件为 `lidar_config.yaml`。如需切换配置，可执行：

```bash
ros2 launch px4_state_converter odometry.launch.py \
  config_file:=lidar_config.yaml
```

### 3. 从 PX4 导出 Ground Truth 里程计

```bash
ros2 launch px4_state_converter gt_odometry.launch.py
```

同样支持通过 `config_file` 覆盖默认配置：

```bash
ros2 launch px4_state_converter gt_odometry.launch.py \
  config_file:=lidar_config.yaml
```

## AirLink 地面站链路

`tools/airlink/` 负责将 PX4 的 MAVLink 数据交给 `mavlink-routerd`，并以 UDP server 方式等待 QGroundControl 主动连接。GCS 先向机载电脑监听端口发出 UDP 包后，`mavlink-routerd` 会学习该源 IP/端口，并把后续 MAVLink 数据回发到这个 GCS。

请先通过仓库子模块源码安装 `mavlink-router`：

```bash
git submodule update --init --recursive third_party/mavlink-router
./tools/airlink/install_mavlink_router.sh
```

随后执行以下脚本，交互式选择链路配置并写入 `/etc/mavlink-router/main.conf`：

```bash
./tools/airlink/configure_airlink.sh
```

仓库当前提供两种预设配置：

- `tools/airlink/configs/serial.yaml`：机载电脑通过串口直连 PX4
- `tools/airlink/configs/udp.yaml`：PX4 通过 UDP 将 MAVLink 发送至机载电脑

如需排查链路问题，可直接查看服务日志：

```bash
sudo journalctl -u mavlink-router -f
```

QGroundControl 侧需要手动添加 UDP 链路，目标地址填写机载电脑 IP，端口填写 `14550`。不需要在 AirLink 配置里写地面站 IP；`gcs_listen` 会从 QGC 发来的第一包学习回发目标。

## 配置与默认约定

### RL 模式默认配置文件

四个 RL launch 默认分别对应以下 YAML：

- `px4_external_modes/am_position_mode/config/sim_am_position_motor.yaml`
- `px4_external_modes/am_position_mode/config/sim_am_position_ctbr.yaml`
- `px4_external_modes/am_position_mode/config/real_am_position_motor.yaml`
- `px4_external_modes/am_position_mode/config/real_am_position_ctbr.yaml`

默认模型文件路径为：

```text
px4_external_modes/am_position_mode/weights/policy.onnx
```

可通过 launch 参数 `model_path` 传入绝对路径进行覆盖。

### 当前代码中的固定约定

以下行为当前由 launch 文件或源码固定：

- `cmd_vel_timeout_s = 0.5`
- `MicroXRCEAgent` 启动命令固定为 `udp4 -p 8888`
- `AirLink` 默认串口波特率固定为 `921600`
- `AirLink` 面向 QGroundControl 的 UDP 端口固定为 `14550`
- 当 `link_mode=udp` 时，PX4 侧 UDP 监听端口固定为 `14540`

## 许可证

本项目当前主要面向科研与其他非商业用途开放。第三方依赖的授权条款请以 `third_party/` 中各模块的原始许可证与声明为准；在实际使用前，建议结合具体场景自行评估外部依赖带来的合规与授权要求。
