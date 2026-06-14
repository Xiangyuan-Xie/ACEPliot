<a id="readme-top"></a>

<div align="center">

<p align="center">
  <a href="https://ubuntu.com/"><img src="https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu" alt="Ubuntu 22.04" /></a>
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS%202-Humble-brightgreen.svg?logo=ros" alt="ROS 2 Humble" /></a>
  <a href="https://px4.io/"><img src="https://img.shields.io/badge/PX4-Autopilot-021A60.svg" alt="PX4 Autopilot" /></a>
  <a href="https://docs.ros.org/en/rolling/Installation/Alternatives/Ubuntu-Development-Setup.html"><img src="https://img.shields.io/badge/build-colcon-orange.svg" alt="colcon workspace" /></a>
</p>

<h1 align="center">ACEPliot</h1>

<p align="center">
  面向 PX4 与 ROS 2 的无人机控制工作区。
</p>

</div>

<details>
  <summary>目录</summary>
  <ol>
    <li><a href="#项目简介">项目简介</a></li>
    <li><a href="#技术栈">技术栈</a></li>
    <li>
      <a href="#快速开始">快速开始</a>
      <ul>
        <li><a href="#环境要求">环境要求</a></li>
        <li><a href="#安装">安装</a></li>
        <li><a href="#构建">构建</a></li>
      </ul>
    </li>
    <li>
      <a href="#使用">使用</a>
      <ul>
        <li><a href="#rl-控制模式">RL 控制模式</a></li>
        <li><a href="#trajectory_generators">trajectory_generators</a></li>
        <li><a href="#px4_state_converter">px4_state_converter</a></li>
        <li><a href="#airlink-地面站链路">AirLink 地面站链路</a></li>
        <li><a href="#配置与默认约定">配置与默认约定</a></li>
      </ul>
    </li>
    <li><a href="#贡献">贡献</a></li>
    <li><a href="#许可证">许可证</a></li>
    <li><a href="#联系">联系</a></li>
    <li><a href="#致谢">致谢</a></li>
  </ol>
</details>

## 项目简介

ACEPliot 是一个面向 PX4 与 ROS 2 的工程化无人机控制工作区，服务于控制算法研究、仿真验证与真机部署。它把 PX4 External Flight Modes、强化学习控制模式、状态转换、轨迹生成和真机地面站链路放在统一的 colcon workspace 中，便于在同一套工程组织下完成仿真到实机的迭代。

当前仓库主要包含以下能力：

- 基于 PX4 External Flight Modes 的多旋翼强化学习控制模式。
- PX4 与 ROS 2 之间的状态、IMU、外部里程计和 Ground Truth 里程计转换。
- 基于轨迹生成器的 Offboard 参考指令发布。
- 面向真机 MAVLink / AirLink 链路的配置工具。
- 可选的 uXRCE-DDS 通信入口，用于连接 PX4 端 `uxrce_dds_client` 与机载电脑端 `MicroXRCEAgent`。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 技术栈

- [Ubuntu 22.04](https://ubuntu.com/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [PX4 Autopilot](https://px4.io/)
- [PX4 ROS 2 Interface Library](https://github.com/PX4/px4-ros2-interface-lib)
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [ONNX Runtime](https://onnxruntime.ai/)
- [mavlink-router](https://github.com/mavlink-router/mavlink-router)
- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)

主要 ROS 2 包：

| 包 | 用途 |
| --- | --- |
| `rl_base_mode` | RL 外部飞行模式基础运行库、观测缓存、ONNX 推理、日志工具 |
| `am_position_mode` | 多旋翼机械臂位置控制模式，提供 Motor 与 CTBR 两类入口 |
| `px4_state_converter` | PX4 / ROS 2 状态、IMU、里程计转换 |
| `figure8_trajectory_mode` | 8 字轨迹位置/速度 Offboard 参考生成 |

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 快速开始

### 环境要求

本文档默认系统已经具备以下基础环境：

- Ubuntu 22.04。
- ROS 2 Humble。
- PX4 v1.14 或更高版本。
- C++17 编译工具链。

如果尚未完成 ROS 2 或 PX4 的安装，建议先按照各自官方文档完成基础环境部署，再继续进行本仓库的依赖安装与构建。

### 安装

克隆仓库并初始化子模块：

```bash
git clone git@github.com:Xiangyuan-Xie/ACEPliot.git
cd ACEPliot
git submodule update --init --recursive
```

安装基础系统依赖：

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

安装工作区 ROS 依赖：

```bash
cd <ACEPliot_ROOT>
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

`AirLink` 相关工具依赖 `mavlink-routerd`。本仓库将官方 `mavlink-router` 固定为 `third_party/mavlink-router` 子模块，并通过源码构建安装：

```bash
cd <ACEPliot_ROOT>
git submodule update --init --recursive third_party/mavlink-router
./tools/airlink/install_mavlink_router.sh
which mavlink-routerd
```

真机 ROS 2 / PX4 通信还需要机载电脑端的 `MicroXRCEAgent`。仓库中的真机相关 launch 默认不会自动启动 `MicroXRCEAgent`；推荐在单独终端中只启动一个 agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

如需恢复由 launch 文件代为启动的旧行为，可在对应 launch 命令中显式传入：

```bash
start_micro_xrce_agent:=true
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
which MicroXRCEAgent
```

### 构建

编译整个工作区：

```bash
cd <ACEPliot_ROOT>
source /opt/ros/humble/setup.bash
colcon build
```

载入工作区环境：

```bash
source <ACEPliot_ROOT>/install/setup.bash
```

只编译相关包时，优先使用包名限定范围：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select am_position_mode
```

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 使用

本仓库当前主要包含四类使用入口：

- `am_position_mode`：强化学习控制模式。
- `trajectory_generators`：轨迹生成与 Offboard 参考发布。
- `px4_state_converter`：PX4 与 ROS 2 之间的状态、IMU、里程计转换。
- `tools/airlink`：真机 MAVLink 地面站链路配置。

### RL 控制模式

`am_position_mode` 当前提供四个 launch 入口，覆盖仿真与真机两类场景，以及 Motor / CTBR 两种控制输出形式。

#### 仿真

Motor 版本：

```bash
ros2 launch am_position_mode sim_am_position_motor.launch.py
```

CTBR 版本：

```bash
ros2 launch am_position_mode sim_am_position_ctbr.launch.py
```

这两个仿真 launch 默认读取各自对应的 YAML 配置，并支持覆盖以下参数：

| 参数 | 说明 |
| --- | --- |
| `config_file` | launch 使用的 YAML 配置文件 |
| `model_path` | ONNX 策略模型路径 |
| `use_sim_time` | 是否使用仿真时间 |
| `sim_clock_topic` | 仿真时钟话题 |
| `use_ros2_odom` | 是否使用外部 ROS 2 里程计 |
| `offboard_control_mode_topic` | PX4 `OffboardControlMode` 输入话题 |
| `trajectory_setpoint_topic` | PX4 `TrajectorySetpoint` 输入话题 |
| `offboard_setpoint_timeout_s` | 外部 Offboard 参考超时时间 |

示例：覆盖模型路径与 Offboard 参考话题：

```bash
ros2 launch am_position_mode sim_am_position_motor.launch.py \
  model_path:=/absolute/path/to/policy.onnx \
  trajectory_setpoint_topic:=/fmu/in/trajectory_setpoint
```

#### 真机

Motor 版本：

```bash
ros2 launch am_position_mode real_am_position_motor.launch.py
```

CTBR 版本：

```bash
ros2 launch am_position_mode real_am_position_ctbr.launch.py
```

真机 launch 默认只启动 RL 模式节点，不自动启动机载电脑端的 `MicroXRCEAgent`。需要由 launch 代为启动时，显式传入：

```bash
ros2 launch am_position_mode real_am_position_motor.launch.py \
  start_micro_xrce_agent:=true
```

更推荐在单独终端中手动启动一次：

```bash
MicroXRCEAgent udp4 -p 8888
```

使用真机模式前请确认：

- PX4 端已正确启动 `uxrce_dds_client`。
- 机载电脑中已安装 `MicroXRCEAgent`。
- 机载电脑端已有一个 `MicroXRCEAgent udp4 -p 8888` 在运行，或本次 launch 显式传入了 `start_micro_xrce_agent:=true`。
- 机载电脑上已经运行过 `./tools/airlink/configure_airlink.sh`。

这两个真机 launch 支持覆盖以下参数：

| 参数 | 说明 |
| --- | --- |
| `config_file` | launch 使用的 YAML 配置文件 |
| `model_path` | ONNX 策略模型路径 |
| `use_ros2_odom` | 是否使用外部 ROS 2 里程计 |
| `offboard_control_mode_topic` | PX4 `OffboardControlMode` 输入话题 |
| `trajectory_setpoint_topic` | PX4 `TrajectorySetpoint` 输入话题 |
| `offboard_setpoint_timeout_s` | 外部 Offboard 参考超时时间 |
| `start_micro_xrce_agent` | 是否由 launch 启动 `MicroXRCEAgent udp4 -p 8888`，默认 `false` |

示例：覆盖 CTBR 模型路径：

```bash
ros2 launch am_position_mode real_am_position_ctbr.launch.py \
  model_path:=/absolute/path/to/policy.onnx
```

### trajectory_generators

`trajectory_generators/` 当前包含一个 ROS 2 包：

- `figure8_trajectory_mode`

该包内置共享的轨迹数据结构、PX4 Offboard 消息转换与状态读取工具，并提供位置模式与速度模式两个运行入口。

#### `figure8_position_mode`

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

该模式发布 `position` 与 `yaw`。公开参数包括 `figure8.*` 以及公共发布/状态参数，例如：

- `publish_rate_hz`
- `use_ros2_odom`
- `odom_topic`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `path_topic`
- `publish_path`
- `figure8.period_s`
- `figure8.target_height`

#### `figure8_velocity_mode`

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

该模式发布 `velocity` 与 `yaw_rate`。公开参数同样包括 `figure8.*` 以及公共发布/状态参数，例如：

- `publish_rate_hz`
- `use_ros2_odom`
- `odom_topic`
- `offboard_control_mode_topic`
- `trajectory_setpoint_topic`
- `path_topic`
- `publish_path`
- `figure8.period_s`
- `figure8.target_height`

### px4_state_converter

`px4_state_converter` 负责 PX4 与 ROS 2 之间的状态、IMU 与里程计转换。以下 launch 默认不会自动启动机载电脑端的 `MicroXRCEAgent`。需要 uXRCE-DDS 通信时，推荐先在单独终端中手动启动一次：

```bash
MicroXRCEAgent udp4 -p 8888
```

如需由 launch 文件代为启动，可显式传入 `start_micro_xrce_agent:=true`。

#### 向 PX4 发送外部里程计

通用外部里程计入口：

```bash
ros2 launch px4_state_converter generic_odometry.launch.py
```

默认配置文件为 `nokov_mocap_config.yaml`。如需切换配置，可执行：

```bash
ros2 launch px4_state_converter generic_odometry.launch.py \
  config_file:=fast_lio_config.yaml
```

#### 从 PX4 导出里程计

```bash
ros2 launch px4_state_converter odometry.launch.py
```

默认配置文件为 `lidar_config.yaml`。如需切换配置，可执行：

```bash
ros2 launch px4_state_converter odometry.launch.py \
  config_file:=lidar_config.yaml
```

#### 从 PX4 导出 Ground Truth 里程计

```bash
ros2 launch px4_state_converter gt_odometry.launch.py
```

同样支持通过 `config_file` 覆盖默认配置：

```bash
ros2 launch px4_state_converter gt_odometry.launch.py \
  config_file:=lidar_config.yaml
```

### AirLink 地面站链路

`tools/airlink/` 负责将 PX4 的 MAVLink 数据交给 `mavlink-routerd`，并以 UDP server 方式等待 QGroundControl 主动连接。GCS 先向机载电脑监听端口发出 UDP 包后，`mavlink-routerd` 会学习该源 IP / 端口，并把后续 MAVLink 数据回发到这个 GCS。

请先通过仓库子模块源码安装 `mavlink-router`：

```bash
cd <ACEPliot_ROOT>
git submodule update --init --recursive third_party/mavlink-router
./tools/airlink/install_mavlink_router.sh
```

随后执行以下脚本，交互式选择链路配置并写入 `/etc/mavlink-router/main.conf`：

```bash
./tools/airlink/configure_airlink.sh
```

仓库当前提供两种预设配置：

| 配置 | 用途 |
| --- | --- |
| `tools/airlink/configs/serial.yaml` | 机载电脑通过串口直连 PX4 |
| `tools/airlink/configs/udp.yaml` | PX4 通过 UDP 将 MAVLink 发送至机载电脑 |

如需排查链路问题，可直接查看服务日志：

```bash
sudo journalctl -u mavlink-router -f
```

QGroundControl 侧需要手动添加 UDP 链路，目标地址填写机载电脑 IP，端口填写 `14550`。不需要在 AirLink 配置里写地面站 IP；`gcs_listen` 会从 QGC 发来的第一包学习回发目标。

### 配置与默认约定

四个 RL launch 默认分别对应以下 YAML：

| launch | 默认配置 |
| --- | --- |
| `sim_am_position_motor.launch.py` | `px4_external_modes/am_position_mode/config/sim_am_position_motor.yaml` |
| `sim_am_position_ctbr.launch.py` | `px4_external_modes/am_position_mode/config/sim_am_position_ctbr.yaml` |
| `real_am_position_motor.launch.py` | `px4_external_modes/am_position_mode/config/real_am_position_motor.yaml` |
| `real_am_position_ctbr.launch.py` | `px4_external_modes/am_position_mode/config/real_am_position_ctbr.yaml` |

默认模型文件路径为：

```text
px4_external_modes/am_position_mode/weights/policy.onnx
```

可通过 launch 参数 `model_path` 传入绝对路径进行覆盖。

以下行为当前由 launch 文件或源码固定：

- `cmd_vel_timeout_s = 0.5`
- 真机相关 launch 提供 `start_micro_xrce_agent` 参数，默认不自动启动；显式启用时 `MicroXRCEAgent` 命令为 `udp4 -p 8888`
- `AirLink` 默认串口波特率固定为 `921600`
- `AirLink` 面向 QGroundControl 的 UDP 端口固定为 `14550`
- 当 `link_mode=udp` 时，PX4 侧 UDP 监听端口固定为 `14540`

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 贡献

欢迎提交 Issue、功能建议和 Pull Request。为便于维护和代码审查，建议每次贡献围绕一个边界清晰的改动展开，例如控制模式、状态转换、轨迹生成、真机链路、测试修复或文档更新。请避免在同一个分支或 PR 中混合多个无关改动。

### 分支与提交

建议从最新主分支创建功能分支：

```bash
git checkout master
git pull
git checkout -b feat/your-feature-name
```

提交信息建议使用简洁明确的格式，例如：

```text
feat(am-position): add ctbr safety limit
feat(trajectory): add figure-eight velocity option
fix(airlink): correct udp endpoint template
docs: update real-flight launch guide
test(px4-state): add odometry conversion regression
```

### 本地检查

提交 PR 前，建议至少运行相关包的构建和测试：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select am_position_mode
colcon test --packages-select am_position_mode --event-handlers console_direct+
colcon test-result --verbose
```

并执行代码质量检查：

```bash
pip install pre-commit
pre-commit run --all-files
```

如果改动涉及 ROS 2 launch、PX4 适配、AirLink、`MicroXRCEAgent`、真机控制或外部里程计，建议补充对应的最小启动验证，并在 PR 中说明运行命令和测试环境。

### 子模块修改

如果需要修改 `third_party/px4_msgs/`、`third_party/px4-ros2-interface-lib/`、`third_party/mavlink-router/` 或其他子模块，请先在对应子模块内完成修改并提交。随后回到 ACEPliot 父仓库，更新并提交对应的 gitlink。

请不要只在父仓库中提交子模块目录的未提交工作区状态，否则其他用户无法复现该修改。

### Pull Request

提交 PR 时，请简要说明：

- 本次改动的目的和背景。
- 修改涉及的主要文件、模块或 ROS 包。
- 已运行的测试、构建、启动或验证命令。
- 是否依赖 PX4、ROS 2、外部工具或特定硬件版本。
- 是否涉及配置格式、话题名、端口、坐标系、接口行为或数据格式变化。

这样可以帮助维护者更快地理解、复现和合并你的贡献。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 许可证

本项目当前主要面向科研与其他非商业用途开放。第三方依赖的授权条款请以 `third_party/` 中各模块的原始许可证与声明为准；在实际使用前，建议结合具体场景自行评估外部依赖带来的合规与授权要求。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 联系

项目维护者：Xiangyuan Xie

项目链接：<https://github.com/Xiangyuan-Xie/ACEPliot>

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 致谢

- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [PX4 Autopilot](https://px4.io/)
- [PX4 ROS 2 Interface Library](https://github.com/PX4/px4-ros2-interface-lib)
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [ONNX Runtime](https://onnxruntime.ai/)
- [mavlink-router](https://github.com/mavlink-router/mavlink-router)
- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>
