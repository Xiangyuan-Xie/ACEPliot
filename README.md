# ACEPliot

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](#) [![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg?logo=ros)](#) [![PX4](https://img.shields.io/badge/PX4-Autopilot-021A60.svg?logo=px4)](#)

ACEPliot 是一套面向 PX4 与 ROS 2 的工程化开源控制框架，主要集成了多旋翼强化学习控制策略部署、外部飞行模式（External Flight Modes）以及高精度的导航与里程计转换节点。本项目旨在为无人机控制算法的前沿研究与实际部署提供标准化的 ROS 2 开发底座。

---

- [ACEPliot](#acepliot)
  - [项目简介](#项目简介)
  - [功能特性](#功能特性)
  - [目录结构](#目录结构)
  - [快速开始](#快速开始)
    - [运行环境](#运行环境)
    - [基于 RL 的外部模式运行](#基于-rl-的外部模式运行)
  - [状态估计与里程计转换](#状态估计与里程计转换)
    - [1. 外部里程计接入 (ROS 2 -\> PX4)](#1-外部里程计接入-ros-2---px4)
    - [2. PX4 状态解算 (PX4 -\> ROS 2)](#2-px4-状态解算-px4---ros-2)
  - [许可证](#许可证)

---

## 项目简介

ACEPliot 旨在解决强化学习算法在多旋翼无人机上的系统集成与实机部署问题。框架在设计上解耦了策略推理与 PX4 底层控制接口，依赖 `px4-ros2-interface-lib` 与 `Micro XRCE-DDS Agent` 组件通信，从而保证飞行控制指令传输的稳定性与低延迟。

## 功能特性

- **强化学习策略原生支持**
  内置基于 ONNX Runtime 的推理引擎，支持直接加载 `.onnx` 模型网络。
- **高效的坐标系与里程计转换**
  提供完备的坐标系转换流水线，负责处理外部传感器（如 VIO、Lidar SLAM、动捕系统 Mocap）到 PX4 的数据融合，涵盖精确的平移与旋转外参校准，以及 ENU 与 NED、FLU 与 FRD 之间的自动化相互变换。
- **工程化与模块化设计**
  项目代码结构清晰，将算法推理与 ROS 2 相关接口层分离，具备良好的二次开发潜力与系统扩展性。

## 目录结构

项目包含三大核心模块：

```text
ACEPliot/
├── px4_external_modes/         # 强化学习外部飞行模式核心功能包
│   ├── rl_mode_utils/          # 通用的 ONNX 推理、日志与算子工具类
│   ├── rl_base_mode/           # 外部模式基类与公共逻辑
│   └── rl_mc_arm_position_mode/# 多旋翼带机械臂协同位置追踪模式
├── px4_state_converter/        # ROS 2 与 PX4 坐标系映射与状态数据桥接包
└── third_party/                # 第三方依赖包
    ├── px4_msgs/               # PX4 官方 ROS 2 消息定义
    ├── px4-ros2-interface-lib/ # PX4 C++ ROS 2 高级接口库
    └── onnxruntime_vendor/     # ONNX Runtime 依赖环境封装
```

## 快速开始

### 运行环境

部署本项目前，请确保目标机器的工作空间满足以下环境要求：

- **操作系统**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **PX4 Autopilot**: v1.14 及其以上或 main 分支（需包含 External Modes 特性）
- **中间件**: Micro XRCE-DDS Agent（用于处理 PX4 与 ROS 2 的底层通信）
- **系统依赖**: Eigen3, onnxruntime（后者可通过本项目的 vendor 包配置供给）

### 安装 Micro XRCE-DDS Agent

PX4 与 ROS 2 的底层通信依赖于 Micro XRCE-DDS Agent，可以通过以下步骤源码编译安装：

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### 基于 RL 的外部模式运行

以多旋翼带机械臂协同位置追踪模式（`arm_position`）为例，该模块提供了两种底层控制级输出方案：直接电机推力控制与角速率及推力控制。

启动直接电机推力模式（默认加载包内 `weights/policy.onnx` 模型）：
```bash
ros2 launch rl_mc_arm_position_mode sim_mc_arm_position_direct_actuators.launch.py
```

启动角速率与统一推力模式：
```bash
ros2 launch rl_mc_arm_position_mode sim_mc_arm_position_rates_thrust.launch.py
```

对于自定义模型，支持通过启动参数进行路径覆盖加载：
```bash
ros2 launch rl_mc_arm_position_mode sim_mc_arm_position_direct_actuators.launch.py model_path:=/your/absolute/path/policy.onnx
```

## 状态估计与里程计转换

`px4_state_converter` 子项目是本框架用来解决 ROS 2 与 PX4 异构坐标系通讯的一站式处理包。其核心围绕着两部分的数据转换逻辑：

### 1. 外部里程计接入 (ROS 2 -> PX4)
用于将外部运算节点（VIO、Lidar SLAM 或 Mocap）解析生成的全局位姿发送进入 PX4 飞控端状态估计器：
- 支持订阅 `nav_msgs/Odometry` 与 `geometry_msgs/PoseStamped` 等标准消息类型。
- 底层自动处理目标参考系的映射，完成由 ENU 转换到 NED 的换算。
- 可通过修改配置文件，直接加载和应用传感器的安装外参（平移补偿向量与旋转矫正矩阵）。

### 2. PX4 状态解算 (PX4 -> ROS 2)
将由飞控层原生下放的传感器数据拉取到并还原至 ROS 2 的标准参考系内，便于后续进行 Rviz2 可视化展现及其他基于局部地图的数据算法开发：
- **IMU 数据转换**：将 PX4 内部的体轴坐标系数据（FRD）转换为 ROS 2 的体轴坐标系（FLU）。
- **里程计转换**：将基于 NED 参考系封装的 `vehicle_odometry` 话题转换为带有标准 TF 连接树结构表现的 ENU 坐标格式表达。

## 许可证

本项目开源用于科研及其他非商业用途。代码遵循有关协议使用，具体条款请参考 `third_party` 中相关第三方模块的原始开源协议以及本仓库后续相关的声明约定。使用前请仔细评估外部依赖所带有的合规与授权限制。
