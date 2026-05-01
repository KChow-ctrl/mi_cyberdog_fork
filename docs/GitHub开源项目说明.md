# MiRoboticsLab GitHub 开源项目说明

> 来源: https://github.com/MiRoboticsLab

---

## 一、仓库列表

| 仓库 | 说明 | 许可 |
|------|------|------|
| [cyberdog_ros2](https://github.com/MiRoboticsLab/cyberdog_ros2) | ROS2 接口层，核心应用包 | Apache 2.0 |
| [cyberdog_ws](https://github.com/MiRoboticsLab/cyberdog_ws) | 预编译 workspace 镜像 | Apache 2.0 |
| [cyberdog_locomotion](https://github.com/MiRoboticsLab/cyberdog_locomotion) | 运动控制代码（MIT Cheetah） | Apache 2.0 |
| [cyberdog_motor_sdk](https://github.com/MiRoboticsLab/cyberdog_motor_sdk) | 电机 SDK，Mode 0/1 控制 | Apache 2.0 |
| [cyberdog_tegra_kernel](https://github.com/MiRoboticsLab/cyberdog_tegra_kernel) | NVIDIA Tegra Linux 内核/驱动 | GPL v2 |
| [cyberdog_simulator](https://github.com/MiRoboticsLab/cyberdog_simulator) | Gazebo 仿真 | Apache 2.0 |
| [cyberdog_vision](https://github.com/MiRoboticsLab/cyberdog_vision) | AI 视觉（Face/Human/Gesture） | Apache 2.0 |
| [cyberdog_miloc](https://github.com/MiRoboticsLab/cyberdog_miloc) | 定位/导航 | Apache 2.0 |
| [cyberdog_camera](https://github.com/MiRoboticsLab/cyberdog_camera) | 相机驱动（Argus） | Apache 2.0 |
| [devices](https://github.com/MiRoboticsLab/devices) | 外设驱动 | Apache 2.0 |
| [interaction](https://github.com/MiRoboticsLab/interaction) | 人机交互（语音/触摸） | Apache 2.0 |
| [utils](https://github.com/MiRoboticsLab/utils) | 通用工具库 | Apache 2.0 |
| [Cyberdog_MD](https://github.com/MiRoboticsLab/Cyberdog_MD) | 外观结构模型（STP 格式） | - |
| [model_files](https://github.com/MiRoboticsLab/model_files) | 模型文件 | Apache 2.0 |

---

## 二、cyberdog_ros2 详解

### 2.1 架构图

```
cyberdog_ros2/
├── 多设备连接
│   ├── wifirssi（WiFi 信号强度）
│   └── bluetooth（蓝牙）
├── 多模态感知
│   ├── cyberdog_body_state（姿态/速度）
│   ├── cyberdog_lightsensor（光感）
│   ├── cyberdog_obstacledetection（超声测障）
│   ├── cyberdog_scenedetection（场景检测）
│   └── cyberdog_camera（相机）
├── 多模态人机交互
│   ├── audio_assistant（集成小爱 SDK）
│   ├── cyberdog_audio（音频）
│   ├── cyberdog_vision（AI 视觉）
│   └── cyberdog_touch（触摸）
├── 自主决策
│   ├── cyberdog_decision（决策系统）
│   ├── cyberdog_led（LED 灯效）
│   └── cyberdog_lightsensor
├── 空间定位与导航
│   ├── cyberdog_laserslam（激光 SLAM）
│   └── cyberdog_miloc（定位）
└── cyberdog_utils / cyberdog_decisionutils
    └── cascade_manager（层级管理器，继承 LifecycleNode）
```

### 2.2 DDS 配置

**默认 DDS**: CycloneDDS
**ROS_DOMAIN_ID**: `42`（bringup 默认）
**WiFi 多播配置**: 指定 `NetworkInterfaceAddress` 为 WiFi IP

---

## 三、cyberdog_locomotion 详解

### 3.1 来源

基于 **MIT Cheetah Software** 开源项目构建。

### 3.2 支持的步态

通过 `checkout_gait` 切换：

| Gait ID | 名称 | 说明 |
|---------|------|------|
| 0 | TRANS | 过渡 |
| 1 | PASSIVE | 被动 |
| 2 | KNEEL | 跪下 |
| 3 | STAND_R | 站立（正面朝前） |
| 4 | STAND_B | 站立（背面朝前） |
| 5 | AMBLE | 慢行 |
| 6 | WALK | 步行 |
| 7 | SLOW_TROT | 慢速小跑 |
| 8 | TROT | 小跑 |
| 9 | FLYTROT | 飞奔 |
| 10 | BOUND | 跳跃 |
| 11 | PRONK | 跳跃 |

### 3.3 编译

**实机部署**（交叉编译）:
```bash
docker run -it --rm --name alan \
  -v your_path/cyberdog_locomotion:/work/build_farm/workspace/cyberdog \
  .net/athena/athena_cheetah_arm64:2.0 /bin/bash

cd /work/build_farm/workspace/cyberdog/
cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake \
  -DONBOARD_BUILD=ON -DBUILD_FACTORY=ON -DBUILD_CYBERDOG2=OFF ..
```

**仿真调试**（PC 本地）:
```bash
source /opt/ros/galactic/setup.bash
colcon build --merge-install --symlink-install \
  --packages-up-to cyberdog_locomotion cyberdog_ros2
```

⚠️ **注意**: 仅在 CyberDog 2 代上充分测试，1 代仅支持部分动作（除作揖和打滚外）。

---

## 四、cyberdog_motor_sdk 详解

### 4.1 Mode 0/1

| Mode | 值 | 说明 |
|------|-----|------|
| Mode 0 | `0` | 出厂默认，普通用户控制 |
| Mode 1 | `1` | 开发者模式，解锁所有高级功能 |

**配置文件**: `/mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt`

**Mode 1 解锁内容**:
- 12 个关节电机伺服控制
- 运动控制 SDK（libcyber_dog_motor_sdk.so）
- 高级步态（后空翻、遛狗模式等）
- 自定义控制算法部署

### 4.2 LCM 通信

LCM（Lightweight Communications and Marshalling）是运控板和 NX 之间的通信协议。

⚠️ **重要**: LCM 通信失败 = 电机无法激活！

```
症状: "Motor control mode has not been activated successfully."
```

---

## 五、贡献指南

### 5.1 Commit 格式

```
[提交类型]: 简明描述（80字符内）

JIRA-ID 或 N/A

进版原因（commit message，详细说明）
```

**提交类型**: `Fix`（Bug 修复）/ `New`（新功能）/ `Modify`（改进）

### 5.2 代码格式化

```bash
# 安装 clang-format
sudo apt install clang-format

# 格式化
clang-format -i file_to_format.cpp
```

---

## 六、相关资源

- 腾讯云开发社区: https://cloud.tencent.com/developer/article/1990634
- ROS2 Wiki 架构图: https://github.com/MiRoboticsLab/cyberdog_ros2/wiki
- 国内加速构建: `colcon build --cmake-args -DBUILD_INSIDE_GFW=ON`
