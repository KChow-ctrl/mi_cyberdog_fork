# mi_cyberdog_fork

小米铁蛋 CyberDog 改进版 —— 集成腾讯云开发者文章 + GitHub 开源项目增强

## 仓库结构

```
mi_cyberdog_fork/
├── README.md
├── docs/
│   ├── 腾讯云文章笔记.md        # cloud.tencent.com 文章核心内容整理
│   └── GitHub开源项目说明.md    # MiRoboticsLab 各仓库说明
├── bridge_improved/
│   ├── bridge_node.cpp          # 改进版桥接器（修复 checkout_mode / LOW_BTR / spin 等）
│   ├── CMakeLists.txt
│   └── package.xml
└── SKILLS/
    └── cyberdog_dev_notes.md    # 开发经验沉淀
```

## 核心改进

### 1. checkout_mode 前置（关键）
所有动作执行前必须先发 `checkout_mode(MANUAL)` 解锁固件 LOCK 保护。

### 2. LOW_BTR 电量保护
电量 < 40% 时固件锁定所有电机输出，提前检查并返回警告。

### 3. spin 参数修复
`cmd_spin(angle)` 现在正确传递旋转角度参数。

### 4. gait_out 确认等待
用 `status_out` 订阅代替硬 sleep(200ms)，等待固件真正就绪。

### 5. 运控板 Mode 1 查询
通过 SSH 读取 `/mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt` 确认开发者模式。

## 参考资料

- 原版 cyberdog_ros2: https://github.com/MiRoboticsLab/cyberdog_ros2
- 腾讯云开发者文章: https://cloud.tencent.com/developer/article/1990634
- 电机 SDK: https://github.com/MiRoboticsLab/cyberdog_motor_sdk
- 运动控制: https://github.com/MiRoboticsLab/cyberdog_locomotion

## 开发环境

- ROS2 **foxy**（不是 Galactic，官方文档有误）
- CyberDog IP: `199.166.55.21`（WiFi）/ `192.168.55.1`（USB）
- SSH: `mi@` / 密码 `123`
