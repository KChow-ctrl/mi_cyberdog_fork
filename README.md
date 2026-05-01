# mi_cyberdog_fork

小米铁蛋 CyberDog — 基于 MiRoboticsLab 开源代码 + 本地改进

## 仓库结构

```
mi_cyberdog_fork/
├── README.md
├── docs/
│   ├── 腾讯云文章笔记.md          # 腾讯云开发者社区文章核心内容
│   └── GitHub开源项目说明.md     # MiRoboticsLab 各仓库说明
└── src/                          # MiRoboticsLab/cyberdog_ros2 源码
    ├── cyberdog_bringup/          # 启动入口（lc_bringup_launch.py）
    ├── cyberdog_decision/        # 决策系统 + motion_manager
    ├── cyberdog_interaction/     # 音频/相机/LED/触摸
    ├── cyberdog_interfaces/      # ROS2 消息/服务/action 定义
    ├── cyberdog_common/          # 通用工具
    ├── tools/                    # 工具脚本
    ├── README.md
    └── ...                       # 其他包
```

## 源码来源

```bash
# 原始 MiRoboticsLab 仓库
git clone https://github.com/MiRoboticsLab/cyberdog_ros2.git
```

## 关键 Action 接口（实测有效）

| Action | Topic | 说明 |
|--------|-------|------|
| `ExtMonOrder` | `/mi1034819/exe_monorder` | 动作执行（stand_up/id=9, prostrate/id=10...） |
| `ChangeMode` | `/mi1034819/checkout_mode` | 模式切换（MANUAL=3 解锁固件） |
| `ChangeGait` | `/mi1034819/checkout_gait` | 步态切换（WALK=6 等） |
| `AudioPlay` | `/mi1034819/audio_play` | 音效播放 |

### 动作 ID（MonOrder.msg）
```
MONO_ORDER_STAND_UP=9, MONO_ORDER_PROSTRATE=10,
MONO_ORDER_STEP_BACK=12, MONO_ORDER_TURN_AROUND=13,
MONO_ORDER_HI_FIVE=14, MONO_ORDER_DANCE=15,
MONO_ORDER_WELCOME=16, MONO_ORDER_TURN_OVER=17,
MONO_ORDER_SIT=18
```

### 步态 ID（Gait.msg）
```
GAIT_TRANS=0, GAIT_PASSIVE=1, GAIT_KNEEL=2,
GAIT_STAND_R=3, GAIT_STAND_B=4, GAIT_AMBLE=5,
GAIT_WALK=6, GAIT_SLOW_TROT=7, GAIT_TROT=8,
GAIT_FLYTROT=9, GAIT_BOUND=10, GAIT_PRONK=11
```

### Mode 枚举
```
DEFAULT=0, LOCK=1, MANUAL=3, SEMI=13, EXPLOR=14, TRACK=15
```

## 关键改进方向

1. **checkout_mode 前置** — 所有动作前必须切 MANUAL 解锁固件 LOCK 保护
2. **LOW_BTR 电量保护** — `safety.status == LOW_BTR` 时锁定电机
3. **velocity 持续发布** — 单次 publish 不足以驱动走路，需 20Hz 持续发送
4. **gait_out 确认等待** — 替代硬 sleep(200ms)
5. **audio_play SSH 回退** — action server 不可用时切换到 `aplay` WAV

## 腾讯云文档核心内容

- USB 多播配置（usb0 + 224.0.0.0 路由）
- 运控板 `/mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt` → Mode 0/1
- 电机 SDK LCM 通信：`Motor control mode has not been activated`
- 开机自启动：`/mnt/UDISK/manager_config/fork_para_conf_lists.json`
- 关闭顺序：先用户程序 → 主程序超时趴下 → 再关主程序

## 开发环境

- ROS2 **foxy**（官方文档写 Galactic 有误）
- CyberDog IP: `199.166.55.21`（WiFi）/ `192.168.55.1`（USB）
- SSH: `mi@` / 密码 `123`
- 运控板 MCU: `root@192.168.55.233`

## 相关链接

- MiRoboticsLab/cyberdog_ros2: https://github.com/MiRoboticsLab/cyberdog_ros2
- 腾讯云文章: https://cloud.tencent.com/developer/article/1990634
- 电机 SDK: https://github.com/MiRoboticsLab/cyberdog_motor_sdk
- 运动控制: https://github.com/MiRoboticsLab/cyberdog_locomotion
