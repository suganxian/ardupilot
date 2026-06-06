# ArduPilot Code Wiki

## 一、项目概述

ArduPilot 是全球最先进、功能最全、最可靠的开源自动驾驶仪软件。自 2010 年开始开发，由专业工程师、计算机科学家和社区贡献者组成的多元化团队维护。该软件能够控制几乎所有类型的无人载具系统，包括固定翼飞机、垂直起降飞行器(VTOL)、多旋翼、直升机、地面车辆、船只、平衡机器人，甚至潜艇。

- **官网**: https://ardupilot.org
- **仓库**: https://github.com/ArduPilot/ardupilot
- **许可证**: GNU General Public License v3 (GPLv3)
- **主要语言**: C++ (飞控核心), Python (工具与测试), Lua (脚本扩展)
- **构建系统**: Waf (基于 Python)

---

## 二、项目整体架构

```
ardupilot/
├── ArduCopter/          # 多旋翼/直升机飞控固件
├── ArduPlane/           # 固定翼/VTOL 飞控固件
├── Rover/               # 地面车辆/船只飞控固件
├── ArduSub/             # 水下航行器(ROV)飞控固件
├── Blimp/               # 飞艇飞控固件
├── AntennaTracker/      # 天线追踪器固件
├── libraries/           # 共享库 (核心子系统)
├── Tools/               # 工具集 (构建工具、自动测试、脚本等)
├── modules/             # 第三方子模块 (git submodules)
├── docs/                # 文档
├── benchmarks/          # 性能基准测试
├── tests/               # 测试配置
├── .github/             # GitHub CI/CD 工作流
├── wscript              # 顶层 Waf 构建脚本
├── waf                  # Waf 构建系统入口
├── BUILD.md             # 构建说明
├── Dockerfile           # Docker 构建环境
└── Makefile             # Make 包装器
```

### 架构层次

```
┌──────────────────────────────────────────────────────────────┐
│                      车辆层 (Vehicles)                        │
│   ArduCopter  ArduPlane  Rover  ArduSub  Blimp  Tracker      │
├──────────────────────────────────────────────────────────────┤
│                    共享库层 (Libraries)                        │
│  ┌─────────┬──────────┬──────────┬──────────┬─────────────┐ │
│  │ 导航/控制 │ 传感器驱动 │ 通信协议  │ 硬件抽象  │ 工具/辅助   │ │
│  │AC_WPNav  │AP_GPS    │GCS_MAVLink│AP_HAL    │AP_Math     │ │
│  │AC_AttCtrl│AP_Baro   │AP_Radio   │AP_HAL_*  │AP_Param    │ │
│  │AP_NavEKF │AP_Compass│AP_Mount   │          │AP_Logger   │ │
│  │AP_Motors │AP_Airspeed│AP_Camera  │          │AP_Mission  │ │
│  └─────────┴──────────┴──────────┴──────────┴─────────────┘ │
├──────────────────────────────────────────────────────────────┤
│                   硬件抽象层 (HAL)                             │
│   AP_HAL_ChibiOS (嵌入式)  AP_HAL_SITL (仿真)  AP_HAL_Linux   │
├──────────────────────────────────────────────────────────────┤
│                 操作系统 / 硬件平台                            │
│   ChibiOS RTOS  |  Linux  |  SITL (Software-in-the-Loop)     │
└──────────────────────────────────────────────────────────────┘
```

---

## 三、车辆模块 (Vehicles) 详细说明

### 3.1 ArduCopter (多旋翼/直升机)

**目录**: `/workspace/ArduCopter/`

**核心类**: `Copter` (继承自 `AP_Vehicle`)

**主要文件**:
| 文件 | 功能 |
|------|------|
| `Copter.h` / `Copter.cpp` | 主类定义与实现，包含 `setup()` 和 `loop()` 主循环 |
| `Parameters.h` / `Parameters.cpp` | 参数系统定义，管理所有可配置参数 |
| `mode.h` / `mode.cpp` | 飞行模式基类 `Mode` 的定义 |
| `config.h` | 编译时配置选项 |
| `defines.h` | 系统宏常量和通用定义 |
| `Attitude.cpp` | 姿态控制逻辑 |
| `motors.cpp` | 电机输出控制 |
| `sensors.cpp` | 传感器初始化和数据读取 |
| `system.cpp` | 系统级功能(初始化、状态管理) |
| `navigation.cpp` | 导航相关逻辑 |
| `commands.cpp` | MAVLink 命令处理 |
| `failsafe.cpp` | 故障保护处理 |
| `fence.cpp` | 地理围栏 |
| `AP_Arming_Copter.cpp` | 解锁检查逻辑 |
| `GCS_Copter.cpp` / `GCS_MAVLink_Copter.cpp` | 地面站通信 |
| `RC_Channel_Copter.cpp` | 遥控器通道映射 |
| `Log.cpp` | 数据日志记录 |

**飞行模式 (在 `mode_*.cpp` 中实现)**:
| 模式 | 文件 | 说明 |
|------|------|------|
| Stabilize | `mode_stabilize.cpp` | 增稳模式，手动控制 |
| Acro | `mode_acro.cpp` | 特技/速率控制模式 |
| AltHold | `mode_althold.cpp` | 定高模式 |
| Loiter | `mode_loiter.cpp` | 悬停(定点)模式 |
| RTL | `mode_rtl.cpp` | 返航模式 |
| Auto | `mode_auto.cpp` | 自动任务模式 |
| Guided | `mode_guided.cpp` | 引导模式(GCS控制) |
| Land | `mode_land.cpp` | 降落模式 |
| Circle | `mode_circle.cpp` | 环绕模式 |
| PosHold | `mode_poshold.cpp` | 位置保持模式 |
| Sport | `mode_sport.cpp` | 运动模式 |
| Drift | `mode_drift.cpp` | 漂移模式 |
| Flip | `mode_flip.cpp` | 翻滚模式 |
| AutoTune | `mode_autotune.cpp` | 自动调参模式 |
| Brake | `mode_brake.cpp` | 制动模式 |
| Throw | `mode_throw.cpp` | 抛投起飞模式 |
| Follow | `mode_follow.cpp` | 跟随模式 |
| ZigZag | `mode_zigzag.cpp` | 之字形模式 |
| SystemID | `mode_systemid.cpp` | 系统辨识模式 |
| Autorotate | `mode_autorotate.cpp` | 自旋降落(直升机) |
| Turtle | `mode_turtle.cpp` | 翻转恢复模式 |
| FlowHold | `mode_flowhold.cpp` | 光流定点模式 |
| AvoidADSB | `mode_avoid_adsb.cpp` | ADS-B 避让模式 |
| SmartRTL | `mode_smart_rtl.cpp` | 智能返航模式 |

**直升机特有模式**: `mode_acro_heli.cpp`, `mode_stabilize_heli.cpp`

### 3.2 ArduPlane (固定翼/VTOL)

**目录**: `/workspace/ArduPlane/`

**核心类**: `Plane` (继承自 `AP_Vehicle`)

**主要文件**:
| 文件 | 功能 |
|------|------|
| `Plane.h` / `Plane.cpp` | 主类定义与实现 |
| `Parameters.h` / `Parameters.cpp` | 参数系统 |
| `mode.h` / `mode.cpp` | 飞行模式基类 |
| `quadplane.h` / `quadplane.cpp` | 垂直起降(VTOL)核心逻辑 |
| `tailsitter.h` / `tailsitter.cpp` | 尾座式 VTOL |
| `tiltrotor.h` / `tiltrotor.cpp` | 倾转旋翼 VTOL |
| `soaring.cpp` | 热气流翱翔 |
| `commands.cpp` | 命令处理 |
| `commands_logic.cpp` | 任务逻辑命令 |
| `navigation.cpp` | 导航 |
| `failsafe.cpp` | 故障保护 |
| `fence.cpp` | 地理围栏 |
| `altitude.cpp` | 高度控制 |
| `servos.cpp` | 舵机输出 |
| `sensors.cpp` | 传感器 |
| `system.cpp` | 系统功能 |
| `radio.cpp` | 遥控器处理 |
| `tuning.cpp` / `tuning.h` | 自动调参 |
| `AP_Arming_Plane.cpp` | 解锁检查 |
| `GCS_Plane.cpp` / `GCS_MAVLink_Plane.cpp` | 地面站通信 |
| `RC_Channel_Plane.cpp` | 遥控器通道 |
| `Log.cpp` | 日志记录 |
| `Attitude.cpp` | 姿态控制 |
| `afs_plane.cpp` | 高级故障保护 |
| `avoidance_adsb.cpp` | ADS-B 避让 |
| `ekf_check.cpp` | EKF 健康检查 |
| `events.cpp` | 事件处理 |
| `is_flying.cpp` | 飞行状态检测 |
| `motor_test.cpp` | 电机测试 |
| `parachute.cpp` | 降落伞 |
| `pullup.cpp` | 自动拉升 |
| `reverse_thrust.cpp` | 反推 |
| `takeoff.cpp` | 起飞逻辑 |
| `VTOL_Assist.cpp` | VTOL 辅助 |

**飞行模式**:
| 模式 | 说明 |
|------|------|
| Manual | 纯手动 |
| FBWA | 增稳A (Fly By Wire A) |
| FBWB | 增稳B (Fly By Wire B) |
| Cruise | 巡航模式 |
| Auto | 自动任务 |
| RTL | 返航 |
| Loiter | 盘旋 |
| Guided | 引导模式 |
| Circle | 环绕 |
| Stabilize | 增稳 |
| Acro | 特技 |
| Training | 教练模式 |
| Takeoff | 起飞 |
| Thermal | 热气流 |
| AutoTune | 自动调参 |
| AvoidADSB | ADS-B 避让 |

**QuadPlane (VTOL) 特有模式**:
| 模式 | 说明 |
|------|------|
| QStabilize | 多旋翼增稳 |
| QHover | 多旋翼悬停 |
| QLoiter | 多旋翼定点 |
| QLand | 多旋翼降落 |
| QRTL | 多旋翼返航 |
| QAcro | 多旋翼特技 |
| QAutoTune | 多旋翼自动调参 |
| LoiterAltQLand | 盘旋后垂直降落 |

### 3.3 Rover (地面车辆/船只)

**目录**: `/workspace/Rover/`

**核心类**: `Rover` (继承自 `AP_Vehicle`)

**主要文件**:
| 文件 | 功能 |
|------|------|
| `Rover.h` / `Rover.cpp` | 主类 |
| `Parameters.h` / `Parameters.cpp` | 参数 |
| `mode.h` / `mode.cpp` | 模式基类 |
| `Steering.cpp` | 转向控制 |
| `sailboat.h` / `sailboat.cpp` | 帆船控制 |
| `balance_bot.cpp` | 平衡机器人 |
| `cruise_learn.cpp` | 巡航学习 |
| `crash_check.cpp` | 碰撞检测 |
| `commands.cpp` | 命令处理 |
| `failsafe.cpp` | 故障保护 |
| `fence.cpp` | 围栏 |
| `sensors.cpp` | 传感器 |
| `system.cpp` | 系统 |
| `radio.cpp` | 遥控器 |
| `AP_Arming_Rover.cpp` | 解锁 |
| `GCS_Rover.cpp` / `GCS_MAVLink_Rover.cpp` | 地面站 |
| `RC_Channel_Rover.cpp` | 遥控器通道 |
| `Log.cpp` | 日志 |
| `afs_rover.cpp` | 高级故障保护 |
| `ekf_check.cpp` | EKF 检查 |
| `motor_test.cpp` | 电机测试 |
| `precision_landing.cpp` | 精准着陆 |

**驾驶模式**:
| 模式 | 说明 |
|------|------|
| Manual | 手动 |
| Acro | 特技 |
| Steering | 转向 |
| Hold | 保持 |
| Loiter | 定点 |
| Auto | 自动任务 |
| RTL | 返航 |
| SmartRTL | 智能返航 |
| Guided | 引导 |
| Circle | 环绕 |
| Follow | 跟随 |
| Simple | 简单模式 |
| Dock | 对接模式 |

### 3.4 ArduSub (水下航行器)

**目录**: `/workspace/ArduSub/`

**核心类**: `Sub` (继承自 `AP_Vehicle`)

**主要文件**: 与 Rover 结构类似，包含 `mode.h`/`mode.cpp`、`Parameters.h` 等。

**飞行模式**:
| 模式 | 说明 |
|------|------|
| Manual | 手动 |
| Stabilize | 增稳 |
| Acro | 特技 |
| AltHold | 定深 |
| PosHold | 定点 |
| Auto | 自动任务 |
| Guided | 引导 |
| Circle | 环绕 |
| Surface | 上浮 |
| Surftrak | 水面追踪 |
| MotorDetect | 电机检测 |

### 3.5 Blimp (飞艇)

**目录**: `/workspace/Blimp/`

**核心类**: `Blimp` (继承自 `AP_Vehicle`)

**飞行模式**: Manual, Loiter, Land, RTL, Velocity

### 3.6 AntennaTracker (天线追踪器)

**目录**: `/workspace/AntennaTracker/`

**核心类**: `Tracker` (继承自 `AP_Vehicle`)

**飞行模式**: Manual, Auto, Scan, Guided, ServoTest

---

## 四、核心共享库 (Libraries) 详细说明

### 4.1 硬件抽象层 (HAL)

| 库 | 说明 |
|----|------|
| **AP_HAL** | 硬件抽象层接口定义。定义了所有 HAL 实现的统一接口，包括 UART、I2C、SPI、GPIO、ADC、RCInput、RCOutput、Scheduler、Storage 等 |
| **AP_HAL_ChibiOS** | 基于 ChibiOS RTOS 的 HAL 实现。用于 Pixhawk/Cube 等嵌入式飞控硬件 |
| **AP_HAL_SITL** | 软件在环仿真 (SITL) HAL 实现。在 PC 上模拟完整飞控运行 |
| **AP_HAL_Linux** | 基于 Linux 的 HAL 实现。用于 NavIO、BeagleBone 等 Linux 飞控板 |

### 4.2 导航与姿态估计

| 库 | 说明 |
|----|------|
| **AP_AHRS** | 姿态航向参考系统 (Attitude Heading Reference System)。融合 IMU、GPS、罗盘等数据，提供飞行器的姿态、位置、速度估计。核心类 `AP_AHRS` |
| **AP_NavEKF** | 导航扩展卡尔曼滤波器基类 |
| **AP_NavEKF2** | EKF2 导航滤波器实现。24 状态扩展卡尔曼滤波器，融合 IMU、GPS、气压计、罗盘、光流、视觉里程计等 |
| **AP_NavEKF3** | EKF3 导航滤波器实现。更先进的 EKF 实现，支持多 IMU、更好的磁力计融合 |
| **AP_InertialNav** | 惯性导航系统。提供基于 EKF 的惯性导航数据 |
| **AP_InertialSensor** | 惯性传感器(IMU)管理。处理陀螺仪和加速度计的数据采集、校准、滤波 |
| **AP_Compass** | 罗盘/磁力计管理。处理磁力计数据采集、校准和融合 |
| **AP_Baro** | 气压计管理。处理气压传感器数据，提供高度估算 |
| **AP_GPS** | GPS 管理。支持 u-blox、NMEA、SBP、SIRF 等多种 GPS 协议 |
| **AP_VisualOdom** | 视觉里程计接口。支持 Intel RealSense T265 等视觉传感器 |
| **AP_ExternalAHRS** | 外部 AHRS 系统支持。如 VectorNav 等外部姿态传感器 |
| **AP_OpticalFlow** | 光流传感器支持。用于低空定点悬停 |
| **AP_Beacon** | 室内定位信标支持。如 Marvelmind、Pozyx、Nooploop |

### 4.3 控制算法

| 库 | 说明 |
|----|------|
| **AC_AttitudeControl** | 多旋翼/直升机姿态控制库。提供角速率控制、姿态角控制、推力控制等 PID 控制器 |
| **AC_WPNav** | 多旋翼航点导航库。实现直线、圆形、悬停等导航模式 |
| **AC_PID** | PID 控制器库。实现标准 PID 控制算法 |
| **AC_AutoTune** | 多旋翼自动调参 |
| **AC_InputManager** | 输入管理器 |
| **AC_Fence** | 多旋翼地理围栏 |
| **AC_Sprayer** | 喷洒器控制 |
| **AC_Autorotation** | 自旋降落控制(直升机) |
| **AR_AttitudeControl** | 地面车辆姿态控制 |
| **AR_PosControl** | 地面车辆位置控制 |
| **AR_WPNav** | 地面车辆航点导航 |
| **AP_L1_Control** | L1 导航控制算法。用于固定翼路径跟踪 |
| **AP_TECS** | 总能量控制系统 (Total Energy Control System)。用于固定翼高度和速度协调控制 |
| **AP_SpdHgtControl** | 速度高度控制 |
| **AP_Steering** | 转向控制(地面车辆) |
| **AP_PID** | PID 控制器 |

### 4.4 电机与舵机控制

| 库 | 说明 |
|----|------|
| **AP_Motors** | 电机控制库。支持多种多旋翼构型(Quad, Hexa, Octa, 共轴等)和直升机的电机混合 |
| **AP_MotorsHeli** | 直升机电机控制 |
| **AR_Motors** | 地面车辆电机控制 |
| **SRV_Channel** | 舵机通道管理。管理和输出所有舵机/电机 PWM 信号 |
| **AP_RCMapper** | 遥控器通道映射 |
| **AP_ServoRelayEvents** | 舵机和继电器事件处理 |
| **AP_BLHeli** | BLHeli 电调协议支持 |
| **AP_ESC_Telem** | 电调遥测数据 |
| **AP_RPM** | 转速传感器 |
| **AP_RobotisServo** | Robotis 舵机协议 |
| **AP_Volz_Protocol** | Volz 舵机协议 |
| **AP_SBusOut** | SBus 输出 |

### 4.5 通信协议

| 库 | 说明 |
|----|------|
| **GCS_MAVLink** | 地面控制站通信。基于 MAVLink 协议，处理与地面站(如 Mission Planner、QGroundControl)的双向通信 |
| **AP_Radio** | 遥控器接收机协议。支持 CRSF/Crossfire, SBus, DSM, PPM, 等多种协议 |
| **AP_RCProtocol** | 遥控器协议解析 |
| **AP_RCTelemetry** | 遥控器遥测回传 |
| **AP_Frsky_Telem** | FrSky 遥测协议 |
| **AP_Hott_Telem** | Graupner HOTT 遥测 |
| **AP_IBus_Telem** | FlySky IBus 遥测 |
| **AP_Devo_Telem** | Walkera Devo 遥测 |
| **AP_NMEA_Output** | NMEA 0183 输出 |
| **AP_DDS** | DDS (Data Distribution Service) 支持 |
| **AP_Networking** | 网络通信 |
| **AP_KDECAN** | KDE CAN 协议 |
| **AP_PiccoloCAN** | Piccolo CAN 协议 |

### 4.6 任务与导航辅助

| 库 | 说明 |
|----|------|
| **AP_Mission** | 任务系统。管理航点任务的存储、加载、执行 |
| **AP_Rally** | 集结点管理。定义并管理安全集结点 |
| **AP_Terrain** | 地形数据库。支持地形跟随和地形避障 |
| **AP_Follow** | 跟随模式。实现跟随其他载具/GPS 位置 |
| **AP_SmartRTL** | 智能返航。记录飞行路径并沿原路返回 |
| **AP_Camera** | 相机控制。支持多种相机触发方式 |
| **AP_Mount** | 云台控制。支持 Alexmos、SToRM32、Servo、Siyi、Xacti 等多种云台 |
| **AP_ADSB** | ADS-B 接收机。接收载人飞机位置信息用于避让 |
| **AP_Avoidance** | 避障系统。综合多种传感器实现避障 |
| **AP_Proximity** | 近距传感器(避障雷达)。支持多种避障传感器 |
| **AP_Fence** | 地理围栏。定义飞行边界 |
| **AP_Landing** | 精密着陆。支持视觉和 IR 信标着陆 |
| **AP_LandingGear** | 起落架控制 |
| **AP_Winch** | 绞盘控制 |
| **AP_Gripper** | 抓取器控制 |

### 4.7 电源与能源

| 库 | 说明 |
|----|------|
| **AP_BattMonitor** | 电池监控。支持电压、电流、电量监测，多种电池类型 |
| **AP_Generator** | 发电机管理。支持 IE 系列、RichenPower、Loweheiser 等 |
| **AP_ICEngine** | 内燃机控制 |
| **AP_EFI** | 电子燃油喷射 |

### 4.8 通知与显示

| 库 | 说明 |
|----|------|
| **AP_Notify** | 通知系统。管理 LED、蜂鸣器、RGB LED 等多种通知设备 |
| **AP_OSD** | 屏幕显示 (On-Screen Display)。在 FPV 视频上叠加飞行数据 |
| **AP_VideoTX** | 视频发射机控制。支持 SmartAudio、Tramp 等协议 |

### 4.9 系统基础

| 库 | 说明 |
|----|------|
| **AP_Vehicle** | 车辆基类。定义所有飞行器类型的公共接口(`setup()`, `loop()`, 模式切换等) |
| **AP_Arming** | 解锁检查系统。飞行前安全检查 |
| **AP_Param** | 参数系统。所有可配置参数的存储、加载、验证 |
| **AP_Logger** | 数据日志记录。记录飞行数据到 SD 卡或闪存 |
| **AP_Math** | 数学库。向量、矩阵、四元数、坐标变换等 |
| **Filter** | 数字滤波器。低通、陷波、卡尔曼等 |
| **AP_Common** | 通用工具。位置、速度等基础数据结构 |
| **AP_Stats** | 统计数据 |
| **AP_RTC** | 实时时钟 |
| **AP_FlashStorage** | 闪存存储 |
| **StorageManager** | 存储管理器 |
| **AP_BoardConfig** | 板卡配置 |
| **AP_Button** | 按钮输入处理 |
| **AP_Module** | 模块系统 |
| **AP_ROMFS** | ROM 文件系统 |
| **AP_SerialManager** | 串口管理器 |
| **AP_SerialLED** | 串行 LED 控制 |
| **AP_Relay** | 继电器控制 |
| **AP_Scripting** | 板载 Lua 脚本引擎。允许用户编写自定义脚本扩展飞控功能 |
| **AP_AdvancedFailsafe** | 高级故障保护 |
| **AP_InternalError** | 内部错误处理 |
| **AP_Declination** | 地磁偏角计算 |
| **AP_TempCalibration** | 温度校准 |
| **AP_CSVReader** | CSV 文件读取 |
| **AP_OLC** | 开放位置编码 (Open Location Code) |
| **AP_GyroFFT** | 陀螺仪 FFT 分析。用于振动分析和滤波器配置 |
| **AP_Quicktune** | 快速调参 |
| **AP_Tuning** | 调参辅助 |
| **AP_Soaring** | 热气流翱翔(固定翼) |
| **AP_WindVane** | 风向标传感器 |
| **AP_WheelEncoder** | 轮式编码器 |
| **AP_LeakDetector** | 漏水检测 |
| **AP_TemperatureSensor** | 温度传感器 |
| **AP_RSSI** | 信号强度指示 |
| **AP_RangeFinder** | 测距仪(超声波/激光)。用于低空高度测量和避障 |
| **AP_Airspeed** | 空速传感器 |
| **AP_IRLock** | IR 锁定(用于精准着陆) |
| **AP_Parachute** | 降落伞控制 |
| **SITL** | 软件在环仿真模型。模拟各种传感器和物理模型 |

### 4.10 第三方协议/驱动

| 库 | 说明 |
|----|------|
| **AP_DroneCAN** | DroneCAN (原 UAVCAN) 协议支持 |
| **AP_IOMCU** | IO 微控制器通信(FMU 与 IO MCU 之间的通信) |
| **AP_OpenDroneID** | 远程 ID 支持 |

---

## 五、关键类与函数说明

### 5.1 AP_Vehicle (车辆基类)

**文件**: `libraries/AP_Vehicle/AP_Vehicle.h`

所有飞行器类型(Copter, Plane, Rover, Sub, Blimp, Tracker)的基类。

**关键方法**:
- `setup()` - 系统初始化(sketch setup)
- `loop()` - 主循环，每个飞行器类型实现自己的调度逻辑
- `set_mode()` - 切换飞行模式
- `init_ardupilot()` - ArduPilot 通用初始化
- `update_dynamic_notch()` - 更新动态陷波滤波器

**关键成员**:
- `AP_AHRS ahrs` - 姿态航向参考系统
- `AP_Baro barometer` - 气压计
- `AP_Compass compass` - 罗盘
- `AP_GPS gps` - GPS
- `AP_InertialSensor ins` - IMU
- `AP_BattMonitor battery` - 电池监控
- `AP_Arming arming` - 解锁检查
- `AP_Mission mission` - 任务系统
- `GCS gcs` - 地面站通信
- `AP_Logger logger` - 日志记录
- `RC_Channels rc_channels` - 遥控器通道

### 5.2 AP_AHRS (姿态航向参考系统)

**文件**: `libraries/AP_AHRS/AP_AHRS.h`

**关键方法**:
- `update()` - 更新 AHRS 状态
- `get_roll()` / `get_pitch()` / `get_yaw()` - 获取欧拉角
- `get_position()` - 获取位置
- `get_velocity_NED()` - 获取 NED 坐标系速度
- `get_relative_position_NED_origin()` - 获取相对原点位置
- `get_wind_estimate()` - 获取风速估计
- `get_EAS2TAS()` - 等效空速转真空速

### 5.3 AP_InertialSensor (IMU)

**文件**: `libraries/AP_InertialSensor/AP_InertialSensor.h`

**关键方法**:
- `init()` - 初始化传感器
- `update()` - 读取传感器数据
- `get_gyro()` - 获取陀螺仪数据
- `get_accel()` - 获取加速度计数据
- `get_delta_angle()` / `get_delta_velocity()` - 获取增量角度/速度
- `calibrate_gyros()` / `calibrate_accel()` - 校准

### 5.4 AC_AttitudeControl (姿态控制)

**文件**: `libraries/AC_AttitudeControl/AC_AttitudeControl.h`

**关键方法**:
- `rate_controller_run()` - 角速率 PID 控制器
- `attitude_control_run_quat()` - 基于四元数的姿态控制
- `input_euler_angle_roll_pitch_euler_rate_yaw()` - 输入欧拉角目标
- `set_throttle_out()` - 设置油门输出
- `thrust_heading_rotation_angles()` - 推力方向旋转角计算

### 5.5 AP_Motors (电机控制)

**文件**: `libraries/AP_Motors/AP_Motors.h`

**关键子类**: `AP_MotorsMatrix`, `AP_MotorsHeli_Single`, `AP_MotorsHeli_Dual`, `AP_MotorsHeli_Quad` 等

**关键方法**:
- `output()` - 输出电机控制信号
- `set_frame_class()` - 设置机架类型
- `set_throttle()` - 设置油门
- `set_roll()` / `set_pitch()` / `set_yaw()` - 设置各轴控制量

### 5.6 AP_GPS (GPS)

**文件**: `libraries/AP_GPS/AP_GPS.h`

**关键方法**:
- `init()` - 初始化 GPS
- `update()` - 更新 GPS 数据
- `status()` - 获取 GPS 状态
- `location()` - 获取位置
- `speed()` / `ground_speed()` - 获取速度
- `num_sats()` - 获取卫星数
- `hdop()` / `vdop()` - 获取精度因子

### 5.7 AP_NavEKF2 / AP_NavEKF3 (导航 EKF)

**文件**: `libraries/AP_NavEKF2/AP_NavEKF2.h`, `libraries/AP_NavEKF3/AP_NavEKF3.h`

**关键方法**:
- `getLLH()` - 获取经纬度和高度
- `getVelNED()` - 获取 NED 速度
- `getQuaternion()` - 获取姿态四元数
- `getPosD()` - 获取高度
- `healthy()` - EKF 健康状态
- `getFilterFaults()` - 获取滤波器故障标志

### 5.8 GCS_MAVLink (地面站通信)

**文件**: `libraries/GCS_MAVLink/GCS.h`, `libraries/GCS_MAVLink/GCS_MAVLink.h`

**关键方法**:
- `send_heartbeat()` - 发送心跳包
- `send_message()` - 发送 MAVLink 消息
- `update_send()` - 更新发送队列
- `update_receive()` - 处理接收到的消息
- `handleMessage()` - 消息处理分发

### 5.9 AP_Mission (任务系统)

**文件**: `libraries/AP_Mission/AP_Mission.h`

**关键方法**:
- `init()` - 初始化任务
- `start()` - 开始执行任务
- `update()` - 更新任务状态
- `set_current_cmd()` - 设置当前命令
- `get_current_nav_cmd()` - 获取当前导航命令
- `jump_to_abort_land()` - 跳转到紧急降落

### 5.10 AP_Logger (数据日志)

**文件**: `libraries/AP_Logger/AP_Logger.h`

**关键方法**:
- `init()` - 初始化日志系统
- `Write()` - 写入日志条目
- `WriteBlock()` - 写入块数据
- `set_mission()` - 设置任务日志
- `EnableWrites()` / `StopLogging()` - 启停日志

### 5.11 AP_Param (参数系统)

**文件**: `libraries/AP_Param/AP_Param.h`

**关键方法**:
- `setup()` - 初始化参数系统
- `set()` / `get()` - 设置/获取参数值
- `save()` / `load()` - 保存/加载参数
- `set_defaults_from()` - 从默认表设置参数

---

## 六、飞行模式架构

ArduPilot 采用统一的飞行模式架构。所有车辆类型共享相同的模式管理机制：

```
AP_Vehicle
  └── Mode (基类)
        ├── mode_auto       # 自动任务
        ├── mode_guided     # 引导模式
        ├── mode_rtl        # 返航
        ├── mode_loiter     # 定点/盘旋
        ├── mode_manual     # 手动
        ├── mode_stabilize  # 增稳
        ├── mode_acro       # 特技
        ├── mode_circle     # 环绕
        └── ... (车辆特定模式)
```

**模式基类关键方法**:
- `init()` - 模式初始化
- `run()` - 模式主循环(高频调用，通常 400Hz)
- `exit()` - 模式退出
- `requires_GPS()` - 是否需要 GPS
- `has_manual_throttle()` - 是否手动油门

---

## 七、构建系统

### 7.1 Waf 构建系统

ArduPilot 使用 **Waf** 构建系统。

**主要构建命令**:
```bash
# 配置 (指定目标板)
./waf configure --board CubeBlack    # Pixhawk Cube Black
./waf configure --board sitl         # SITL 仿真

# 构建各车辆
./waf copter      # 多旋翼
./waf plane       # 固定翼
./waf rover       # 地面车辆
./waf sub         # 水下航行器
./waf heli        # 直升机
./waf blimp       # 飞艇
./waf antennatracker  # 天线追踪器

# 构建并上传
./waf copter --upload

# 运行测试
./waf check       # 构建并运行相关测试
./waf check-all   # 构建并运行所有测试

# 列出可用板卡
./waf list_boards
```

### 7.2 SITL 仿真

SITL (Software-in-the-Loop) 允许在 PC 上运行完整的 ArduPilot 飞控代码：

```bash
./waf configure --board sitl
./waf copter
```

通过 `sim_vehicle.py` 脚本启动仿真：
```bash
sim_vehicle.py -v ArduCopter -f quad
```

### 7.3 Docker 构建

```bash
docker build --rm -t ardupilot-dev .
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf configure --board=sitl
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf copter
```

---

## 八、依赖关系图

### 8.1 车辆层依赖

```
ArduCopter ────┐
ArduPlane ─────┤
Rover ─────────┤
ArduSub ───────┼── 依赖 ──→ libraries/ (所有共享库)
Blimp ─────────┤
AntennaTracker ┘
```

### 8.2 核心库依赖链

```
AP_Vehicle
  ├── AP_AHRS
  │     ├── AP_NavEKF2 / AP_NavEKF3
  │     ├── AP_InertialSensor
  │     ├── AP_Baro
  │     ├── AP_Compass
  │     ├── AP_GPS
  │     ├── AP_Airspeed
  │     ├── AP_Beacon
  │     ├── AP_VisualOdom
  │     ├── AP_ExternalAHRS
  │     └── AP_OpticalFlow
  ├── AC_AttitudeControl
  │     ├── AC_PID
  │     └── AP_Motors
  ├── AC_WPNav / AR_WPNav
  ├── AP_Mission
  ├── GCS_MAVLink
  ├── AP_Logger
  ├── AP_Param
  ├── AP_BattMonitor
  ├── AP_Arming
  ├── AP_Rally
  ├── AP_Fence
  ├── AP_Camera
  ├── AP_Mount
  ├── AP_Notify
  ├── AP_Radio / AP_RCProtocol
  ├── AP_Terrain
  └── AP_HAL (硬件抽象层)
```

### 8.3 HAL 层依赖

```
AP_HAL (接口)
  ├── AP_HAL_ChibiOS  → ChibiOS RTOS → STM32 等 MCU
  ├── AP_HAL_SITL     → Linux/Windows/macOS (仿真)
  └── AP_HAL_Linux    → Linux 系统 (NavIO, BeagleBone 等)
```

### 8.4 外部依赖 (modules/)

通过 git submodules 管理：
- `mavlink` - MAVLink 通信协议定义
- `DroneCAN` - DroneCAN (原 UAVCAN) 协议
- `ChibiOS` - 实时操作系统
- `gtest` - Google Test 测试框架
- `waf` - 构建系统
- 其他外设驱动库

---

## 九、CI/CD 与测试

### 9.1 CI 工作流

项目在 `.github/workflows/` 下维护了完善的 CI 流水线：

| 工作流 | 说明 |
|--------|------|
| `test_sitl_copter.yml` | SITL 多旋翼测试 |
| `test_sitl_plane.yml` | SITL 固定翼测试 |
| `test_sitl_rover.yml` | SITL 地面车辆测试 |
| `test_sitl_sub.yml` | SITL 水下航行器测试 |
| `test_sitl_tracker.yml` | SITL 天线追踪器测试 |
| `test_sitl_blimp.yml` | SITL 飞艇测试 |
| `test_sitl_periph.yml` | SITL 外设测试 |
| `test_chibios.yml` | ChibiOS 编译测试 |
| `test_linux_sbc.yml` | Linux SBC 测试 |
| `test_coverage.yml` | 代码覆盖率测试 |
| `test_unit_tests.yml` | 单元测试 |
| `test_size.yml` | 固件大小检查 |
| `pre-commit.yml` | 预提交检查 |
| `macos_build.yml` | macOS 构建 |
| `cygwin_build.yml` | Cygwin 构建 |
| `esp32_build.yml` | ESP32 构建 |
| `qurt_build.yml` | Qualcomm QURT 构建 |

### 9.2 自动测试

自动测试框架位于 `Tools/autotest/`:
- `autotest.py` - 主测试入口
- `arducopter.py` - 多旋翼测试用例
- `arduplane.py` - 固定翼测试用例
- `rover.py` - 地面车辆测试用例
- `ardusub.py` - 水下航行器测试用例
- `sim_vehicle.py` - SITL 仿真启动脚本

### 9.3 单元测试

单元测试位于 `libraries/*/tests/` 目录下，使用 Google Test 框架。

---

## 十、项目运行方式

### 10.1 环境搭建

```bash
# 克隆仓库
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# 安装依赖 (Ubuntu/Debian)
Tools/environment_install/install-prereqs-ubuntu.sh -y

# 或使用 Docker
docker build --rm -t ardupilot-dev .
```

### 10.2 固件编译

```bash
# 以 Pixhawk Cube Black 为目标编译多旋翼固件
./waf configure --board CubeBlack
./waf copter

# 编译固定翼固件
./waf plane

# 编译地面车辆固件
./waf rover
```

### 10.3 SITL 仿真运行

```bash
# 配置 SITL
./waf configure --board sitl

# 编译并运行仿真
./waf copter
sim_vehicle.py -v ArduCopter -f quad --console --map
```

### 10.4 固件上传

```bash
# 通过 USB 上传到 Pixhawk
./waf copter --upload

# 通过网络上传到 Linux 板卡
./waf configure --board navio2 --rsync-dest root@192.168.1.2:/
./waf --target bin/arducopter --upload
```

### 10.5 运行测试

```bash
# 构建并运行相关测试
./waf check

# 运行所有测试
./waf check-all

# 运行特定单元测试
./waf --targets tests/test_math
```

---

## 十一、参数系统

ArduPilot 拥有超过 1000 个可配置参数，通过 `AP_Param` 库管理。参数存储在 FRAM/EEPROM/闪存中，可通过以下方式配置：

- 地面站软件 (Mission Planner, QGroundControl)
- MAVLink 协议参数消息
- SD 卡上的参数文件

参数按前缀分组：
- `AHRS_*` - 姿态参考系统
- `ATC_*` - 姿态控制
- `BATT_*` - 电池
- `COMPASS_*` - 罗盘
- `EK2_*` / `EK3_*` - EKF 滤波器
- `GPS_*` - GPS
- `MOT_*` - 电机
- `PSC_*` - 位置控制
- `RC*_*` - 遥控器
- `SERVO*_*` - 舵机
- `WPNAV_*` - 航点导航

---

## 十二、数据流概览

```
传感器输入:
  IMU(Gyro+Accel) → AP_InertialSensor
  GPS             → AP_GPS
  气压计           → AP_Baro
  罗盘             → AP_Compass
  空速计           → AP_Airspeed
  遥控器           → AP_RCProtocol → RC_Channel

         ↓

数据融合:
  → AP_AHRS → AP_NavEKF2/3 → 姿态/位置/速度估计

         ↓

控制:
  姿态控制: AC_AttitudeControl → AP_Motors → SRV_Channel → PWM 输出
  导航控制: AC_WPNav / AP_L1_Control / AP_TECS
  任务管理: AP_Mission

         ↓

通信输出:
  遥测数据 → GCS_MAVLink → 地面站
  日志数据 → AP_Logger → SD 卡/闪存
  通知     → AP_Notify → LED/蜂鸣器
  OSD      → AP_OSD → FPV 视频叠加

         ↓

执行器输出:
  电机/舵机 PWM 信号 → 飞行器物理控制
```

---

## 十三、扩展与定制

### 13.1 脚本扩展 (Lua)

通过 `AP_Scripting` 库，用户可以使用 Lua 脚本扩展飞控功能：
- 自定义飞行模式
- 自定义传感器数据处理
- 自定义 LED 和通知行为
- 自定义遥测格式化

### 13.2 外部控制

通过 `AP_ExternalControl` 接口，外部计算机(如树莓派、Jetson)可控制飞控：
- 发送位置/速度/姿态指令
- 读取传感器数据
- 管理任务

### 13.3 自定义固件

通过 `UserCode.cpp` / `UserParameters.cpp` 文件，用户可添加自定义代码而不修改主线代码。

---

## 十四、总结

ArduPilot 是一个高度模块化、功能丰富的开源自动驾驶仪系统。其核心设计理念是：

1. **分层架构**: HAL 层隔离硬件差异，库层提供通用功能，车辆层实现特定类型控制
2. **模式驱动**: 所有操作通过飞行模式(flight mode)进行切换和调度
3. **参数化配置**: 几乎所有行为都可通过参数配置
4. **多传感器融合**: EKF 滤波器融合多种传感器数据以获得鲁棒的状态估计
5. **跨平台**: 支持从嵌入式 MCU 到 Linux PC 的多种平台
6. **可扩展**: 通过 Lua 脚本、外部控制接口和自定义代码提供扩展能力