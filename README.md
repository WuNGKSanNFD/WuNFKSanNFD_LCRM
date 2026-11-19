# Sentry-MDK-27 电控开发日志

> RoboMaster 哨兵机器人 · STM32F407IGHx + FreeRTOS + HAL · Keil MDK 工程

| 项 | 内容 |
| --- | --- |
| 工程 | `Sentry-MDK-27` |
| 主控 | STM32F407IGHx，HSE 12 MHz → PLL → **SYSCLK 168 MHz**（APB1 42 MHz / APB2 84 MHz） |
| 软件栈 | STM32Cube FW_F4 V1.28.3 + FreeRTOS（CMSIS-RTOS2，heap_4，1000 Hz tick，堆 20 000 B） |
| 工具链 | ARMCC V5.06 update 5（AC5），CMSIS 6.2.0 / DFP 3.1.1 |
| 当前版本 | **V2.1.1**（2026-11-19，云台 pitch 双轴完成上车验证；**yaw 仍未验证**） |
| 最近构建 | Code 57 112 B / RO-data 4 196 B / RW-data 636 B / ZI-data 93 788 B，0 error 0 warning（2026-11-19 21:12，增量编译 `gimbal.c` 后链接） |

**文档说明**

本文依据两处材料整理：

1. **仓库源码**——所有涉及实现的结论均对应到具体文件行号，可直接核对；
2. **Git 提交历史**——版本演进脉络（见附录 A）。当前仓库是 2026-11-16 重建的，只有 `1343ab3`（V2.1.0）一次提交，更早的提交不在本仓库内，仅以附录 A 的文字记录保留。

飞书知识库文档 `IxyOwRZHmixeWykl0LQc4KMDngZ` 为登录可见，当前环境无法读取，故本文暂未引用其中内容；待补充后应重点核对第 8 章的里程碑与需求条目。凡是**仅凭提交信息**得出、未经代码交叉验证的阶段，文中已标注「据提交信息」。

V2.1.0 已提交为 `1343ab3`（2026-11-16）；**V2.1.1 的改动仍在工作区**（相对 `1343ab3` 的差异），第 3.13 节即依据该差异整理。

---

## 目录

- [1. 项目速览](#1-项目速览)
- [2. 版本时间线](#2-版本时间线)
- [3. 分阶段开发日志](#3-分阶段开发日志)
- [4. 当前架构](#4-当前架构)
- [5. 关键设计与算法](#5-关键设计与算法)
- [6. 缺陷与待办](#6-缺陷与待办)
- [7. 参数速查](#7-参数速查)
- [8. 里程碑与下一步](#8-里程碑与下一步)
- [附录 A · 提交记录](#附录-a--提交记录)
- [附录 B · 源码清单](#附录-b--源码清单)

---

## 1. 项目速览

### 1.1 一句话概括

一套「**指令集中于 `cmd`、机构各自闭环**」的哨兵电控框架：遥控器 / 视觉上位机分别给出整车意图，`cmd` 把它翻译成底盘、云台、发射机构三份控制量并广播；三个机构各自以固定周期从 `cmd` 拉取指令，完成解算与闭环，并把状态写回，由 `cmd` 汇总后回传上位机。

### 1.2 代码规模

| 层 | 文件数 | 行数 | 说明 |
| --- | --- | --- | --- |
| `application/` | 8 | 1 494 | 业务逻辑：`cmd` / `chassis` / `gimbal` / `shoot` |
| `modules/` | 28 | 4 302 | 执行器、姿态、通信、算法 |
| `bsp/` | 12 | 870 | HAL 之上的薄封装 |
| `RTOSTask/` | 2 | 225 | 初始化与全部任务体 |
| **合计** | **50** | **6 891** | 手写业务代码，不含 HAL / FreeRTOS / USB 中间件与构建产物 |

### 1.3 设计取向

- **Instance 模式**：每个模块都是 `XXX_init(config*)` → `XXX_work()` → `XXX_getData()` 三步，实例指针私有、配置结构体按值拷入，模块之间不暴露内部变量。
- **拉模式解耦**：`chassis_work()` / `gimbal_work()` / `shoot_work()` 主动调用 `cmd_xxxGetCMD()` 取指令，`cmd` **不反向调用**任何机构的控制函数，避免头文件循环依赖。
- **按值返回**：所有 `*_getData()` 返回结构体副本而非指针，跨任务读取无需加锁（代价是瞬时不一致，可接受）。
- **错误码统一**：每个模块都有 `xxx_ok / xxx_error` 枚举，调用方可判定失败并降级。

---

## 2. 版本时间线

| 日期 | 提交 | 版本标识 | 主题 |
| --- | --- | --- | --- |
| 2026-09-27 | `f54c2d8` | — | 底盘跟随 v1 |
| 2026-09-27 | `38fcde4` | — | 麦轮底盘电控代码（解算同全向轮） |
| 2026-09-27 | `3670885` | 暑假短学期版 | 底盘 + 小陀螺，**无云台** |
| 2026-09-27 | `d9ef3f3` | — | README |
| 2026-10-01 | `2fe830e` `c87a1c7` | — | 更改 cmd：底盘跟随模式开启功能 |
| 2026-10-05 | `12c31b4` | — | 加入矩阵运算及卡尔曼滤波器 |
| 2026-10-06 | `a510607` `0b9ae3f` | — | IMU 卡尔曼滤波；宏切换变速小陀螺 / 云台角度反馈来源 |
| 2026-10-07 | `7068f3f` | — | 加入 USB 驱动 |
| 2026-10-09 | `4a6a056` `d6b4e5b` | — | 加入视觉通信协议；完善全部 `.h` 注释 |
| 2026-10-14 | `7859a56` | — | 重构视觉通信协议 |
| 2026-10-20 | `6228c3a` | — | 云台改由 IMU 闭环、PID 前馈改造、视觉断线恢复 |
| 2026-10-21 | `1e674ef` | — | IMU 恒温校准 + Mahony 融合；DJI 电机型号/CAN 口解耦；PWM 与发射机构骨架 |
| 2026-10-23 | `2e5c330` | — | 发射机构实装；前馈独立成 `feedback` 模块 |
| 2026-10-31 | `4096da6` | **V2.0** | 重写 `cmd` |
| 2026-11-02 | `3853388` | **V1.1.0** | 新增底盘（`Sentry-MDK-27` 工程树建立） |
| 2026-11-09 | `fb3960b` | **V2.0.0** | 新增云台及发射机构，待验证 |
| 2026-11-16 | `1343ab3` | **V2.1.0** | 底盘与发射机构完成上车验证，修复若干缺陷 |
| 2026-11-19 | 工作区（未提交） | **V2.1.1** | 云台 pitch 双轴上车验证，修复云台映射 / 使能 / 掩码缺陷 |

> 版本号在 11-02 之后重新起算（`V1.1.0` → `V2.0.0`），与 10-31 的「V2.0」不是同一条编号序列 —— 前者指工程重构批次，后者指整车功能版本，记录时需区分。
>
> V2.1.0 已产出提交 `1343ab3`；V2.1.1 尚无提交，本文以工作区相对 `1343ab3` 的差异为准。

---

## 3. 分阶段开发日志

### 3.1 2026-09-27 · 立项与底盘跑通

**目标**：让哨兵底盘先动起来，能跟随云台朝向。

**做了什么**

- 搭起 `application` / `modules` / `bsp` / `RTOSTask` 四层骨架，FreeRTOS 起任务；
- 完成底盘电机注册与速度单环：4× M3508 挂 CAN1，ID 1~4；
- 写出逆运动学解算，全向轮与麦轮**共用同一套公式**（`application/chassis.c:32`），差异只在轮组安装角度；
- 做出「底盘跟随 v1」：底盘按云台 yaw 偏角旋转，使车头始终对准云台指向；
- 打包成「暑假短学期版本」——底盘 + 小陀螺，**无云台闭环**。

**留下的资产**：`chassis_type_e` 里至今保留 `chassis_type_steering`（舵轮，`application/chassis.c:57` 仍是 `//todo`），说明当时的规划里还有第三种底盘。

### 3.2 2026-10-01 · 控制模式可选化

**据提交信息**：把「底盘跟随」从写死行为改为可用宏开关选择。

**当前体现**：`application/cmd.h:29` 仍保留 `//#define CMD_CHASSIS_FOLLOWING`（默认注释，即默认**不跟随**），以及 `CMD_MODE_CHASSIS_FOLLOW` / `CMD_MODE_CHASSIS_NO_FOLLOW` 两个并列的模式位（`application/cmd.h:80`）。这是本项目「编译期开关 + 运行期模式位」双层配置习惯的起点。

### 3.3 2026-10-05 ~ 10-06 · 姿态链路：矩阵库 + 卡尔曼

**目标**：不再直接拿陀螺仪积分当姿态角，改用带观测的滤波。

**做了什么**

- 自写 `modules/matrix.c` —— 小型定点规模矩阵运算（加、乘、求逆），供卡尔曼复用；
- 实现 `modules/kalman_filter.c` 三轴卡尔曼：roll/pitch 用「加速度解算角 + 陀螺角速度」**双观测**，yaw 只有陀螺角速度**单观测**；
- IMU 输出分裂成两套：原始解算量（`roll/pitch/yaw`）与滤波量（`filter_roll/filter_pitch/filter_yaw`），下游按需取用；
- 加入宏切换：变速小陀螺、云台角度反馈来源（IMU 或编码器）。

**当前体现**：`modules/IMU.h:42-56` 的 `IMU_config_s` 仍保留 `kal_roll/kal_pitch` 为 `kalman_config2_s`（双观测）、`kal_yaw` 为 `kalman_config1_s`（单观测）的类型区分，是这次设计的直接遗存。

### 3.4 2026-10-07 ~ 10-09 · 上位机链路 + 全工程文档化

**目标**：把视觉上位机接进来，同时把接口写清楚。

**做了什么**

- 引入 USB CDC（`bsp/bsp_usb.c` + `USB_DEVICE/`），把板子枚举成虚拟串口；
- 抽象出**通用帧格式** `modules/protocol.c`：`SOF | data_len | seq | CRC8 | cmd_id | data | CRC16`，明确它不是视觉专用，裁判系统也能套；
- `modules/vision.c` 在此之上做视觉专用收发与断线守护；
- 一次性补全所有 `.h` 的 Doxygen 注释——**这是本项目代码可读性的分水岭**，此后每个头文件都写明单位、调用周期、并发约束。

### 3.5 2026-10-11 ~ 10-14 · 视觉协议重构与控制位重排

**目标**：早期协议字段不够用，且位定义有歧义。

**做了什么**

- 重构视觉收发结构体：下行 `cmd_id + forward + turn + rotate + yaw + pitch`（5 个 float），上行 `cmd_id + yaw + pitch + free_data`（3 个 float）；
- **控制位从「扁平枚举」重排为「位域分区」**：`0xF000` 模式 / `0x0F00` 发射 / `0x00F0` 云台 / `0x000F` 底盘，比较前必须先掩码。

**代价**：这是一次**不兼容改动**（小陀螺从 `0x0002` 变为 `0x0020` 等），上位机必须同步。当前 `application/cmd.h:68-91` 的注释仍在强调「先掩码再比较」。

### 3.6 2026-10-15 ~ 10-20 · 云台改由 IMU 闭环 + 前馈改造 + 断线恢复

**目标**：编码器闭环带不动云台（间隙、噪声），改用 IMU 姿态角做位置反馈。

**做了什么**

- `gimbal_work()` 的位置反馈换成 `IMU.filter_yaw` / `IMU.filter_pitch`，速度反馈仍用编码器（IMU 陀螺噪声大，只适合做位置）；
- PID 加入前馈通道 `f_out`，由外部写入；
- 视觉断线从「锁死」改为「超时停控、收到下一帧自动恢复」。

**遗留**：此阶段 `f_out` 仍由 PID 内部按 `f_kp × target` 计算，属于「半前馈」；10-23 才彻底拆走。

### 3.7 2026-10-21 · IMU 恒温校准 + Mahony + 电机接口解耦

这是**单次改动量最大**的一次提交，四件事并行。

**(1) 上电恒温校准**（`modules/IMU.c`，状态机四态）

```
加热(heating) → 到温稳定(settling, 500 周期) → 采样零偏(collecting, 1000 周期) → 完成(done)
```

- 目标温度 40 ℃，用 TIM10_CH1（PF6）PWM 驱动加热电阻，PID 输出占空比 0~1；
- 静止判定：采样窗口内三轴 gyro 峰峰值 < 2 °/s **且** 加速度模长在 0.8~1.2 g 之间；
- 兜底：等温超时 600 000 次（约 10 min）则「用当前温度校准」，但把 `valid` 标 0 供上层判断（`modules/IMU.h:26`）。

**(2) Mahony 融合**（`modules/mahony.c`）

新增 6 轴 Mahony 作为卡尔曼的**可选替代**（`IMU_config_s::filter_type`），参数 `kp = 1.0`、`ki = 0`、`dt = 0.00114`（`RTOSTask/robotTask.c:56-60`）。工程里**当前实际启用的是 Mahony**，尽管 `modules/IMU.h:34` 的枚举注释仍写着「卡尔曼（本工程默认）」——文档与配置已经漂移，见 §6.3。

**(3) DJI 电机接口解耦**

`DJI_motor_motorID_s` 从「型号 + ID」扩展为「**型号 + ID + CAN 口**」（`modules/DJI_motor.h:40-46`），所有接口（`enable` / `disable` / `setTarget` / `setFeedback` / `setPidConfig` / `getData` / `setPidMode`）统统带上 `CAN_HandleTypeDef*`。原因很朴素：**同型号同 ID 的电机可能挂在不同总线上**，只靠 ID 无法区分。

**(4) 发射机构骨架**

建立 `application/shoot.c/.h` 的框架（当时 34 行），先把状态位和接口定下来。

### 3.8 2026-10-23 · 发射机构实装 + 前馈独立成模块

**目标**：发射机构从骨架变成能用；把前馈从 PID 里赶出去。

**(1) 前馈模块化**（新增 `modules/feedback.c/.h`）

`pid_instance_s` 里删掉了 `f_kp` 与 `last_target`，PID **不再自己算前馈**；`f_out` 变成纯输入端口，由 `feedback_calculate()` 写入，`pid_calculate()` 只负责求和、不清零也不修改（`modules/pid.h:6`、`:90`、`:126`）。

`feedback` 独立支持 4 种模式，可用位掩码组合（`modules/feedback.h:25-28`）：

| 模式位 | 作用 |
| --- | --- |
| `feedback_mode_velocity` | `k_v × 参考速度` |
| `feedback_mode_acceleration` | `k_a × 参考加速度` |
| `feedback_mode_friction` | 按参考速度方向输出 `±k_s`，零速附近线性过渡，避免抖动 |
| `feedback_mode_gravity` | `gravity_gain × cos(角度 − 相位)` |

**设计理由**（写在头文件里）：前馈与重力补偿的模型、整定节奏与 PID 完全不同，拆开后便于单独测试和跨机构复用。

**(2) 发射机构实装**（`application/shoot.c`，34 → 178 行）

| 项 | 配置 |
| --- | --- |
| 摩擦轮 | 2× M3508，CAN2 ID 3 / 4，目标转速 ±`fric_speed`（左右反向） |
| 拨盘 | M2006，CAN2 ID 2，目标转速 `dial_speed × 60/9` rpm（9 孔） |
| 使能状态 | `shoot_workStatus_e` 位掩码：bit0 = 摩擦轮，bit4 = 拨盘 |
| 安全逻辑 | `SHOOT_FRIC_DISABLE_SPEED = 1.0` rpm —— **摩擦轮降到 1 rpm 以下才允许断电**，否则只发 0 目标让它自然减速 |

最后一条是本项目最典型的「机械保护优先于响应速度」取舍：高速摩擦轮直接断电会带来剧烈反冲和轴承冲击，宁可多花一秒减速。

### 3.9 2026-10-31 · V2.0：重写 `cmd`

**目标**：`cmd` 是整车最脏的地方——所有模式分支、所有限幅、所有 PID 都堆在一个文件里，改一处功能要读三百行。

**做了什么**

- 把 `cmd` 从「一堆并列全局变量 + 散落函数」重写为**单一实例结构体**（`application/cmd.c:4-44`），配置项（速度上限、角度范围、死区、阈值）、控制量（三份 `xxxCMD`）与状态（PID 实例、`workStatus`）全部收拢；
- 抽出三条**职责单一**的流水线：`cmd_chassisUpdata()` / `cmd_gimbalUpdata()` / `cmd_shootUpdata()`，统一负责「限幅 + workFlag=0 时清零」；
- 把拨杆到模式的映射独立成 `cmd_remote_to_mode()`，回归纯函数（不产生控制量，只吐模式字）；
- 三个控制分支（遥控 / 视觉 / 键鼠）各自只负责**填三份 raw 指令**，再统一走 Updata，保证限幅逻辑只有一份。

**结构上的收益**：新增一种控制源，只需再写一个 `cmd_xxxControl()` 填 raw 结构体，不用碰限幅与解算。

**注意**：`4096da6` 这一提交还顺带把工程树整理成 `Sentry-MDK-V2.0/`，其中只保留 `cmd`，底盘/云台/发射都被摘掉——这是**刻意的减法**，先让指令层单独编译通过，再逐个机构接回来。

### 3.10 2026-11-02 · V1.1.0：新增底盘（`Sentry-MDK-27` 立项）

**目标**：在 V2.0 的新指令层上，把底盘重新接回。

**做了什么**

- 建立 `Sentry-MDK-27/` 工程树，作为 2027 赛季主工程；
- 底盘重新实装（`application/chassis.c`）：
  - 全向轮 / 麦轮共用逆解，`chassis_type_omni` 为当前配置（`application/chassis.c:74`）；
  - 机械参数：底盘半径 280 mm、轮毂半径 78.9 mm、小陀螺目标 5 rad/s；
  - 四轮速度单环：`kp = 400`、`ki = 5`、`max_i = 1000`、`maxout = 8000`；
- 新增 `.analysis_yaw/` 工具链——**用 Python 把实车日志拉出来做系统辨识与滤波对比**（`cascade.py` / `plantfit.py` / `graysim.py` / `data*_filters.py` 等），生成 `d2_z*` / `d3_z*` / `d4_z*` 一组图，用于确定 yaw 轴闭环的 kp 与滤波强度。

**方法论上的变化**：从这一阶段开始，**整定参数先离线拟合、再上车验证**，而不是上车试凑。`.analysis_yaw/` 里的图片就是每次整定的证据链。

### 3.11 2026-11-09 · V2.0.0：新增云台与发射机构（待验证）

**目标**：补齐云台三轴与发射机构，整车功能闭环。

**(1) 云台改为三轴 DM 电机**（新增 `application/gimbal.c/.h`，236 行）

| 轴 | 电机 | 控制模式 | 反馈来源 |
| --- | --- | --- | --- |
| yaw | DM-J4310，CAN1 | MIT + 位置环 | `IMU.filter_yaw`（deg → rad） |
| big pitch | DM-J4310，CAN1 | MIT + 位置环 | 编码器，**目标恒为 π/2** |
| small pitch | DM-J4310，CAN1 | MIT + 位置环 | `IMU.filter_pitch`（deg → rad） |

关键设计：

- **反馈可外部接管**：`DM_motor_setFeedbcak()` 允许调用方覆盖测量值，于是云台闭环可以直接吃 IMU 姿态角，绕开减速器间隙；
- **失能前先回零位**：`workFlag == 0` 时不立刻断电，而是先让三轴回 0（small pitch 用 IMU 闭环），**三轴都进入 ±0.5 rad 判定范围**后才真正失能（`application/gimbal.c:181-203`），避免云台自由下落砸到限位；
- **使能沿检测**：`last_workStatus` 记录上一周期状态，只在 0→1 的边沿执行一次「big pitch 置 π/2」的初始化动作。

**(2) 发射机构重新接入**

在 10-23 版本基础上适配新的 `cmd` 结构，五档状态（0 失能 / 1 仅使能 / 2 摩擦轮 / 3 连发 / 4 单发）。

**(3) 底盘与 `cmd` 的对接调整**

- `cmd` 增加 `gimbal_getStatus()` / `shoot_getStatus()` 汇总，与 `chassis_getStatus()` 一起打包成 16 位 `cmd_id` 回传上位机（`application/cmd.c:574-584`）；
- 新增 `IMU_setYawOffset()` 的调用：`cmd_work()` 首周期读 yaw 电机编码器，把「编码器角 − 机械中位角」写给 IMU，使 `filter_yaw = 0` 对应云台机械中位。

**状态**：代码完整但**未上机验证**（提交信息明确标注「待验证」），第 6 章列出上机前必须处理的阻塞项。

### 3.12 2026-11-15 · V2.1.0：底盘与发射机构上车验证

**里程碑**：V2.0.0 留下的两个「待验证」机构——**底盘与发射机构**——完成实车验证，闭环可用；验证过程中暴露的缺陷同步修复。**云台本次未改动，仍未验证**。

#### 一、先解决「看不见」：给底盘补一条观测量

上车验证的第一步不是调参，而是能同时看到**目标**和**实际**。`serialTask` 此前是一整片注释掉的调试代码，本版改成实装的双列 CSV（`RTOSTask/robotTask.c:190-218`）：

```c
DJI_motor_getData(chassis_2_ID, &chassis_2);      // 2 号轮实测转速 rpm
chassis_data_s chassis = chassis_getData();       // 逆解目标 rpm
Serial_Printf(&huart6, "%.6f,%.6f\n", chassis_2.speed, chassis.chassis_speed[1]);
```

- 左列 = **实测**（M3508 反馈解析后的输出轴 rpm）；
- 右列 = **目标**（逆解算出的 2 号轮 rpm）；
- 5 ms 一行，115200 波特率下可整段导出做阶跃/斜坡分析。

为此给 `chassis` 补了对外接口 `chassis_getData()` 与 `chassis_data_s`（`application/chassis.c:268-276`、`chassis.h:29-32`）——此前 `chassis_instance_s` 完全私有，外部拿不到解算结果，这是**观测能力**而非控制逻辑的补齐。

> 这条打印线在正式比赛版本中应条件编译掉（见 §6.3#7）。

#### 二、发射机构：从「逻辑走不通」到「能打出去」

上车验证暴露的第一个问题不是参数，而是**发射机构压根没被使能**。

**缺陷 1：s1 拨上时发射位域永远为 0**

```c
// V2.0.0
case 1:
  if((cmd_mode & 0xF000) == CMD_MODE_REMOTE)          // ← 此刻 cmd_mode 还是 CMD_MODE_OFF
    cmd_mode = (cmd_mode & 0xF0FF) | CMD_MODE_SHOOT_ON;
  break;
```

`cmd_mode` 进入函数时的初值是 `CMD_MODE_OFF = 0`，`0 & 0xF000` 恒为 0，条件**永远不成立**——s1 拨上等于这一档什么都没做。接下来 s2 分支只在 `0xFFF0` / `0x0FF0` 掩码内改低位，**`0xF000` 优先级位域始终是 0**，于是 `cmd_work()` 拿到 `priority == CMD_MODE_OFF`，走 `cmd_all_disable()` 直接返回。

也就是说 **V2.0.0 里 s1 拨到「上」时整车失能**——不是发射打不出去，是连底盘都不动。只有 s1 拨中（遥控）或拨下（视觉）能出模式。这个现象在台架上不明显，一上车就会被当成「整机没反应」。

V2.1.0 改为直接整字赋值（`application/cmd.c:51-72`），顺带让模式切换时清空发射位域，语义更干净：

```c
case 1:  cmd_mode = (cmd_mode_e)(CMD_MODE_SHOOT_ON | CMD_MODE_REMOTE);  break;
case 2:  cmd_mode = CMD_MODE_VISION;                                     break;
case 3:  cmd_mode = CMD_MODE_REMOTE;                                     break;
```

改后 s1 拨上 = `0x1100`（遥控 + 发射机构总开关），`priority = 0x1000` 正常进入遥控分支，拨轮才有机会触发摩擦轮与开火。

**缺陷 2：拨轮无条件解析**

V2.1.0 给拨轮解析套了一层总开关，只有发射机构「已开启」时才允许拨轮触发摩擦轮 / 连发 / 单发（`application/cmd.c:97-108`），避免底盘模式下手误碰拨轮。

**缺陷 3：拨盘转速把「发/秒」当成了 rpm**

```c
// V2.0.0：dial_speed 被直接当 rpm 下发
DJI_motor_setTarget(dial, 0, cmd.dial_speed);
```

V2.1.0 引入孔位换算，`dial_speed` 从此表示**发弹频率 Hz**：

```c
#define SHOOT_DAIL_CNT        9.0f                        // 拨盘孔位数（V2.0.0 为 8）
#define SHOOT_DIAL_HZ_TO_RPM  (60.0f / SHOOT_DAIL_CNT)    // n 发/秒 → 拨盘 rpm
...
DJI_motor_setTarget(dial, 0, cmd.dial_speed * SHOOT_DIAL_HZ_TO_RPM);
```

`cmd.dial_speed = 30` 即「30 发/秒 → 拨盘 200 rpm」，量纲第一次对得上。孔位数由 8 修正为 9——这个数字直接决定射频标定是否准确，是本次验证里**最容易被忽略但影响最大**的一处。

**缺陷 4：切到「仅摩擦轮」时拨盘不停**

V2.0.0 的 `workFlag == 2` 分支只下发摩擦轮目标，拨盘保留上一拍的值；从连发切回摩擦轮时拨盘仍在转。V2.1.0 显式补了一行清零（`application/shoot.c:151`）。

#### 三、底盘：跟随改编译期开关，不跟随支持手动自转

`cmd.h` 新增宏段，把「底盘是否跟随云台」变成显式开关：

```c
//#define CMD_CHASSIS_FOLLOWING     // 默认注释 = 不跟随
```

模式解析补上 `#else` 分支（`application/cmd.c:86-92`）。V2.0.0 里宏根本不存在、代码却硬写 `CMD_MODE_CHASSIS_FOLLOW`，等于**跟随永远开启**，而此时的跟随环读的是一个从未注册的 GM6020（§6.1#3），输出不可预测——这解释了上车前为什么必须先把它改成不跟随。

同时 `CMD_MODE_CHASSIS_NO_FOLLOW` 分支从「自转恒为 0」补成由 `ch0` 连续控制（`application/cmd.c:322-326`）：

```c
chassisCMD.chassis_rotate = cmd->chassis_rotate_speed * remote_ch.ch0 / (660.0f - cmd->remote_dead);
```

#### 四、PID：积分限幅改为钳制累积项

`modules/pid.c:130-134` 把积分限幅的作用对象从 `i_out` 改成 `i_term`：

```c
// V2.0.0：只钳 i_out，i_term 仍可无限累积
if(pid_instance->i_out > pid_instance->max_i) pid_instance->i_out = pid_instance->max_i;

// V2.1.0：直接钳累积项（等价于限制 i_out 的上限为 max_i）
if(pid_instance->i_term >  pid_instance->max_i / pid_instance->ki) ...
```

这正是 `modules/pid.h:53` 早就写明的隐患——「`max_i` 只限 `i_out`，不限制 `i_term` 累积」。改后长时间饱和再退饱和的滞后明显改善。

⚠️ 但引入了新约束：**`ki = 0` 时该钳位失效**（`max_i / 0 = inf`；若 `max_i` 也为 0 则得 `NaN`，比较恒假）。此时 `i_out ≡ 0` 无实际影响，但语义上应显式保护，见 §6.3#17。

另：去掉了 `pid.c` 文件头的 UTF-8 BOM，与其余源码统一。

#### 五、参数整定：上车实测后回改

| 项 | V2.0.0 | **V2.1.0** | 位置 | 方向 |
| --- | --- | --- | --- | --- |
| 底盘速度环 `kp` / `ki` | 450 / 7 | **400 / 5** | `chassis.c:93-94` | 降刚度，抑制轮速纹波 |
| 摩擦轮速度环 `kp` | 40 | **60** | `shoot.c:36` | 提刚度，转速更硬 |
| 拨盘速度环 `ki` / `max_i` | 5 / 6000 | **1 / 3000** | `shoot.c:59,61` | 弱积分，防过冲 |
| 摩擦轮目标转速 | 350 rpm | **80 rpm** | `cmd.c:479` | 回到可用射程 |
| 拨盘发弹频率 | 100 Hz | **30 Hz** | `cmd.c:480` | 回到可用射频 |
| 开火拨轮阈值 | 500 | **600** | `cmd.c:490` | 防误触 |
| `small_yaw_init`（小 yaw 机械中位） | 120 | **0** | `cmd.c:482` | 云台基准重标定 |
| `pitch_init`（pitch 机械中位） | 53 | **0** | `cmd.c:483` | 同上 |

几处变化的共同倾向：**积分项全面减弱**（底盘 ki 7→5、拨盘 ki 5→1、拨盘 max_i 6000→3000），这是实车出现「过冲 / 低频摆动」时的典型处理；摩擦轮 `kp` 反向提高，是因为摩擦轮只需要硬转速、不需要平滑。

#### 六、其它

- `modules/DJI_motor.h:15` 减速比宏补括号：`268.0f/17.0f` → `(268.0f/17.0f)`，消除预处理展开时的运算优先级隐患。
- `application/shoot.h:27-28` 给 `shoot_ok / shoot_error` 显式赋值（`0x01 / 0x00`）。
- **IMU 校准门禁被注释**（`application/cmd.c:554-559`）：V2.0.0 里这段是**启用**的，V2.1.0 主动注释掉了。合理推测是不想让底盘与发射在等 IMU 升温到 40 ℃ 的数十秒里被锁死；但这同时意味着云台可以在未标定状态下被使能，风险见 §6.1#5。

#### 本版修复清单

| # | 缺陷 | 影响 |
| --- | --- | --- |
| 1 | s1 拨上时模式字优先级位域恒为 0 | **整车失能**（不只发射机构） |
| 2 | 拨轮无条件触发发射 | 底盘模式下手误碰拨轮即开火 |
| 3 | 拨盘转速量纲错误（Hz 当 rpm） | 射频偏差约 6.7 倍 |
| 4 | 拨盘孔位数 8 → 9 | 射频标定错误 |
| 5 | 「仅摩擦轮」时拨盘不清零 | 切模式时拨盘续转 |
| 6 | 底盘跟随硬编码为开启 | 跟随环读到未注册电机，行为不可预测 |
| 7 | 不跟随模式自转恒 0 | 该模式无自转能力 |
| 8 | PID 积分限幅钳错对象 | 长时间饱和后退饱和滞后 |
| 9 | 减速比宏缺括号 | 展开优先级隐患 |

#### 遗留

- **云台仍未上车验证**（`gimbal.c` 本次未改动）。
- §6.1 中 **#1 小陀螺、#2 DM 三轴同 ID、#3 跟随环 GM6020、#4 yaw 修正锁存、#7 DM kd 丢位、#8/#9 缓冲区越界、#10 视觉开火不可达、#11 位语义相反** 均未修复。
- 连发（`workFlag == 3`）仍不下发摩擦轮目标，实操依赖「先开摩擦轮再连发」的顺序（§6.1#6）。

> 以上是 V2.1.0 收尾时的状态；其中 #2 三轴同 ID、#4 yaw 修正锁存、#5 门禁三项目前已在 V2.1.1 解决，见 §3.13。

### 3.13 2026-11-19 · V2.1.1：云台 pitch 上车验证

**里程碑**：继 V2.1.0 的底盘与发射机构之后，**云台 pitch 两轴完成上车验证**——big pitch 稳定保持抬头姿态、small pitch 能跟随遥控 pitch 指令双向运动、失能后能自行回零。云台「pitch 闭环从未在实车跑通过」的状态到此结束。

**验证范围**：仅 big pitch / small pitch 两轴。**yaw 本版主动不使能**（见第四节），因此云台整体仍不算验证通过。

#### 一、先让三台电机「各归各号」

V2.0.0 起云台三轴的 DM 配置是复制粘贴的产物：三份全是 `ID = 0x05`、`MST_ID = 0x01`、`&hcan1`（§6.1#2）。`DM_motor_pairInstance()` 按 ID 线性查找、返回首个匹配，于是三份 `setPidConfig` / `setFeedbcak` 全部落到第一个实例上，`DM_Motor_pidWork()` 也把三帧发往同一个 CAN ID —— 三轴里只有一台电机收得到指令。

| 轴 | V2.0.0 ~ V2.1.0 | **V2.1.1** | 总线 |
| --- | --- | --- | --- |
| yaw | `ID 0x05` / `MST 0x01` | **`0x01` / `0x11`** | CAN1 → **CAN2** |
| big pitch | 同上 | **`0x02` / `0x12`** | 同上 |
| small pitch | 同上 | **`0x03` / `0x13`** | 同上 |

`MST_ID` 是电机回传帧的 ID（`modules/DM_motor.c:136` 按它配对接收回调），所以「发送 ID 顺排 `0x0x`、回传 ID 顺排 `0x1x`」是一次把硬件拨码与软件表对齐的操作。总线整体从 CAN1 挪到 CAN2，与底盘电机分处两条总线。

同时按实测重设 MIT 帧的物理上限：pitch 两轴 `VMAX 10 → 30` rad/s、`TMAX 28 → 10` N·m；位置环 `maxout` 由 100 收到 **3.0 / 9.0** N·m。注意 DM 的 MIT 打包对超限帧是**整帧拒发**而不是限幅（`modules/DM_motor.c:198-203`）：旧配置 `maxout = 100` 是 `TMAX = 10` 的十倍，一旦输出越界，电机直接收不到这一帧；收到 3.0 / 9.0 之后，位置环的限幅才真正落在电机能接受的范围内。

#### 二、pitch 闭环的三处映射错误

这三处只有上车才会暴露——代码本身自洽，错的是「代码 ↔ 机械」的对应关系。

**1. 反馈读错轴**：small pitch 一直拿 `IMU.filter_pitch` 当位置反馈，但实车上该轴的角度变化落在 IMU 的 **roll** 上（安装方向与机体坐标不重合）。位置环测的因此是另一个轴的角度，闭环实际处于开环甚至正反馈。V2.1.1 两处调用（含回零路径）全部改为 `filter_roll`（`application/gimbal.c:185,221`）。

**2. 目标符号相反**：`cmd.pitch` 以「抬头为正」，而电机侧的正方向与「抬头」相反（实车确认）。V2.1.1 在 `application/gimbal.c:216` 取反：

```c
gimbal->data.smallPitch_angle = -cmd.pitch * GIMBAL_DEG_TO_RAD;
```

**3. 量程与零点对不上**：`pitchAngle` 仍是 53° ± 20°，而 `pitch_init` 已在 V2.1.0 改成 0，限幅落在 33°~73° 这段与机械零点无关的区间。V2.1.1 改成 **±60°**（`application/cmd.c:489-490`），与零点一致。

三处改完，「遥控推杆 → `cmd` 限幅 → 电机目标角」才第一次同向、同量纲。

#### 三、big pitch 从 90° 收到 86°

`big pitch` 的目标此前恒为 `π/2`（90°），是纸面值；实车上 90° 会顶到结构限位。V2.1.1 改为 **86°**（`application/gimbal.c:211,215`），留 4° 余量。这是本版唯一按机械实测回改的数值，其余都是逻辑修正。

#### 四、使能链路：云台不再「跟着遥控器一起上电」

V2.1.0 的 `cmd_remoteControl()` 一进遥控分支就把 `gimbalCMD.workFlag` 置 1，等于只要 s1 不在 OFF 档云台就使能——调试 pitch 时无法单独断电，任何档位切换都会连带云台动作。

V2.1.1 改成默认 0、只在 s2 拨中（`CMD_MODE_GIMBAL_ON`）置 1（`application/cmd.c:298,312-316`）：

```c
gimbalCMD.workFlag = 0;
...
else if((*cmd_mode & 0x00F0) == CMD_MODE_GIMBAL_ON)
  gimbalCMD.workFlag = 1;
```

配套在 s2 拨中分支补写云台位域（`application/cmd.c:93`）：`cmd_mode = (cmd_mode & 0xFF0F) | CMD_MODE_GIMBAL_ON;`。此前该分支只写底盘位域 `0x000F`，云台位域恒为 0，上面的判断永远不成立。

另外 `gimbal_enable()` 里的 yaw 由 `enable` 改成 `disable`（`application/gimbal.c:139`）：本版只验证 pitch 两轴，yaw 不参与，避免云台在无人看管时自己转。`gimbal_disable()` 仍是三轴全发。

#### 五、掩码写错半字节：`0xFFF0` → `0xFF0F`

模式字的位域划分是 `0xF000 模式 | 0x0F00 发射 | 0x00F0 云台 | 0x000F 底盘`（`application/cmd.h:69`）。s2 拨上（小陀螺）分支原本用 `cmd_mode & 0xFFF0` 清位，掩掉的是**底盘**位域，该清的**云台**位域反而留着：

```c
// V2.1.0：清错位域
cmd_mode = (cmd_mode_e)((cmd_mode & 0xFFF0) | CMD_MODE_TOP);
// V2.1.1
cmd_mode = (cmd_mode_e)((cmd_mode & 0xFF0F) | CMD_MODE_TOP);
```

⚠️ 这条修正**并没有救活小陀螺**。`s1` 分支是整字赋值（`cmd_mode = CMD_MODE_SHOOT_ON | CMD_MODE_REMOTE` → `0x1100`），底盘位域每拍都被重置成 0；`cmd_remoteControl()` 里 `(cmd_mode & 0x000F) == CMD_MODE_OFF` 依然成立，`chassisCMD.workFlag` 照样被清 0。`top_speed` 也仍然没有消费者（`application/chassis.c:240` 整段注释）。§6.1#1 只是掩码语义变对了，两个真正的成因一个都没动。

#### 六、失能判定：量纲修正，以及一条被摘掉的轴

`GIMBAL_DISABLE_RANGE` 在 `application/gimbal.h:16` 标的是 **rad**（0.5），但实际语义是 **deg**。V2.1.0 直接拿它和编码器 rad 比较，等效判定范围 ±28.6°，「三轴都回零位」形同虚设；V2.1.1 统一乘 `GIMBAL_DEG_TO_RAD`（`application/gimbal.c:189,196`），判定收紧到 **±0.5°**。

⚠️ 但 yaw 从使能列表摘掉之后，失能流程出现一条走不通的路：判断是串行的 `else if` 链，且**先判 yaw**（`application/gimbal.c:189`）。yaw 不再使能、位置不会回零，只要它停在 ±0.5° 之外，`gimbal_disable()` 就永远轮不到，`workStatus` 会一直报 1。挂 yaw 之前必须把未使能轴从判定里跳过——已列入 §6.1#12。

#### 七、IMU 校准门禁恢复

V2.1.0 为了不让底盘与发射在等 IMU 升温的数十秒里被锁死，把 `IMU_isCalibrated()` 判断整段注释（§6.1#5）。V2.1.1 重新启用（`application/cmd.c:558-562`）：

```c
if(IMU_isCalibrated() == 0)
{
  cmd_all_disable();
  return;
}
```

代价是上电后整车锁定数十秒；收益是 §6.1#4 一并消失——`calib_applied` 现在只在门禁放行之后才执行，`IMU_setYawOffset()` 读到的是**校准完成时**的 yaw 编码器，修正量第一次有意义。

#### 八、观测量跟着验证对象走

`serialTask` 的双列 CSV 从底盘切到云台（`RTOSTask/robotTask.c:215-219`）：

```c
DM_motor_getData(0x2, &big_pitch);
DM_motor_getData(0x3, &small_pitch);
Serial_Printf(&huart6,"%.6f, %.6f\n", small_pitch.speed, big_pitch.position);
```

左列 small pitch 实测**转速**、右列 big pitch 实测**位置**，5 ms 一行。和 V2.1.0 调底盘时是同一套方法：先能看见，再谈调参。缺点是两列是两个物理量、也没有对应目标值——下一步要看 pitch 跟随误差，应改成「目标角, 实测角」。

#### 九、温控：去掉积分

IMU 加热 PID `kp 0.20 → 0.10`、`ki 0.0032 → 0`（`RTOSTask/robotTask.c:75-76`）。升温是单向过程，积分只会在接近目标前把占空比顶到饱和，容易出现温度过冲与反复通断；只留 P 之后通断更平缓。

#### 本版修复清单

| # | 缺陷 | 影响 | 位置 |
| --- | --- | --- | --- |
| 1 | 云台三轴 DM 同 ID、同总线 | 三份配置指向同一台电机，另两轴收不到目标 | `gimbal.c:27,62,97` |
| 2 | small pitch 位置反馈取 `filter_pitch` | 闭环读错轴，pitch 不收敛 | `gimbal.c:185,221` |
| 3 | pitch 目标符号与机械方向相反 | 推杆方向反了 | `gimbal.c:216` |
| 4 | pitch 量程仍是 53°±20°，与零点不符 | 限幅区间偏离机械零点 | `cmd.c:489-490` |
| 5 | big pitch 目标 90° 顶结构限位 | 使能时顶死结构 | `gimbal.c:211,215` |
| 6 | 遥控模式下云台无条件使能 | 无法单独断电，切档连带云台动作 | `cmd.c:298,315` |
| 7 | s2 拨上掩错位域（清底盘、留云台） | 模式组合切换时位域脏 | `cmd.c:78` |
| 8 | 失能阈值把 deg 宏当 rad 用 | 判定范围 ±28.6°，回零判定失效 | `gimbal.c:189,196` |
| 9 | 位置环 `maxout` 100 远超 `TMAX` | 输出越界时 MIT 帧整帧被拒发，力矩并未被限幅 | `gimbal.c:88,123` |
| 10 | IMU 校准门禁被注释 | 未校准即闭环，云台甩头；yaw 修正量错 | `cmd.c:558-562` |
| 11 | 温控积分饱和 | 升温过冲、通断振荡 | `robotTask.c:75-76` |

#### 遗留

- **yaw 未验证**，且失能流程卡在 yaw（§6.1#12）——挂 yaw 之前必须先改这里。
- **小陀螺仍不工作**：§6.1#1 的两个成因（底盘位域被 s1 整字清零、`top_speed` 无消费者）都还在。
- **§6.1 其余未修**：#3、#6、#7、#8、#9、#10、#11。
- **pitch 参数是手调值**：big pitch `kp = 6`、small pitch `kp = 22` 来自上车试凑，尚无阶跃 / 频响数据支撑，建议按 `.analysis_yaw/` 的方法补一份辨识记录。
- **yaw 目标角没有 ±π 限幅**：`cmd->yawAngle` 在 `cmd_init()` 里定义了却从未被使用（`cmd.c:487-488`），`small_yaw` 由遥控 / 视觉累加；一旦超过 `PMAX = π`，`DM_motor_sendMIT()` 会整帧拒发（`DM_motor.c:198-203`）。挂 yaw 之前必须补。
- **本版新增文档债**：`gimbal.h:16`（`GIMBAL_DISABLE_RANGE` 仍标 rad）、`gimbal.h:28`（big pitch 仍写「固定 π/2」）、`gimbal.c:17`（仍写「当前均为 0x05/0x01」）、`robotTask.c:218`（取了 `IMU` 未使用）。
- **发射机构本版未改逻辑**（`shoot.c` 只加了注释），V2.1.0 的结论继续有效。

---

## 4. 当前架构

### 4.1 分层与依赖方向

```
┌─ RTOSTask/       robot_init() + 9 个任务体
├─ application/    cmd ←→ chassis / gimbal / shoot        业务与解算
├─ modules/        DJI_motor / DM_motor / pid / feedback   执行器与算法
│                  BMI088 / IMU / kalman_filter / mahony / matrix   姿态链路
│                  vision / protocol / crc / remote / deamon        通信与输入
├─ bsp/            bsp_can / bsp_spi / bsp_usb / bsp_pwm / bsp_dwt / serial
├─ USB_DEVICE/     CubeMX 生成（CDC）
└─ Core + Drivers + Middlewares   CubeMX + HAL/CMSIS + FreeRTOS
```

**依赖规则**：

- 向下单向：`application → modules → bsp → HAL`；
- **`bsp` 不认识 `application`**：需要回调时由上层在 `init` 时注册函数指针（如 PWM 的 `pwm_moduleCallback`、USB 的接收回调），避免底层反向包含；
- **机构之间互不包含**：`chassis` / `gimbal` / `shoot` 只认 `cmd` 的头文件，彼此之间没有直接调用。

### 4.2 目录职责

| 路径 | 内容 |
| --- | --- |
| [Core/Src](Core/Src) | `main.c`（时钟/外设初始化）、`freertos.c`（任务创建与优先级）、`can.c`、`spi.c`、`tim.c`、`usart.c`、`dma.c`、`gpio.c` |
| [bsp](bsp) | `bsp_can`（实例 + 接收回调分发）、`bsp_spi`（多实例带 CS）、`bsp_usb`（CDC 软开关）、`bsp_pwm`（多实例占空比）、`bsp_dwt`（微秒延时）、`serial`（DMA + IDLE 收发 + printf） |
| [modules](modules) | 执行器（`DJI_motor` / `DM_motor`）、算法（`pid` / `feedback` / `matrix` / `kalman_filter` / `mahony`）、姿态（`BMI088` / `IMU`）、通信（`protocol` / `crc` / `vision`）、输入与守护（`remote` / `deamon`） |
| [application](application) | `cmd`（指令解析与模式）、`chassis`（逆解 + 四轮速度环）、`gimbal`（三轴位置环）、`shoot`（摩擦轮 + 拨盘） |
| [RTOSTask/robotTask.c](RTOSTask/robotTask.c) | `robot_init()` 与全部任务体 |
| [MDK-ARM](MDK-ARM) | Keil 工程 `Sentry-MDK.uvprojx` 与构建产物 |

### 4.3 FreeRTOS 任务表

| 任务 | 栈 (B) | 优先级 | 周期 | 入口 | 职责 |
| --- | --- | --- | --- | --- | --- |
| `IMU` | 2048 | Low | **1 ms** | `IMU_work()` | BMI088 采样、姿态滤波、温控、校准 |
| `Motor` | 2048 | Normal | **2 ms** | `DJI_Motor_work()` + `DM_Motor_pidWork()` | DJI 速度/位置环 + DM 闭环发帧 |
| `Gimbal` | 2048 | Normal2 | **2 ms** | `gimbal_work()` | 三轴目标刷新与反馈接管 |
| `Cmd` | 2048 | **AboveNormal** | **4 ms** | `cmd_work()` | 读遥控/视觉、算模式、写指令、回传状态 |
| `Chassis` | 2048 | Normal2 | **5 ms** | `chassis_work()` | 逆解 + 四轮目标下发 |
| `Shoot` | **512** | Normal | **5 ms** | `shoot_work()` | 摩擦轮/拨盘使能与目标 |
| `Deamon` | 2048 | Normal2 | **10 ms** | `deamonStart()` | 模块掉线检测 |
| `Serial` | 2048 | Low | 5 ms | 调试打印 | 当前打印 big pitch 位置与 small pitch 转速（V2.1.1 起） |
| `defaultTask` | 512 | Normal | 1 ms | 空转 | CMSIS 默认任务占位 |

**周期选取的依据**：

- `IMU` 1 ms 是**姿态解算的节拍**，`IMU_config_s::dt = 0.001` 必须与之相符，否则滤波发散；
- `Motor` 2 ms 是**达妙 DM 电机的硬约束**——MIT/PVT/闭环帧若超过 2 ms 未发，电机侧触发通信超时保护（`modules/DM_motor.h:4`）；
- `Cmd` 4 ms 比 Chassis(5) / Shoot(5) 快，保证**指令永远比执行新**；
- `Cmd` 优先级最高（AboveNormal），因为它要汇总三机构状态并驱动 USB 发送，卡顿会连带拖慢上位机闭环。

> ⚠️ 多个任务并发读写 `cmd` 的内部结构体（`Cmd` 写、其余读），**当前没有任何互斥保护**。4 ms vs 5 ms 的周期差意味着读到的必然是「上一拍或上上拍」的完整结构体（Cortex-M4 上 32 位对齐的 float 读写是原子的），实际风险可控，但结构体整体一致性无保证——列为 §6.3 改进项。

### 4.4 一帧控制的数据流

```
遥控器 ──USART3/DMA──▶ remote_getData()  ─┐
                                          ├─▶ cmd_work() ──▶ cmd->chassisCMD ──▶ chassis_work()
视觉上位机 ──USB CDC──▶ vision_getData() ─┘        │                              ├─▶ gimbal_work()
                                                  │                              └─▶ shoot_work()
                                                  └──▶ vision_sendData(状态 + IMU 角度) ──USB──▶ 上位机
```

具体到 `cmd_work()`（`application/cmd.c:552`）的单次执行：

1. **回传**：读三机构状态与 IMU 姿态，拼 `vision_tx_s` 发出（`application/cmd.c:574-584`）；
2. **取输入**：`remote_getData()`，失败（掉线）→ `cmd_all_disable()` 直接返回；
3. **定模式**：`cmd_remote_to_mode()` 把拨杆/拨轮翻译成模式字；
4. **总开关**：只取 `0xF000` 优先级位域，为 `CMD_MODE_OFF`（右拨杆拨下）→ 全车失能；
5. **分派**：`CMD_MODE_REMOTE` → `cmd_remoteControl()`；`CMD_MODE_VISION` → `cmd_visionControl()`；`CMD_MODE_KEYBOARD` → 空实现；
6. **限幅落盘**：内部走 `cmd_xxxUpdata()`，写入三份控制量。

### 4.5 硬件资源

| 外设 | 引脚 | 配置 | 用途 |
| --- | --- | --- | --- |
| CAN1 | PD0 / PD1 | **1 Mbps**（Prescaler 6，BS1 3TQ，BS2 3TQ） | 底盘 4× M3508 |
| CAN2 | PB5 / PB6 | **1 Mbps** | 摩擦轮 2× M3508 + 拨盘 M2006 + **云台 3× DM-J4310（V2.1.1 起）** |
| USB OTG FS | PA11 / PA12 | Device Only，CDC 虚拟串口 | 视觉上位机通信（`SOF = 0x11`） |
| SPI1 | PB3(SCK) / PB4(MISO) / PA7(MOSI) | Full-Duplex Master | BMI088（CS：PA4 加速度计、PB0 陀螺仪） |
| TIM10 CH1 | PF6 | PWM，驱动加热电阻 | IMU 恒温（占空比 0~1） |
| USART3 | PC10(TX) / PC11(RX) | 异步 | DR16 接收机（DBUS） |
| USART6 | PG14(TX) / PG9(RX) | 异步 | 调试串口 115200 |

### 4.6 CAN 报文与 ID 分配

| 总线 | 设备 | 反馈 ID | 发送帧 | 备注 |
| --- | --- | --- | --- | --- |
| CAN1 | M3508 底盘 ×4 | `0x201`~`0x204` | `0x200`（ID 1-4） | `CHASSIS_UL=0 / DL=1 / DR=2 / UR=3` |
| CAN2 | DM-J4310 yaw | `0x11` | `0x01` | V2.1.1 起独立 ID；**当前不发使能帧**（见 §3.13） |
| CAN2 | DM-J4310 big pitch | `0x12` | `0x02` | 目标恒 86° |
| CAN2 | DM-J4310 small pitch | `0x13` | `0x03` | 位置反馈取 `IMU.filter_roll` |
| CAN2 | M3508 摩擦轮 左 / 右 | `0x203` / `0x204` | `0x1FF` | 目标转速互为相反数 |
| CAN2 | M2006 拨盘 | `0x202` | `0x1FF` | `dial_speed × 60/9` rpm |
| CAN2 | GM6020 小 yaw（**仅代码引用，未注册**） | `0x205` | `0x1FF` | ⚠️ 见 §6.1 |

---

## 5. 关键设计与算法

### 5.1 底盘逆运动学

全向轮与麦轮共用一套公式（`application/chassis.c:32-63`），输出为**各轮目标转速 rpm**：

```c
// 1) 场地坐标系 → 车体坐标系（按底盘当前朝向角旋转）
forward = cos(angle)·forward − sin(angle)·turn
turn    = sin(angle)·forward + cos(angle)·turn

// 2) 线速度 → 轮转速（√2 为 45° 轮组投影系数）
vector_f = √2·forward · 60 / (R_tyre/1000) / 2π
vector_t = √2·turn    · 60 / (R_tyre/1000) / 2π
vector_r = rotate · (R_chassis/1000) · 60 / (R_tyre/1000) / 2π

// 3) 四轮合成
UL = +f + t + r      DL = +f − t + r
DR = −f − t + r      UR = −f + t + r
```

**为什么先旋转**：`angle` 由 `cmd` 给出（遥控模式恒为 0），旋转的目的是让操作手的推杆方向**始终对应场地坐标系**，而不是随车头转动。小陀螺模式下底盘持续自转，若不做这层变换，推杆会变成"跟着车转"。

**麦轮为何同式**：45° 麦轮与 45° 全向轮在运动学上等价（都是把轮面速度投影到 45° 方向），差异在摩擦特性与承载，不在解算。这也是 `38fcde4` 提交能把麦轮直接接上的原因。

**舵轮**（`chassis_type_steering`）需要每轮独立的转向角解算，`chassis_soulution()` 里仍是 `//todo`（`application/chassis.c:57-61`）。

### 5.2 底盘跟随

`cmd_chassisFollowMode()`（`application/cmd.c:210`）把「云台相对底盘的偏角」闭环成「底盘自转速度」：

```c
error = small_yaw_init − yaw_motor.angle          // 相对机械中位的偏差
if (|error| ≤ 3.0°) return 0                       // 死区，抑制静止抖动
while (error >  180°) error −= 360°                // 取最短路径
while (error < −180°) error += 360°
output = PID(target = angle + error, measure = angle)
```

**绕的那一圈**：直接把 `error` 当 PID 误差会让编码器的圈数差异变成整圈误差；这里把 `error` 归一化到 ±180° 后再**加回测量值当目标**，等价于 `PID(error)`，但复用了 `pid_calculate(target, measure)` 的签名。

两套 PID 参数按模式切换（`application/cmd.c:495-518`）：

| 用途 | kp | ki | max_i | maxout | 说明 |
| --- | --- | --- | --- | --- | --- |
| 底盘跟随 | 0.1 | 0 | 1.0 | 10.0 | 1° 偏差约 0.03 rad/s |
| 小陀螺跟随 | 0.1 | 2e-6 | 0.0087 | 1.0 | 1° 偏差约 0.002 rad/s，输出限 1 °/s |

小陀螺跟随的输出限幅刻意压到 1 °/s —— 小陀螺本身在高速旋转，跟随环只做缓慢修正，不能抢主转速。

> ⚠️ 这套跟随环目前**不会被调用**：`s2` 拨上进入 `CMD_MODE_TOP` 时 `0x000F` 位段仍为 0，`cmd_remoteControl()` 的底盘分支会走 `CMD_MODE_OFF` 把 `workFlag` 清 0（详见 §6.1#1）。V2.1.1 改了掩码（§3.13 五），但 `s1` 是整字赋值、底盘位域每拍都被重置，路径依旧不可达。

### 5.3 云台三轴

位置环全部下沉到 DM 电机的 MIT 模式，`gimbal_work()` 只做**目标刷新与反馈接管**（`application/gimbal.c:159`）：

```c
// 反馈接管：位置用 IMU，速度用编码器
DM_motor_setFeedbcak(yaw,         IMU.filter_yaw  * DEG2RAD, yaw_data.speed);
DM_motor_setFeedbcak(small_pitch, IMU.filter_roll * DEG2RAD, smallPitch_data.speed);

// 目标下发（rad）
DM_Motor_setPidConfig(yaw,         cmd.small_yaw  * DEG2RAD, 0);
DM_Motor_setPidConfig(big_pitch,   86.0f          * DEG2RAD, 0);
DM_Motor_setPidConfig(small_pitch, -cmd.pitch     * DEG2RAD, 0);
```

**为什么位置用 IMU、速度用编码器**：IMU 姿态角无累积间隙、无噪声尖峰，适合做位置；但陀螺仪积分出的角速度精度不如编码器差分，且 IMU 只有 1 kHz 而云台环是 2 ms，速度环用编码器更稳。

**small pitch 为什么读 roll**：V2.1.1 实车标定后确认，该轴的转动反映在 IMU 的 `roll` 上（IMU 安装方向与机体坐标不重合），而不是 `pitch`。这是「安装 ↔ 代码」的对应关系，不是算法选择 —— 改 IMU 安装方向必须同步改这里。

**big pitch 恒 86°**：哨兵的双 pitch 结构中，大 pitch 只承担"抬头"的固定姿态，实际瞄准由 small pitch 完成。90° 是纸面值，实车会顶到结构限位，故 V2.1.1 收到 **86°**。

**yaw 暂不使能**：`gimbal_enable()` 对 yaw 发的是失能帧（`application/gimbal.c:139`），目标值照发但电机不响应。这是 pitch 单轴验证期的隔离手段，不是最终形态。

**失能时序**（`application/gimbal.c:176-206`）：`workFlag == 0` 时不清零就直接断电，而是先让各轴回 0，**逐轴判定**是否进入 ±0.5°（`GIMBAL_DISABLE_RANGE` 按 deg 定义，比较前乘 `GIMBAL_DEG_TO_RAD`）：

```
small_pitch 用 IMU roll 闭环回 0
  ├─ yaw 超出 ±0.5°      → 反馈切回编码器，目标 0
  ├─ big pitch 超出 ±0.5° → 目标 0
  └─ 全部在范围内        → gimbal_disable()
```

写成 `else if` 链而非并行判断，意味着**每周期只推进一个轴**，回位是串行的。⚠️ yaw 不再使能后，第一级判定不会自行满足，`gimbal_disable()` 可能永远走不到（§6.1#12）。

### 5.4 发射机构

状态机由 `cmd_shootCMD_s::workFlag` 驱动（`application/shoot.c:106`）：

| workFlag | 含义 | 摩擦轮 | 拨盘 |
| --- | --- | --- | --- |
| 0 | 失能 | 先降速到 < 1 rpm 再断电 | 立即断电 |
| 1 | 仅使能 | 0 | 0 |
| 2 | 摩擦轮 | ±`fric_speed`（80 rpm） | 0 |
| 3 | 连发 | **未下发**（继承上一状态） | `dial_speed × 60/9` |
| 4 | 单发 | 0 | 0（`//todo`） |

**9 孔拨盘换算**：`SHOOT_DIAL_HZ_TO_RPM = 60 / 9`，即「每秒发 n 颗」对应「拨盘 n × 60/9 rpm」。这样 `cmd` 里的 `dial_speed = 30` 就直观表示 30 发/秒的意图。

**摩擦轮安全断电**：`SHOOT_FRIC_DISABLE_SPEED = 1.0` rpm —— 只要任一摩擦轮还在 1 rpm 以上，就只给它发 0 目标让它自然减速，**绝不直接 `disable`**。原因是 M3508 带大惯量摩擦轮，高速断电会反电动势冲击驱动、并让轮组自由旋转很久。

### 5.5 IMU 姿态链路

```
BMI088 ──SPI1──▶ IMU_work() (1 ms)
                   ├─ 原始解算：roll/pitch 由 atan2 加速度得出，yaw 由 gyro_z 积分
                   ├─ 滤波：Mahony（默认）或 三轴卡尔曼（二选一）
                   ├─ 零点：校准完成瞬间记 yaw_zero，之后 filter_yaw 从 0 起算
                   ├─ 修正：filter_yaw += yaw_offset（外部经 IMU_setYawOffset() 写入）
                   └─ 温控：每约 100 ms 用 PID 调加热占空比
```

**滤波参数（卡尔曼，`RTOSTask/robotTask.c:33-49`）**

| 轴 | 观测 | R | Q | 备注 |
| --- | --- | --- | --- | --- |
| roll | 加速度角 + 陀螺角速度 | `[0.0330, 0; 0, 0.0078]` | `[3.33e-7, 5e-4; 5e-4, 1.0]` | 双观测 |
| pitch | 同上 | `[0.0350, −2.63e-4; −2.63e-4, 0.0132]` | `[3.33e-6, 5e-3; 5e-3, 10.0]` | 双观测 |
| yaw | 仅陀螺角速度 | `[0.0084]` | `[3.33e-6, 5e-3; 5e-3, 10.0]` | 单观测，会漂 |

**Mahony 参数**：`kp = 1.0`（重力修正比例）、`ki = 0`（零偏积分关闭）、`dt = 0.00114`（实测控制周期，不是标称的 0.001）。

**恒温控制**（`RTOSTask/robotTask.c:65-88`）

| 项 | 值 |
| --- | --- |
| 目标温度 | 40 ℃ |
| 加热执行 | TIM10_CH1 → PF6 PWM，占空比 0~1 |
| PID | `kp 0.20 / ki 0.0032 / kd 0 / max_i 0.88 / maxout 0.9`，普通位置式 |
| 计算周期 | 约 100 ms（仅在温度更新时算一次） |
| 失效保护 | 温度超过 `IMU_HEAT_TIMEOUT = 500` 次未更新 → 自动关加热 |

**为什么恒温**：MEMS 陀螺的零偏对温度极敏感，从室温到 40 ℃ 的零偏漂移可达数 °/s。先把芯片加热到固定温度再标定，等于**把零偏变成一个只随批次变化的常数**，可以从根本上压住 yaw 漂移。

**校准状态机**（`modules/IMU.c`）

| 状态 | 条件 | 动作 |
| --- | --- | --- |
| `heating` | 温度 < 40 ± 1 ℃ | 继续加热；超时 600 000 次则降级 |
| `settling` | 到温 | 等 500 个周期让热应力稳定 |
| `collecting` | 稳定后 | 采 1000 个周期求 gyro 均值当零偏；期间若检测到运动（峰峰值 > 2 °/s 或加速度不在 0.8~1.2 g）就重试 |
| `done` | 采样完成 | 扣除零偏，下一周期记 `yaw_zero` |

`IMU_isCalibrated()` 的口径是「`calib_state == done` **且** `calib_zero_pending == 0`」（`modules/IMU.c:422`），即**yaw 零点也记完了才算完成**，防止云台用未归零的 yaw 使能。

### 5.6 PID 与前馈

`pid_calculate(target, measure, inst)` 计算：

```
error  = target − measure
p_out  = kp × error
i_out  = ki × i_term        （按 max_i 限幅）
d_out  = kd × d_term
f_out  = 外部写入，本函数不动
output = p_out + i_out + d_out + f_out   → 限幅 → 输出滤波 → 死区保持
```

**模式位**（`modules/pid.h:23-38`，Keil AC5 不支持 `0b` 字面量，故用 `1 << n`）：

| 位 | 模式 | 作用 |
| --- | --- | --- |
| bit0 | `pid_mode_normal` | 普通：基础积分 + 误差微分；置位会屏蔽 bit1~bit5 |
| bit1 | `pid_mode_i_separation` | 积分分离：`\|error\| > epsilon` 时清零 `i_term` |
| bit2 | `pid_mode_i_variableSpeed` | 变速积分：A/B 双阈值，误差大时衰减积分 |
| bit3 | `pid_mode_i_trapezoidal` | 梯形积分 |
| bit4 | `pid_mode_d_incomplete` | 不完全微分（微分低通） |
| bit5 | `pid_mode_d_onMeasurement` | 微分先行（对测量值微分） |
| bit6 | `pid_mode_output_filter` | 输出低通，可与其他位叠加 |

**两条必须记住的约定**（`modules/pid.h:5`、`:53`）：

1. `kp/ki/kd` **隐含采样周期，未做 dt 归一化** —— 改变调用周期必须重新整定；
2. `max_i` **只限 `i_out`，不限 `i_term` 的累积** —— 长时间饱和后 `i_term` 可能很大，退饱和会有明显滞后。

**前馈为什么拆出去**：`f_kp × target` 这种「比例前馈」只对速度环勉强成立，对位置环、带重力的轴都不对。拆成 `feedback` 模块后，重力补偿 `cos(θ − φ)`、摩擦补偿 `±k_s` 各用各的模型，PID 专心做误差反馈。

### 5.7 通信协议

#### 5.7.1 通用帧格式（`modules/protocol.c`）

```
┌──────┬──────────────┬──────┬──────┬────────────┬───────────┬────────┐
│ SOF  │  data_len    │ seq  │ CRC8 │  cmd_id    │   data    │ CRC16  │
│ 1 B  │    2 B       │ 1 B  │ 1 B  │    2 B     │  ≤ 30 B   │  2 B   │
└──────┴──────────────┴──────┴──────┴────────────┴───────────┴────────┘
   └───────── 5 B 帧头（CRC8 覆盖）────────┘  └── CRC16 覆盖 cmd_id+data ──┘
```

- `SOF` 视觉固定 `0x11`（`modules/vision.c:58`）；
- `data_len` / `cmd_id` 小端；
- 帧总长 = `5 + 2 + data_len + 2`；
- `PROTOCOL_MAX_DATA_LEN = 30`，打包函数**不校验**超长（`modules/protocol.h:47`），调用方自负。

#### 5.7.2 视觉收发内容

| 方向 | 字段 | 字节 | 单位 |
| --- | --- | --- | --- |
| 本板 → 上位机 | `cmd_id`（状态位汇总） | 2 | — |
| | `yaw` | 4 | deg |
| | `pitch` | 4 | deg |
| | `free_data` | 4 | 恒 0 |
| 上位机 → 本板 | `cmd_id` | 2 | — |
| | `forward` / `turn` / `rotate` | 12 | m/s、m/s、rad/s |
| | `yaw` / `pitch` | 8 | deg |

**回传的 `cmd_id`** 由三机构状态拼成（`application/cmd.c:578-583`）：

```
cmd_id = chassisStatus | (gimbalStatus << 4) | (shootStatus << 8)
```

位置与 `vision_cmd_e` 的位域**恰好对应**，上位机可以用同一套位定义读状态。

#### 5.7.3 控制位定义（`application/cmd.h:72-91` + `modules/vision.h:20-32`）

| 位段 | 值 | 含义 |
| --- | --- | --- |
| `0xF000` | `0x0000` | 关闭（全失能） |
| | `0x1000` | 遥控 |
| | `0x2000` | 键鼠（未实现） |
| | `0x3000` | 视觉 |
| `0x000F` | `0x1` | 底盘跟随 |
| | `0x2` | 底盘不跟随 |
| `0x00F0` | `0x10` | 云台使能 |
| | `0x20` | 小陀螺 |
| `0x0F00` | `0x100` | 发射机构总开关 |
| | `0x200` | 摩擦轮 |
| | `0x300` | 连发 |
| | `0x400` | 单发 |

> ⚠️ `cmd_mode_e` 与 `vision_cmd_e` 在 `0x000F` 位段上**取值互斥**：`cmd` 里 `CHASSIS_FOLLOW = 0x0001`，而 `vision` 里 `0x0001` 是**不跟随**、`0x0002` 才是跟随。两边直接互换会得到相反的底盘行为，改协议时必须逐位核对（见 §6.1）。

### 5.8 掉线守护（`modules/deamon.c`）

一个极简的软件看门狗：注册模块 → 收到数据喂狗 → 周期任务检查超时 → 调回调。

| 模块 | ID 基址 | 实例 | 超时阈值 | 实际时间（10 ms/次） |
| --- | --- | --- | --- | --- |
| DJI 电机 | `0x10` | `0x10`~`0x1F` | — | — |
| 遥控器 | `0x20` | `0x20` | `10` | **100 ms** |
| BMI088 | `0x30` | `0x30`~`0x3F` | — | — |
| 视觉 | `0x40` | `0x40` | `100` | **1 s** |

**设计要点**：

- 计时单位 = 一次 `deamonStart()` 调用（10 ms），改任务周期必须同步改阈值；
- 掉线回调**每个周期都会重复触发**，回调必须自己幂等（`modules/deamon.h:40` 有明确警告）；
- `deamon_WDOG()` 未注册的 ID 会以 `-1` 下标越界写实例表（`modules/deamon.h:68` 标注了 `@warning`）——这是已知隐患。

### 5.9 上电时序

```
上电
 └─ HAL / 时钟 / 外设初始化
     └─ MX_FREERTOS_Init()  创建 9 个任务（此时不调度）
         └─ robot_init()
             ├─ remote_init()          UART3 + 守护注册
             ├─ Serial_init()          UART6 调试口
             ├─ BMI_init()             SPI1 + 两路 CS
             ├─ IMU_init()             姿态算法注册 + 加热 PWM（初始占空比 0）
             ├─ vision_init()          USB 回调 + 守护注册
             ├─ cmd_init()             参数装载 + 全车失能
             ├─ chassis_init()         4× M3508 注册
             ├─ gimbal_init()          3× DM 注册
             └─ shoot_init()           摩擦轮 + 拨盘注册，全部失能
                 └─ osKernelStart()
```

**关键点**：所有 `init` 都在**调度器启动前**完成，因此初始化函数里可以直接阻塞、可以忽略并发；一旦 `osKernelStart()` 返回不再回来，任何遗漏的初始化都不会有机会补。

### 5.10 安全设计汇总

| 场景 | 机制 | 位置 |
| --- | --- | --- |
| 遥控掉线 | 100 ms 超时 → `cmd_all_disable()` | `application/cmd.c:588-592` |
| 视觉掉线 | 1 s 超时 → `vision_getData().state = 0` → 全失能 | `application/cmd.c:447-452` |
| 右拨杆拨下 | 模式字 `0xF000 == OFF` → 全失能 | `application/cmd.c:598-603` |
| 云台失能 | 三轴回零位、逐轴判定后才断电 | `application/gimbal.c:181-203` |
| 摩擦轮失能 | 降到 1 rpm 以下才断电 | `application/shoot.c:120-129` |
| 加热失效 | 温度 500 次未更新 → 关加热 | `modules/IMU.c` |
| 底盘解算失败 | 自动 `chassis_disable()` | `application/chassis.c:245-249` |
| 电机使能失败 | 任一失败 → `workStatus = 0` 并返回 error | `application/chassis.c:164-178` |

**设计哲学**：所有异常路径都收敛到「**失能**」，而不是「保持上一拍」。宁可停车，不可乱动。

---

## 6. 缺陷与待办

### 6.1 阻塞项（上机前必须处理）

| # | 问题 | 位置 | 说明 | 状态 |
| --- | --- | --- | --- | --- |
| 1 | **小陀螺模式整车不动** | `application/cmd.c:306-334` | 两处叠加：①`s2` 拨上进入 `CMD_MODE_TOP` 时，`0x000F` 位段仍为 `0`，随后「底盘命令处理」的 `else if == CMD_MODE_OFF` 分支把 `chassisCMD.workFlag` 覆写成 0 —— **底盘直接失能**；②`chassisCMD.top_speed` 全工程**没有消费者**（`chassis.c:240` 只读 `chassis_rotate`），即使不失能，自转量也没人下发。结论：小陀螺模式当前完全不工作 | ❌ 未修复（V2.1.1 只纠正了掩码语义） |
| 2 | **三轴 DM 电机 ID 冲突** | `application/gimbal.c:27,62,97` | yaw / big pitch / small pitch 曾全部配成 `ID = 0x05`、`MST_ID = 0x01`。`DM_motor_init()` 按数组下标追加实例（三台都在线），但 `DM_motor_pairInstance()` **按 ID 线性查找、返回首个匹配**，于是所有 `setPidConfig` / `setFeedbcak` / `getData(0x05)` 都只作用于第一个实例；同时 `DM_Motor_pidWork()` 会把 3 帧都发往 CAN ID `0x05`。实际效果是「一台电机 + 两份永远收不到目标的占位配置」 | ✅ V2.1.1 已修复（`0x01/0x11`、`0x02/0x12`、`0x03/0x13`，迁至 CAN2） |
| 3 | **跟随环读不到 yaw 编码器** | `application/cmd.c:212` | `cmd_chassisFollowMode()` 读 `{DJI_G6020, 0x01, &hcan2}`，但全工程 `DJI_motor_init()` 只注册了 4 台底盘 + 3 台发射电机，**没有任何地方注册 GM6020**。`DJI_motor_getData()` 必然返回 error 且**返回值未检查**，`yaw_motor.angle` 是未初始化的栈变量，跟随环输出不可预测（未定义行为） | ❌ 未修复 |
| 4 | **yaw 修正量在 IMU 校准前就被锁存** | `application/cmd.c:565-574` | `calib_applied` 静态标志在 `cmdTask` 首次运行时就置 1，而 IMU 恒温校准需要数十秒到数分钟。修正量因此读到的是**上电瞬间**的编码器值；且校准完成时 `yaw_zero` 会把 `filter_yaw` 重新归零，使这次修正失去意义 | ✅ V2.1.1 随门禁恢复一并解决 |
| 5 | **校准门禁被注释掉**（V2.1.0 主动改） | `application/cmd.c:558-562` | `IMU_isCalibrated()` 的判断曾整段注释。V2.0.0 里这段是**启用**的，V2.1.0 为了不让底盘/发射在等 IMU 升温的数十秒里被锁死而注释掉。代价是校准未完成时 `filter_yaw` 尚未归零、陀螺零偏未扣除，**云台位置环此时使能会直接甩头** | ✅ V2.1.1 已恢复（整车等待，不再分级放行） |
| 6 | **连发模式不下发摩擦轮目标** | `application/shoot.c:155-158` | `workFlag == 3` 只设拨盘目标，摩擦轮目标保持上一拍。若从 `workFlag == 1`（全 0）直接进连发，摩擦轮不转却拨盘狂转 —— **卡弹**。V2.1.0 已补上 `workFlag == 2` 时拨盘清零，但连发侧的摩擦轮目标仍未补 | ❌ 未修复 |
| 7 | **DM MIT 帧丢弃 kd** | `modules/DM_motor.c:221` | `((kd_des << 4) & 0x0F)` 恒为 0（左移后低 4 位必为 0），kd 的低 4 位永远丢失。标准写法应为 `((kd_des & 0x0F) << 4) \| ((t_des >> 8) & 0x0F)`。当前云台 `kd = 0` 侥幸未暴露，一旦加阻尼就会失效 | ❌ 未修复 |
| 8 | **协议解包不校验 `data_len`** | `modules/protocol.c:65` | `memcpy(str->data, array+7, data_len)`，`data` 只有 30 字节，而 `data_len` 取自帧头、最大 65535。上位机发一个合法 CRC 的超长帧即可造成**缓冲区越界写** | ❌ 未修复 |
| 9 | **`vision.c` 接收缓冲区越界** | `modules/vision.c:12,17` | `uint8_t a[40]; memcpy(a, data, len);` —— `len` 来自 USB 回调、**未做任何上限检查**，而缓冲区只有 40 字节。且 `a` 此后**从未被使用**，纯属多余，建议直接删除 | ❌ 未修复 |
| 10 | **视觉开火分支不可达** | `application/cmd.c:431` | 该条件要求 `(cmd_mode & 0x0F00) == CMD_MODE_FIRE_AUTO`，但视觉模式下 `cmd_mode = CMD_MODE_VISION = 0x3000`，`0x0F00` 掩码结果为 0，条件**永远不成立**。视觉侧的开火指令实际被忽略 | ❌ 未修复 |
| 11 | **`cmd_mode_e` 与 `vision_cmd_e` 底盘位语义相反** | `application/cmd.h:80-81` vs `modules/vision.h:24-25` | cmd：`0x1`=跟随、`0x2`=不跟随；vision：`0x1`=不跟随、`0x2`=跟随。视觉分支用 `vision_cmd_e` 比较、遥控分支用 `cmd_mode_e`，两套并存，极易误改 | ❌ 未修复 |
| 12 | **云台失能流程依赖已摘掉的 yaw** | `application/gimbal.c:189-199` | V2.1.1 起 yaw 不再使能，`yaw_data.position` 不会回零；而失能判定是串行 `else if` 且**先判 yaw**。只要 yaw 停在 ±0.5° 之外，`gimbal_disable()` 永远走不到，`gimbal_getStatus()` 恒报 1。挂 yaw 前必须把未使能轴从判定里跳过（或改并行判定） | ⚠️ V2.1.1 引入 |

### 6.2 功能待办

| # | 项 | 位置 | 说明 |
| --- | --- | --- | --- |
| 1 | 单发实现 | `application/shoot.c:160-166` | `workFlag == 4` 只把三个目标清零，没有「拨盘转一格（40°）」的动作 |
| 2 | 键鼠控制 | `application/cmd.c:461-464` | `cmd_keyboardControl()` 空函数；哨兵不用键鼠，该位段已被视觉占用 |
| 3 | 舵轮底盘 | `application/chassis.c:57-61` | `chassis_type_steering` 返回 `chassis_error` |
| 4 | 底盘跟随默认关闭 | `application/cmd.h:29` | `CMD_CHASSIS_FOLLOWING` 宏仍是注释状态，默认走「不跟随」分支 |
| 5 | 单发节奏控制 | — | 连发目前是纯速度环，没有"每发间隔"的时间约束，射频全靠拨盘转速决定 |
| 6 | 视觉模式下的发射控制 | `application/cmd.c:390-444` | 已填 `shootCMD`，但同时受 §6.1#6（连发不下发摩擦轮）与 §6.1#10（开火分支不可达）影响，整条路径实际不可用 |

### 6.3 工程与文档

| # | 项 | 位置 | 说明 |
| --- | --- | --- | --- |
| 1 | `filter_type` 注释与实际配置不符 | `modules/IMU.h:34` vs `RTOSTask/robotTask.c:54` | 头文件写「卡尔曼（本工程默认）」，实际 `robot_init()` 配的是 `IMU_filter_mahony` |
| 2 | `yaw_compensate` 注释值过期 | `modules/IMU.h:48` | 注释写 `0.000041f`，实际配置 `0.000009f`（`RTOSTask/robotTask.c:51`） |
| 3 | `shoot_status_e` 枚举值反直觉 | `application/shoot.h:25-29` | `shoot_ok = 0x01`、`shoot_error = 0x00`，与工程内其他模块（`ok = 0`）相反 |
| 4 | `DM_Motor_setPidConfig` / `DM_motor_setPidMode` 大小写不一致 | `modules/DM_motor.h:244,259` | 同一模块内 `DM_Motor_` / `DM_motor_` 混用，容易打错 |
| 5 | `deamon_WDOG()` 未注册 ID 越界写 | `modules/deamon.c` | 已在头文件标 `@warning`，但在中断上下文调用，越界后果严重，建议加边界检查 |
| 6 | 共享结构体无互斥 | `application/cmd.c` | `cmd_work()` 写、三机构读，无临界区；结构体整体一致性无保证 |
| 7 | `serialTask` 中的调试打印仍在跑 | `RTOSTask/robotTask.c:219` | 每 5 ms 打印一次 `"%.6f, %.6f\n"`（V2.1.1 起为 big pitch 位置 / small pitch 转速），占用 UART6 与 CPU，正式版应条件编译 |
| 8 | `bsp/_pylibs/` 混入版本库 | `bsp/_pylibs/` | pip 临时目录不应提交 |
| 9 | `feedback` 模块已实现但从未接线 | `modules/feedback.c` | 全工程无 `feedback_calculate()` 调用点；PID 的 `f_out` 也无人写入，等价于前馈恒为 0 |
| 10 | `bsp_dwt` 是死代码且实现有误 | `bsp/bsp_dwt.c:29-41` | 无任何调用点；`DWT_getData(uint32_t cnt_old)` 按值传参、内部赋值无效，除数用 `SystemCoreClock`（Hz）而非 MHz |
| 11 | `crc8_getSum()` 内有越界调试写 | `modules/crc.c:63,70` | 全局 `uint8_t debug[4]`，每字节写 `debug[len]`，一旦以 `len > 4` 调用即越界。当前调用均为 `len = 4` 才未触发 |
| 12 | USB 初始化晚于 `vision_init()` | `RTOSTask/robotTask.c:93` vs `Core/Src/freertos.c:206` | `vision` 在 `robot_init()` 注册回调，`MX_USB_DEVICE_Init()` 却在 `defaultTask` 里执行；调度器启动初期 `cmd_work()` 若先发帧，CDC 尚未就绪 |
| 13 | `pid_calculate()` 在 `ki = 0` 时除零 | `modules/pid.c:131-134` | 积分限幅按 `max_i / ki` 计算，`ki = 0` 时得 `±inf`，浮点不报错但**限幅完全失效**。云台三轴位置环与多个速度环正好 `ki = 0` |
| 14 | TIM10 加热 PWM 周期约 16 s | `Core/Src/tim.c:43-45` | `PSC = 42000`、`ARR = 65535`，168 MHz 定时器时钟下计数 4 kHz、周期约 16.4 s。作为热惯量系统的慢 PWM 可接受（避免频繁通断），但与 100 ms 的 PID 更新节拍相差两个数量级，建议确认是否有意为之 |
| 15 | DM 反馈 torque 解析掩码过宽 | `modules/DM_motor.c:168` | `torque = ((rx[4] << 8) \| rx[5]) & 0x7FFF` —— 按 DM 帧格式，`rx[4]` 的低 4 位属于 **kd**，torque 只有 12 位。`& 0x7FFF` 保留了 15 位，会把 kd 混进力矩；且随后按 `/4095.0f`（12 位）归一化，量纲也不自洽。当前力矩反馈只用于显示，未进控制环，故列为工程项而非阻塞项 |
| 16 | DM 写寄存器用错下标 | `modules/DM_motor.c:362-363` | `DM_instance[idx]->ID`（`idx` 是**实例总数**，指向表尾之后）而非 `DM_instance[index_id]->ID`。一旦调用 `DM_Motor_writeParameters()` 就会读越界指针；当前无调用点，属潜在崩溃 |
| 17 | **V2.1.0 的积分钳位在 `ki = 0` 时失效** | `modules/pid.c:131-134` | 新钳位写成 `max_i / ki`，`ki = 0` 时得 `±inf`；若 `max_i` 也为 0 则得 `NaN`，比较恒假、钳位完全不生效。此时 `i_out ≡ 0` 无实际影响，但建议加 `ki > 0` 前置判断，避免以后有人给 `ki = 0` 的环配非零 `max_i` 时误以为限幅有效 |
| 18 | `GIMBAL_DISABLE_RANGE` 注释仍标 rad | `application/gimbal.h:16` | V2.1.1 起该宏按 **deg** 使用（比较前乘 `GIMBAL_DEG_TO_RAD`），宏注释与实现相反，改动阈值前务必先看 §3.13 六 |
| 19 | `gimbal_init()` 注释与 ID 配置不符 | `application/gimbal.c:17` | `@note` 仍写「当前均为 0x05/0x01」，实际已是 `0x01/0x02/0x03` 与 `0x11/0x12/0x13` |
| 20 | `gimbal.h` big pitch 注释过期 | `application/gimbal.h:28` | 仍写「当前固定 π/2」，实际目标为 86° |
| 21 | `serialTask` 取了 `IMU` 却未使用 | `RTOSTask/robotTask.c:218` | `IMU_data_s IMU = IMU_getData();` 之后没有任何引用；且打印的两列是不同物理量，看不出 pitch 跟随误差 |

### 6.4 与旧版说明文档的差异

仓库根目录的 `README.md`（描述前身 `Sentry-MDK` 主线的设计）在以下各处**已与 `Sentry-MDK-27` 不一致**，阅读时需以代码为准：

| README 描述 | `Sentry-MDK-27` 实际 |
| --- | --- |
| `application/motor_config.c` 集中放电机与 PID 参数表 | **已删除**，参数直接写在各 `xxx_init()` 里 |
| `modules/debug.c` 串口在线整定 | **已删除** |
| 达妙 J4340 挂在底盘（小陀螺由 DM 驱动） | 底盘不再含 DM（`chassis.c:123-160` 整段注释），小陀螺的 DM 驱动已移除，但替代路径未接通（见 §6.1#1） |
| `Shoot` 线程 2 ms | 实际 **5 ms**（`RTOSTask/robotTask.c:184`） |
| 上电有校准门禁 | 门禁已恢复（V2.1.1，见 §6.1#5） |
| 云台角度反馈由宏 `CMD_MODE_GIMBAL_FROM_IMU` 切换 | 无该宏，`gimbal_work()` 无条件用 IMU |
| 底盘跟随默认开启 | 默认关闭（`application/cmd.h:29`） |

---

## 7. 参数速查

### 7.1 机械参数（`application/chassis.c:73-77`）

| 参数 | 值 | 单位 |
| --- | --- | --- |
| 底盘半径 `chassis_radius` | 280 | mm |
| 轮毂半径 `tyre_radius` | 78.9 | mm |
| 小陀螺目标转速 `gyroRotate` | 5.0 | rad/s |
| 拨盘孔数 `SHOOT_DAIL_CNT` | 9 | 个 |
| 摩擦轮失能阈值 | 1.0 | rpm |

### 7.2 电机 PID

| 对象 | 环 | kp | ki | kd | max_i | maxout | 周期 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 底盘 M3508 | 速度 | 400 | 5 | 0 | 1000 | 8000 | 5 ms |
| 摩擦轮 M3508 | 速度 | 60 | 0.1 | 0 | 500 | 8000 | 5 ms |
| 拨盘 M2006 | 速度 | 140 | 1.0 | 0 | 3000 | 30000 | 5 ms |
| DM yaw | 速度 | 1.0 | 0 | 0 | 4.0 | 28.0 | 2 ms |
| DM yaw | 位置 | 1.0 | 0 | 0 | 2.0 | 100.0 | 2 ms |
| DM big pitch | 速度 | 1.0 | 0.05 | 0 | 4.0 | 10.0 | 2 ms |
| DM big pitch | 位置 | **6.0** | 0 | 0 | 2.0 | **3.0** | 2 ms |
| DM small pitch | 速度 | 1.0 | 0.01 | 0 | 1.0 | 10.0 | 2 ms |
| DM small pitch | 位置 | **22.0** | 0 | 0 | 2.0 | **9.0** | 2 ms |
| 底盘跟随 | 位置 | 0.1 | 0 | 0 | 1.0 | 10.0 | 4 ms |
| 小陀螺跟随 | 位置 | 0.1 | 2e-6 | 0 | 0.0087 | 1.0 | 4 ms |
| IMU 加热 | 位置 | **0.10** | **0** | 0 | 0.88 | 0.9 | ~100 ms |

> DM 位置环的 `maxout` 单位是 N·m，超过 MIT 帧的 `TMAX` 时 `DM_motor_sendMIT()` 会**整帧拒发**（`modules/DM_motor.c:198-203`），不是限幅。三轴 MIT 帧上限：yaw `PMAX π / VMAX 10 / TMAX 28`，big pitch 与 small pitch `PMAX π / VMAX 30 / TMAX 10`。
>
> V2.1.0 → V2.1.1 的变化：big pitch 位置 `kp 1.0 → 6.0`、`maxout 100 → 3.0`；small pitch 位置 `kp 1.0 → 22.0`、`maxout 100 → 9.0`；温控 `kp 0.20 → 0.10`、`ki 0.0032 → 0`。均为上车试凑值，尚无辨识数据支撑。

### 7.3 `cmd` 限幅与阈值（`application/cmd.c:475-495`）

| 项 | 值 | 单位 |
| --- | --- | --- |
| 前进最大速度 `forward_speed` | 1.0 | m/s |
| 横移最大速度 `turn_speed` | 1.0 | m/s |
| 底盘自转最大 `chassis_rotate_speed` | 5.0 | rad/s |
| 小陀螺最大 `chassis_top_speed` | 5.0 | rad/s |
| 云台旋转速度 `gimbal_rotate_speed` | 1.0 | 单位/拍 |
| pitch 速度 `pitch_speed` | 1.0 | 单位/拍 |
| yaw 角度范围 | 30° ~ 210°（中心 120° ± 90°） | deg |
| pitch 角度范围 | **−60° ~ 60°**（中心 0°，V2.1.1 起） | deg |
| 摩擦轮目标转速 `fric_speed` | 80 | rpm |
| 拨盘发弹频率 `dial_speed` | 30 | 发/秒 |
| 摩擦轮触发拨轮量 | ≥ 100 | — |
| 开火触发拨轮量 | ≥ 600 | — |
| 遥控死区 | 50 | — |
| 小 yaw 机械中位 `small_yaw_init` | 0 | deg |
| pitch 机械中位 `pitch_init` | 0 | deg |

---

## 8. 里程碑与下一步

### 8.1 已达成

- ✅ 底盘全向/麦轮逆解 + 四轮速度闭环（V1.1.0 实装，**V2.1.0 上车验证**）
- ✅ 遥控 + 视觉双控制源，统一的模式位域
- ✅ IMU 恒温校准 + Mahony/卡尔曼双算法可切换
- ✅ 视觉通信链路（USB CDC + 通用帧 + 断线守护）
- ✅ 发射机构（摩擦轮 + 拨盘）实装（**V2.1.0 上车验证**）
- ✅ 云台 **pitch 双轴**（big pitch 固定 86° + small pitch 瞄准）接入 IMU 反馈闭环（**V2.1.1 上车验证**）
- ✅ 云台三轴 DM 电机独立 ID、迁至 CAN2、MIT 帧参数按实测重设（V2.1.1）
- ✅ IMU 校准门禁恢复，yaw 修正量不再取上电瞬间值（V2.1.1）
- ✅ PID 模式位化、前馈独立成 `feedback` 模块
- ✅ 观测通道按验证对象切换：底盘双列 CSV（V2.1.0）→ 云台双列 CSV（V2.1.1）

### 8.2 下一阶段（建议顺序）

1. **云台 yaw 验证** —— 当前最大的未验证项。上机前必须先修 §6.1#12（失能流程卡在已摘除的 yaw）与 #3（跟随环读未注册电机），再给 yaw 做方向与零点标定；
2. **小陀螺模式修复** —— §6.1#1，需同时解决「底盘被误失能」与「`top_speed` 无消费者」两个问题；
3. **pitch 整定收尾** —— big pitch `kp = 6`、small pitch `kp = 22` 是上车试凑值，建议按 `.analysis_yaw/` 的方法补阶跃 / 频响记录后再定稿；
4. **把前馈接上** —— `feedback` 模块与 `f_out` 通路都空着（§6.3#9），pitch 的重力补偿是目前最明显的性能缺口；
5. **发射机构单发与射频控制** —— 补 `workFlag == 4`，修连发不下发摩擦轮（§6.1#6），并给连发加最小间隔；
6. **加固通信层** —— 补 `data_len` 校验（§6.1#8）与删除 `vision.c` 的冗余 40 字节缓冲（§6.1#9），这两处直接对上位机的输入做无界 `memcpy`；
7. **提交 V2.1.1** —— 当前改动仍在工作区，建议先提交再开 yaw 验证，避免两批整定混在同一份 diff 里；
8. **同步更新根目录 `README.md`** —— 按 §6.4 的差异表逐条修正，或直接用本文替换其中的架构章节；
9. **补齐飞书文档对应内容** —— 见文首说明。

---

## 附录 A · 提交记录

```
（未提交）  2026-11-19  V2.1.1 云台 pitch 双轴上车验证；修复云台映射 / 使能 / 掩码缺陷
1343ab3  2026-11-16  V2.1.0 底盘与发射机构上车验证，修复若干缺陷
fb3960b  2026-11-09  V2.0.0 新增云台及发射机构，待验证
3853388  2026-11-02  V1.1.0新增底盘
4096da6  2026-10-31  V2.0版本，重写cmd
2e5c330  2026-10-23  发射机构实装与前馈模块化，DJI 电机接口带 CAN 口
1e674ef  2026-10-21  IMU 恒温校准与 Mahony 融合，DJI 电机型号/CAN 口解耦，新增 PWM 与发射机构骨架
6228c3a  2026-10-20  26.10.20更新
7859a56  2026-10-14  重构与视觉通信协议
d6b4e5b  2026-10-09  完善所有 .h 文件注释
4a6a056  2026-10-09  加入与视觉通信协议
7068f3f  2026-10-07  加入 usb 驱动
0b9ae3f  2026-10-06  10/06更新
a510607  2026-10-06  IMU 卡尔曼滤波，增加宏定义切换：变速小陀螺、云台角度反馈来源
12c31b4  2026-10-05  加入矩阵运算及卡尔曼滤波器
c87a1c7  2026-10-01  更新
2fe830e  2026-10-01  更改 cmd 代码：底盘跟随模式开启功能
d9ef3f3  2026-09-27  README
3670885  2026-09-27  暑假短学期版本-底盘及小陀螺-无云台
38fcde4  2026-09-27  麦轮底盘电控代码（解算同全向轮底盘）
f54c2d8  2026-09-27  底盘跟随v1
```

> 2026-11-16 仓库重建，当前 git 历史只保留 `1343ab3` 一个根提交；`1343ab3` 之前的哈希在本仓库内已不可达，上表按迁移前的记录整理，仅作版本追溯之用。

## 附录 B · 源码清单

| 模块 | 文件 | 行数 | 职责 |
| --- | --- | --- | --- |
| 指令 | [cmd.c](application/cmd.c) / [cmd.h](application/cmd.h) | 671 / 122 | 模式解析、限幅、状态汇总 |
| 底盘 | [chassis.c](application/chassis.c) / [chassis.h](application/chassis.h) | 276 / 76 | 逆解 + 四轮速度环 |
| 云台 | [gimbal.c](application/gimbal.c) / [gimbal.h](application/gimbal.h) | 236 / 57 | 三轴位置环 + IMU 反馈接管 |
| 发射 | [shoot.c](application/shoot.c) / [shoot.h](application/shoot.h) | 178 / 51 | 摩擦轮 + 拨盘 |
| DJI 电机 | [DJI_motor.c](modules/DJI_motor.c) / [DJI_motor.h](modules/DJI_motor.h) | 455 / 145 | M2006/M3508/GM6020 双环 |
| DM 电机 | [DM_motor.c](modules/DM_motor.c) / [DM_motor.h](modules/DM_motor.h) | 588 / 276 | MIT/PVT/闭环 + 50 项寄存器读写 |
| PID | [pid.c](modules/pid.c) / [pid.h](modules/pid.h) | 159 / 130 | 7 位模式的可裁剪 PID |
| 前馈 | [feedback.c](modules/feedback.c) / [feedback.h](modules/feedback.h) | 107 / 105 | 速度/加速度/摩擦/重力补偿 |
| 姿态 | [IMU.c](modules/IMU.c) / [IMU.h](modules/IMU.h) | 441 / 159 | 解算 + 滤波 + 温控 + 校准 |
| 传感器 | [BMI088.c](modules/BMI088.c) | 225 | 6 轴 IMU SPI 驱动 |
| 滤波 | [kalman_filter.c](modules/kalman_filter.c) / [mahony.c](modules/mahony.c) / [matrix.c](modules/matrix.c) | 187 / 176 / 315 | 三种姿态算法实现 |
| 通信 | [protocol.c](modules/protocol.c) / [vision.c](modules/vision.c) / [crc.c](modules/crc.c) | 71 / 119 / 116 | 通用帧 + 视觉 + CRC8/16 |
| 输入 | [remote.c](modules/remote.c) / [deamon.c](modules/deamon.c) | 124 / 87 | DBUS 解析 + 掉线守护 |
| 底层 | [bsp_can.c](bsp/bsp_can.c) / [bsp_spi.c](bsp/bsp_spi.c) / [bsp_usb.c](bsp/bsp_usb.c) / [bsp_pwm.c](bsp/bsp_pwm.c) / [bsp_dwt.c](bsp/bsp_dwt.c) / [serial.c](bsp/serial.c) | 3171 / 4481 / 1134 / 1879 / 852 / 4111 B | HAL 薄封装 |
| 任务 | [robotTask.c](RTOSTask/robotTask.c) | 225 | 初始化 + 9 个任务体 |

---

*最后更新：2026-11-19 · 对应 **V2.1.1**（工作区，未提交；基线提交 `1343ab3` 为 V2.1.0）*
