# 🏎️ 全国大学生电子设计竞赛 - 智能视觉循迹小车

![STM32](https://img.shields.io/badge/STM32-F103-blue.svg) ![OpenMV](https://img.shields.io/badge/OpenMV-MicroPython-green.svg) ![Status](https://img.shields.io/badge/Status-Active-brightgreen.svg) ![License](https://img.shields.io/badge/License-MIT-yellow.svg)

本项目为全国大学生电子设计竞赛（电赛）的智能小车综合解决方案。系统以 STM32 为主控核心，结合 OpenMV 机器视觉与灰度传感器，实现极速巡线、动态目标搜寻、视觉精准逼近与自主入库等完整战术流程。

本项目采用模块化开发，根目录划分为三个核心子系统：`control`（电控）、`vision`（视觉）与 `hardware`（硬件）。

---

## 📂 仓库目录结构

```text
📦 Car-for-NUEDC-SCU
 ┣ 📂 Control           # 🧠 主控核心代码 (STM32 HAL库, C语言)
 ┣ 📂 Hardware          # 🛠️ 硬件原理图与PCB (相关资料)
 ┣ 📂 Vision            # 👁️ 视觉处理代码 (OpenMV, MicroPython)
 ┣ 📜 README.md         # 📖 项目总说明
 ┗ 📜 Requirements.pdf  # 📝 比赛要求
```

---

## 🌟 核心技术亮点

本项目在 **“求稳”** 与 **“竞速”** 之间寻找到了完美的平衡，尤其在控制算法和架构设计上进行了深度优化：

### 1. 🧠 非阻塞式单向状态机 (Non-blocking State Machine)
- 放弃传统的 `HAL_Delay()` 延时编程，采用基于 `HAL_GetTick()` 的全局时间戳管理。
- 引入 **“时间护盾”** 与 **“定时器陷阱”** 机制，在极速（>0.8m/s）状态下实现车库入口和十字路口的异步无损检测。
- 状态切换如行云流水（发车 -> 循迹 -> 视觉接管 -> 停车打卡 -> 返程恢复 -> 入库），逻辑严密不卡顿。

### 2. 🚀 高速自适应巡线算法 (Dynamic Gain Scheduling)
- **动态增益调度**：引入 `speed_weight` 速度权重。在底盘变速时（如从 0.3m/s 提速至 0.8m/s），系统自动等比例放大 PD 修正力度，完美克服高速惯性，实现“一套参数打天下”。
- **动态弯道死区**：依据车速动态调整进弯判断阈值（如高速时放宽阈值至 3.4 以免疫直道画龙，低速时缩紧至 2.0 以确保寻的敏锐）。

### 3. 👁️ 多源融合防撞系统 (Multi-Sensor Fail-safe)
- **视觉主导 + 超声波兜底**：OpenMV 通过面积过滤计算目标距离并下发停车标志位（Stop Flag）；底层超声波采用 50ms 轮询与硬件防超时溢出机制作为二次物理防撞旁路。
- 解决传感器时序粘连问题，双重保险确保撞柱率为 0。

### 4. ⚡ 硬件级抗扰优化 (Hardware-Aware Software Design)
- 针对电机急刹车引起的反电动势与瞬态电压跌落（Voltage Sag），在软件层面对声光打卡任务加入 **200ms 电流错峰避让时序**，彻底解决蜂鸣器低压沙哑与哑火问题。

---

## 🧩 模块详细说明

### 1. Control (电控子系统)
位于 `control/` 目录。
- **环境**：STM32CubeMX + HAL库 + VS Code
- **主频**：TIM1 10ms 中断作为系统“心跳”，驱动底层运动学解算。
- **接口**：通过 `config.h` 实现全局战术宏配置（一键切换比赛模式/测试模式、调试输出、起步与返程速度）。
- **通讯协议**：与 OpenMV 采用高可靠 6 字节自定义帧协议：
  `[0x55 (帧头)] + [目标标志] + [X偏差高8位] + [X偏差低8位] + [停车标志] + [0xAA (帧尾)]`

### 2. Vision (视觉子系统)
位于 `vision/` 目录。
- **环境**：OpenMV IDE
- **功能**：基于 LAB 色彩空间的绿色目标精准分割，过滤环境噪点。计算最大色块中心点与画面中轴的偏差（x_error），并结合像素面积评估物理距离，输出停车接管信号。

### 3. Hardware (硬件子系统)
位于 `hardware/` 目录。
- 包含主控扩展板、降压模块（5V/3.3V 大电流稳压网络）、电机驱动板及相关 3D 打印结构件。
- **传感器组合**：5路数字灰度传感器、HC-SR04 超声波、增量式光电/霍尔编码器。

---

## 🛠️ 快速开始

1. **克隆仓库**：
   ```bash
   git clone https://github.com/CNJerryZheng/Car-for-NUEDC-SCU.git
   ```
2. **电控烧录**：
   - 打开 `control/` 目录下的工程文件。
   - 在 `Inc/config.h` 中设置 `#define TEST_MODE 0` 以及 `#define TASK_MODE 1`。
   - 编译并下载至 STM32F103 主板。
3. **视觉配置**：
   - 将 `vision/` 目录下的 `main.py` 通过 OpenMV IDE 保存至摄像头存储卡内。
   - 根据赛场实际光线，在 IDE 内微调绿色阈值 `green_threshold`。
4. **下地运行**：
   - 上电，等待传感器初始化完毕（开机提示音）。
   - 按下 `START_KEY`，松手后小车鸣笛出发。

---

## 🤝 团队分工

* **电控开发 & 架构设计**: [@CNJerryZheng](https://github.com/CNJerryZheng)
* **硬件设计 & 机械结构**: [@Misybon](https://github.com/Misybon)
* **视觉算法 & 调试**: [@BiuBiu_Miku](https://github.com/BiuBiu_Miku)

---

> **Note**: 本代码仅代表我们在电赛备战与比赛期间的技术沉淀。如果本项目对您的学习或比赛有所启发，欢迎点亮 ⭐ **Star**！如有问题，欢迎提交 Issue 探讨。
