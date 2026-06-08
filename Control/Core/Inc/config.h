#pragma once

#include "main.h"

/**
 * 测试模式选择宏定义 (用于 main.c)
 * 0: 【比赛模式】运行 app.c 的完整状态机
 * 1: 【底盘测试】闭环定速直行测试 (验证速度 PID)
 * 2: 【循迹测试】开启循迹模式 (验证巡线平滑度)
 * 3: 【声光测试】循环播放声光提示
 * 4: 【VOFA测试】通过串口发送超声波和 OpenMV 数据到电脑查看
 */
#define TEST_MODE 0

/**
 * 任务流程切换开关 (用于 app.c)
 * 0: 【调试模式】跳过OpenMV识别,执行返程
 * 1: 【正常模式】完整比赛流程 
 */
#define TASK_MODE 1

/**
 * 超声波系数 (用于 sensor.c)
 */
#define ULTRASONIC_COEFFICIENT 58.0f // 超声波测距的时间-距离转换系数

// 循迹算法选择
// 1: 使用外环 PD 算法
// 0: 使用原本的 LUT 法(已废弃，做参考)
#define USE_PD_TRACKING 1

/**
 * 底盘操控调试 (用于 chassis.c)
 */
//PID调试
#define VOFA_DEBUG 0 // 1:开启VOFA调试输出, 0:关闭
#define LKP 9.5f
#define LKI 0.8f
#define LKD 0.0f
#define RKP 8.5f
#define RKI 0.8f
#define RKD 0.0f

//死区数值
#define deatharea 0.0f

//视觉微调系数
#define vision_adjust_factor 0.002f

// 差速动态放大调车速度
#define Debug_Speed 0.50f

// 弯道动态降速参数以及最低降速限制
#define Turn_Slowdown_Factor 0.06f
#define Min_Base_Speed 0.25f

// 循迹算法调试
#if USE_PD_TRACKING == 1

// PD参数
#define track_Kp 0.08f
#define track_Kd 0.40f

// 滤波器的平滑系数
#define PD_filter_alpha 0.6f
#define PID_filter_alpha 0.1f

#else
// LUT算法，误差等级分别为 1, 2, 3, 4
#define turn_diff_1 0.08f
#define turn_diff_2 0.16f
#define turn_diff_3 0.24f
#define turn_diff_4 0.32f
#endif

// 速度 (单位: m/s)
#define SPEED_OUTWARD 0.30f // 出发去寻找目标的循迹速度
#define SPEED_RETURN 0.85f // 任务完成后返程的循迹速度

// 任务完成后回线策略
// 0: 【倒车回线】打卡后挂倒挡，原路退回黑线
// 1: 【直行回线】打卡后继续直行，往前寻找黑线
#define RETURN_STRATEGY 0

// 是否循环完成任务
// 0: 任务完成后停在车库里不动了
// 1: 任务完成后继续循环重新出发
#define LOOP_COMPLETED_TASK 1

// 蜂鸣器硬件电平与行为配置
#define ENABLE_STARTUP_BEEP 1 // 1:开启开机和发车鸣笛, 0:关闭
#define BEEP_ACTIVE_LEVEL 1 // 1:高电平触发鸣笛, 0:低电平触发鸣笛

// 自动电平映射转换
#if BEEP_ACTIVE_LEVEL == 1
#define BEEP_ON GPIO_PIN_SET
#define BEEP_OFF GPIO_PIN_RESET
#else
#define BEEP_ON GPIO_PIN_RESET
#define BEEP_OFF GPIO_PIN_SET
#endif