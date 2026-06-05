#include "app.h"
#include "alert.h"
#include "chassis.h"
#include "main.h"
#include "motor.h"
#include "openmv.h"
#include "sensor.h"

// 定义系统的所有状态
typedef enum
{
    STATE_WAIT_START,
    STATE_TRACKING_OUT,
    STATE_APPROACH_TARGET,
    STATE_TASK_ALERT,
    STATE_RETURN_LINE,
    STATE_TRACKING_BACK,
    STATE_PARKING,
    STATE_STOPPED
} CarState_t;

static CarState_t car_state = STATE_WAIT_START;
static uint32_t state_timer = 0;
// 🌟 新增：返程计时器（时间护盾）
static uint32_t return_journey_start_time = 0;

extern uint8_t enable_line_tracking;

void App_Init(void)
{
    car_state = STATE_WAIT_START;
    state_timer = HAL_GetTick();
}

void App_Run(void)
{
    uint8_t alert_done = Alert_Process();

    switch (car_state)
    {
    case STATE_WAIT_START:
        // 🚨 核心修复 1：强行关掉底层自动循迹，防止它覆盖我们的停车指令！
        enable_line_tracking = 0;
        Chassis_SetPhysicalSpeed(0.0f, 0.0f); // 保持原地静止死锁

        // 🌟 扫描启动开关（高电平有效）
        if (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(20); // 软件消抖：延时20ms避开机械按键按下的前沿抖动

            // 再次确认是否真的按下了
            if (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_SET)
            {
                // 【发车前置动作】：让蜂鸣器短鸣一声 “滴~”，提示手可以放开了
                HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

                // 🔒 稳健松手检测：如果手一直按着，程序就卡死在这里，防止你手还没抽回来车就飞了
                while (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_SET)
                    ;

                HAL_Delay(200); // 给你额外的 200ms 反应时间把手彻底抽回

                // 重置所有核心时间戳，防止定时器溢出误判
                state_timer = HAL_GetTick();
                return_journey_start_time = HAL_GetTick();

                // 🚀 正式发车！
                enable_line_tracking = 1; // 🚨 核心修复 2：解除封印，恢复底层自动循迹！

                // 提示：你目前在测试，可以直接跳到 STATE_TRACKING_BACK
                car_state = STATE_TRACKING_BACK;
            }
        }
        break;

    case STATE_TRACKING_OUT:

        if (openmv_found_target == 1)
        {
            car_state = STATE_APPROACH_TARGET;
        }

        break;

    case STATE_APPROACH_TARGET:
        enable_line_tracking = 0;
        float dist = Hcsr04GetLength();
        if (dist > 0.0f && dist < 0.4f)
        {
            Chassis_SetPhysicalSpeed(0.0f, 0.0f);
            Alert_Start(ALERT_TARGET_FOUND);
            car_state = STATE_TASK_ALERT;
        }
        else
        {
            float mv_turn = openmv_x_error * 0.005f;
            Chassis_SetPhysicalSpeed(0.3f + mv_turn, 0.3f - mv_turn);
        }
        break;

    case STATE_TASK_ALERT:
        Chassis_SetPhysicalSpeed(0.0f, 0.0f);
        if (alert_done == 1)
        {
            state_timer = HAL_GetTick();
            car_state = STATE_RETURN_LINE;
        }
        break;

    case STATE_RETURN_LINE:
        Chassis_SetPhysicalSpeed(-0.3f, -0.3f);
        if ((HAL_GetTick() - state_timer > 1000) && (Get_XunJi_State() != 0x00))
        {
            // 🌟 记录正式开始返程的时刻，开启“时间护盾”！
            return_journey_start_time = HAL_GetTick();
            car_state = STATE_TRACKING_BACK;
        }
        break;

    case STATE_TRACKING_BACK:
        // 🌟 核心改变：始终保持循迹开启！让底层 LUT 去对付急转弯！
        enable_line_tracking = 1;

        // 护盾时间：确保完全离开绿柱子区域后才开始检测车库
        if (HAL_GetTick() - return_journey_start_time > 2500)
        {
            static uint8_t garage_armed = 0; // 车库陷阱触发器
            static uint32_t arm_time = 0; // 陷阱开启时间
            uint8_t current_state = Get_XunJi_State();

            if (garage_armed == 0)
            {
                // 如果扫到全黑 (或接近全黑)，说明“疑似”遇到车库或十字路口或急转弯
                if (current_state == 0x1F || current_state == 0x0E || current_state == 0x1B)
                {
                    garage_armed = 1; // 开启车库陷阱
                    arm_time = HAL_GetTick();
                }
            }
            else // 车库陷阱已开启！此时 LUT 仍在默默工作帮你纠偏
            {
                // 1. 如果在极短时间内 (300ms 内) 彻底脱离黑线，变成全白
                // 这说明线物理上断了，100% 是车库！
                if (current_state == 0x00)
                {
                    enable_line_tracking = 0; // 确认进车库，正式切断循迹
                    state_timer = HAL_GetTick(); // 给下一步拖后轮计时用
                    car_state = STATE_PARKING;
                    garage_armed = 0; // 重置陷阱
                }
                // 2. 如果超过 50ms 还没变成全白，说明只是急转弯或者十字路口
                else if (HAL_GetTick() - arm_time > 50)
                {
                    garage_armed = 0; // 虚惊一场，自动解除陷阱，继续正常跑
                }
            }
        }
        break;

    case STATE_PARKING:
    {
        static uint8_t parking_step = 0;
        static uint8_t parking_alert_started = 0;

        if (parking_step == 0)
        {
            // 此时车头已经进入纯白车库，盲开 650ms 把后轮拖进来
            // 💡 如果你觉得停得太深，就把 650 改小 (比如 500)
            if (HAL_GetTick() - state_timer < 500)
            {
                Chassis_SetPhysicalSpeed(0.35f, 0.35f);
            }
            else
            {
                // 核心主动刹车，瞬间拉死克服惯性
                Set_Motor_Output('A', -400);
                Set_Motor_Output('B', -400);
                Set_Motor_Output('C', -400);
                Set_Motor_Output('D', -400);
                HAL_Delay(50);

                parking_step = 1;
            }
        }
        else if (parking_step == 1)
        {
            // 彻底死锁
            Chassis_SetPhysicalSpeed(0.0f, 0.0f);
            Set_Motor_Output('A', 0);
            Set_Motor_Output('B', 0);
            Set_Motor_Output('C', 0);
            Set_Motor_Output('D', 0);

            if (parking_alert_started == 0)
            {
                Alert_Start(ALERT_PARKING);
                parking_alert_started = 1;
            }
            else if (alert_done == 1)
            {
                car_state = STATE_STOPPED;
            }
        }
        break;
    }
    case STATE_STOPPED:
        Chassis_SetPhysicalSpeed(0.0f, 0.0f);
        Set_Motor_Output('A', 0);
        Set_Motor_Output('B', 0);
        Set_Motor_Output('C', 0);
        Set_Motor_Output('D', 0);
        Chassis_Stop_And_Reset(); // 🌟 比赛结束，彻底断电重置
        break;
    }
}