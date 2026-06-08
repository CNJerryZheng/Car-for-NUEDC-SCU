/**
 ******************************************************************************
 * @file           : app.c
 * @brief          : 智能小车核心应用逻辑 (状态机)
 ******************************************************************************
 */

#include "app.h"
#include "alert.h"
#include "chassis.h"
#include "config.h"
#include "main.h"
#include "motor.h"
#include "openmv.h"
#include "sensor.h"

typedef enum
{
    STATE_WAIT_START, //等待启动
    STATE_TRACKING_OUT, //出发循迹
    STATE_APPROACH_TARGET, //接近目标
    STATE_TASK_ALERT, //任务完成提示
    STATE_RETURN_LINE, //返程找线
    STATE_TRACKING_BACK, //返程循迹
    STATE_PARKING, //停车
    STATE_STOPPED //完全停止
} CarState_t;

static CarState_t car_state = STATE_WAIT_START;
static uint32_t state_timer = 0;
static uint32_t return_journey_start_time = 0; //返程计时器（时间护盾）

//超声波 50ms 非阻塞旁路(Fail - safe)
static uint32_t last_sonar_time = 0;
static float last_dist = 9.9f;

static uint8_t garage_armed = 0; // 车库陷阱触发器
static uint32_t arm_time = 0; // 陷阱开启时间

static uint8_t parking_step = 0;
static uint8_t parking_alert_started = 0;

extern uint8_t enable_line_tracking;

void App_Init(void)
{
    car_state = STATE_WAIT_START;
    state_timer = HAL_GetTick();
}

void APP_Clear(void)
{
    // 清理视觉残留信号
    extern uint8_t openmv_found_target;
    extern uint8_t openmv_stop_flag;
    openmv_found_target = 0;
    openmv_stop_flag = 0;

    // 清理车库陷阱与停车步骤
    garage_armed = 0;
    parking_step = 0;
    parking_alert_started = 0;

    // 切换状态机回初始状态
    car_state = STATE_WAIT_START;
}

void App_Run(void)
{
    uint8_t alert_done = Alert_Process();

    switch (car_state)
    {
    case STATE_WAIT_START:
        // 强行关掉底层自动循迹，防止覆盖停车指令
        enable_line_tracking = 0;
        Chassis_SetPhysicalSpeed(0.0f, 0.0f); // 保持原地静止死锁

        // 扫描启动开关（高电平有效）
        if (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(20); // 软件消抖：延时20ms避开机械按键按下的前沿抖动

            // 再次确认是否真的按下了
            if (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_SET)
            {
#if ENABLE_STARTUP_BEEP == 1
                // 让蜂鸣器短鸣一声 ，提示手可以放开了
                HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_ON);
                HAL_Delay(100);
                HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_OFF);
#endif

                // 稳健松手检测：如果手一直按着，程序就卡死在这里，直到手彻底松开为止，防止误触发后续状态
                while (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_SET)
                    ;

                HAL_Delay(200); // 额外 200ms 反应时间

                // 重置所有核心时间戳，防止定时器溢出误判
                state_timer = HAL_GetTick();
                return_journey_start_time = HAL_GetTick();

                enable_line_tracking = 1; // 恢复底层自动循迹

#if TASK_MODE == 0
                // 【调试模式】直接应用返程极速，并跳过任务直接回车库
                Chassis_SetTrackingBaseSpeed(SPEED_RETURN);
                car_state = STATE_TRACKING_BACK;
#else
                // 【正常模式】应用出发慢速，去寻找目标
                Chassis_SetTrackingBaseSpeed(SPEED_OUTWARD);
                car_state = STATE_TRACKING_OUT;
#endif
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
        enable_line_tracking = 0; // 停用底层循迹控制

        // 超声波 50ms 非阻塞旁路 (Fail-safe)

        if (HAL_GetTick() - last_sonar_time > 50)
        {
            last_dist = Hcsr04GetLength(); // 每 50ms 更新一次超声波作为兜底防撞
            last_sonar_time = HAL_GetTick();
        }

        // 视觉主导，超声波辅助，双重触发停车
        if (openmv_stop_flag == 1 || (last_dist > 0.0f && last_dist < 0.4f))
        {
            HAL_Delay(80);
            Chassis_SetPhysicalSpeed(0.0f, 0.0f);
            Alert_Start(ALERT_TARGET_FOUND);

            // 清空视觉停车标志，防止误触发后续状态
            openmv_stop_flag = 0;

            car_state = STATE_TASK_ALERT;
        }
        else
        {
            // 视觉 P 参数微调
            float mv_turn = openmv_x_error * vision_adjust_factor;
            Chassis_SetPhysicalSpeed(0.3f + mv_turn, 0.3f - mv_turn);
        }
        break;

    case STATE_TASK_ALERT:
        Chassis_Stop_And_Reset(); // 死锁停车闪灯蜂鸣
        if (alert_done == 1)
        {
            state_timer = HAL_GetTick();
            car_state = STATE_RETURN_LINE;
        }
        break;

    case STATE_RETURN_LINE:

#if RETURN_STRATEGY == 0
        // 【策略 0：原路倒车】
        Chassis_SetPhysicalSpeed(-0.3f, -0.3f);
#else
        // 【策略 1：继续直行】
        Chassis_SetPhysicalSpeed(0.3f, 0.3f);
#endif
        // 500ms 的护盾时间，确保车完全离开绿柱子区域，防止被地上的阴影或绿柱子底座误判为黑线
        if ((HAL_GetTick() - state_timer > 500) && (Get_XunJi_State() != 0x00))
        {
            // 记录正式开始返程的时刻，开启时间护盾
            return_journey_start_time = HAL_GetTick();

            // 核心操作：任务已完成，动态切换为返程极速模式
            Chassis_SetTrackingBaseSpeed(SPEED_RETURN);

            car_state = STATE_TRACKING_BACK;
        }

        break;

    case STATE_TRACKING_BACK:
        // 返程循迹，持续监测车库陷阱
        enable_line_tracking = 1;

        // 确保完全离开绿柱子区域后才开始检测车库
        if (HAL_GetTick() - return_journey_start_time > 2500)
        {
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
            else // 车库陷阱已开启！循迹继续，但密切监视状态变化
            {
                // 如果在极短时间内 (50ms内) 彻底脱离黑线，变成全白
                // 这说明线物理上断了，判断为车库入口，正式切换状态进入停车流程
                if (current_state == 0x00)
                {
                    enable_line_tracking = 0; // 确认进车库，正式切断循迹
                    state_timer = HAL_GetTick(); // 给下一步拖后轮计时用
                    car_state = STATE_PARKING;
                    garage_armed = 0; // 重置陷阱
                }
                // 如果超过 50ms 还没变成全白，说明只是急转弯或者十字路口
                else if (HAL_GetTick() - arm_time > 50)
                {
                    garage_armed = 0; // 自动解除陷阱，继续正常跑
                }
            }
        }
        break;

    case STATE_PARKING:
    {
        if (parking_step == 0)
        {
            //此时车头已经进入车库，盲开 350ms 把后轮拖进来
            if (HAL_GetTick() - state_timer < 350)
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
            Chassis_Stop_And_Reset();

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
        enable_line_tracking = 0;
        Chassis_Stop_And_Reset(); // 比赛结束，彻底断电重置
#if LOOP_COMPLETED_TASK == 1
        // 如果配置允许循环完成任务，停留3秒后自动重置状态机
        if (HAL_GetTick() - state_timer > 3000)
        {
            APP_Clear();
        }
#endif
        break;
    }
}