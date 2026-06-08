#include "chassis.h"
#include "app.h"
#include "config.h"
#include "debug.h"
#include "motor.h"
#include "pid.h"
#include "sensor.h"
#include "usart.h"

PID_TypeDef pid_left;
PID_TypeDef pid_right;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// 底盘运动状态
static float target_physical_L = 0.0f;
static float target_physical_R = 0.0f;

// 滤波器历史数据
static float filter_speed_L = 0.0f;
static float filter_speed_R = 0.0f;

// 增量式 PID 的“油门蓄水池”
static float current_throttle_L = 0.0f;
static float current_throttle_R = 0.0f;

// 物理常量定义
#define WHEEL_PERIMETER_M 0.21f
#define PULSES_PER_ROUND 1560.0f
static float base_speed = 0.30f;

// 是否启用循迹
uint8_t enable_line_tracking = 1;

#if USE_PD_TRACKING == 1
static float track_last_error = 0.0f;
#else
static uint8_t last_valid_state = 0x04;
uint8_t is_passing_cross = 0;
static uint32_t chassis_cross_timer = 0;
#endif

void Chassis_Init(void)
{
    // 内环增量式 PID：Kp 负责刹车，Ki 负责推力
    PID_Init(&pid_left, LKP, LKI, LKD);
    PID_Init(&pid_right, RKP, RKI, RKD);
}

// 专门用于修改循迹基准速度的函数
void Chassis_SetTrackingBaseSpeed(float speed)
{
    base_speed = speed;
}

// 直接设置物理速度的函数
void Chassis_SetPhysicalSpeed(float speed_L_m_s, float speed_R_m_s)
{
    target_physical_L = speed_L_m_s;
    target_physical_R = speed_R_m_s;
}

//刹车和状态重置
void Chassis_Stop_And_Reset(void)
{
    // 目标速度清零
    target_physical_L = 0.0f;
    target_physical_R = 0.0f;

    // 彻底清空增量式 PID 的“油门蓄水池”
    current_throttle_L = 0.0f;
    current_throttle_R = 0.0f;

    // 清空滤波器历史数据
    filter_speed_L = 0.0f;
    filter_speed_R = 0.0f;

    // 清空 PID 内部偏差记录
    pid_left.Error = 0;
    pid_left.Last_Error = 0;
    pid_left.Prev_Error = 0;
    pid_right.Error = 0;
    pid_right.Last_Error = 0;
    pid_right.Prev_Error = 0;

    // 清空外环 PD 的历史误差
#if USE_PD_TRACKING == 1
    track_last_error = 0.0f;
#endif

    // 强制物理电机断电死锁
    Set_Motor_Output('A', 0);
    Set_Motor_Output('B', 0);
    Set_Motor_Output('C', 0);
    Set_Motor_Output('D', 0);
}

void Chassis_Update(void)
{
    uint8_t xunji_state = Get_XunJi_State();

    if (enable_line_tracking == 0)
    {
        //清理状态但不干预速度
#if USE_PD_TRACKING == 0
        is_passing_cross = 0;
#endif
    }
    else
    {
#if USE_PD_TRACKING == 1 // PD 循迹方案
        float current_error_raw = Get_Line_Error(xunji_state);

        static float filtered_error = 0.0f;
        filtered_error = PD_filter_alpha * current_error_raw + (1.0f - PD_filter_alpha) * filtered_error;

        // 计算原始的 PD 修正量
        float turn_output = track_Kp * filtered_error + track_Kd * (filtered_error - track_last_error);
        track_last_error = filtered_error;

        // 计算并乘上速度权重(根据调车的速度快慢自动放大或缩小转向修正量)
        float speed_weight = base_speed / Debug_Speed;
        turn_output = turn_output * speed_weight;

        // 弯道动态降速
        float abs_error = filtered_error;
        if (abs_error < 0)
            abs_error = -abs_error;

        float dynamic_base_speed = base_speed - Turn_Slowdown_Factor * abs_error;
        if (dynamic_base_speed < Min_Base_Speed)
            dynamic_base_speed = Min_Base_Speed;

        // 差速叠加
        target_physical_L = dynamic_base_speed + turn_output;
        target_physical_R = dynamic_base_speed - turn_output;

#else // LUT 循迹方案
        if (xunji_state == 0x1F)
        {
            is_passing_cross = 1;
            chassis_cross_timer = HAL_GetTick();
        }

        if (is_passing_cross == 1)
        {
            if (HAL_GetTick() - chassis_cross_timer < 200)
            {
                target_physical_L = base_speed;
                target_physical_R = base_speed;
            }
            else
            {
                is_passing_cross = 0;
            }
        }
        else
        {
            float speed_L = base_speed;
            float speed_R = base_speed;

            // 计算速度权重
            float speed_weight = base_speed / Debug_Speed;

            // 动态放大或缩小四档转弯差速
            float dyn_td1 = turn_diff_1 * speed_weight;
            float dyn_td2 = turn_diff_2 * speed_weight;
            float dyn_td3 = turn_diff_3 * speed_weight;
            float dyn_td4 = turn_diff_4 * speed_weight;

            // 根据 4 个误差等级，提前算好 4 个降速后的基准速度
            float dyn_base_1 = base_speed - Turn_Slowdown_Factor * 1.0f;
            if (dyn_base_1 < Min_Base_Speed)
                dyn_base_1 = Min_Base_Speed;
            float dyn_base_2 = base_speed - Turn_Slowdown_Factor * 2.0f;
            if (dyn_base_2 < Min_Base_Speed)
                dyn_base_2 = Min_Base_Speed;
            float dyn_base_3 = base_speed - Turn_Slowdown_Factor * 3.0f;
            if (dyn_base_3 < Min_Base_Speed)
                dyn_base_3 = Min_Base_Speed;
            float dyn_base_4 = base_speed - Turn_Slowdown_Factor * 4.0f;
            if (dyn_base_4 < Min_Base_Speed)
                dyn_base_4 = Min_Base_Speed;

            switch (xunji_state)
            {
            case 0x04: // 居中：全速直行
                speed_L = base_speed;
                speed_R = base_speed;
                last_valid_state = xunji_state;
                break;
            case 0x0C: // 微调：使用 dyn_base_1
                speed_L = dyn_base_1 - dyn_td1;
                speed_R = dyn_base_1 + dyn_td1;
                last_valid_state = xunji_state;
                break;
            case 0x08: // 轻微：使用 dyn_base_2
                speed_L = dyn_base_2 - dyn_td2;
                speed_R = dyn_base_2 + dyn_td2;
                last_valid_state = xunji_state;
                break;
            case 0x18: // 中度：使用 dyn_base_3
                speed_L = dyn_base_3 - dyn_td3;
                speed_R = dyn_base_3 + dyn_td3;
                last_valid_state = xunji_state;
                break;
            case 0x10: // 极度：使用 dyn_base_4
                speed_L = dyn_base_4 - dyn_td4;
                speed_R = dyn_base_4 + dyn_td4;
                last_valid_state = xunji_state;
                break;
            case 0x06:
                speed_L = dyn_base_1 + dyn_td1;
                speed_R = dyn_base_1 - dyn_td1;
                last_valid_state = xunji_state;
                break;
            case 0x02:
                speed_L = dyn_base_2 + dyn_td2;
                speed_R = dyn_base_2 - dyn_td2;
                last_valid_state = xunji_state;
                break;
            case 0x03:
                speed_L = dyn_base_3 + dyn_td3;
                speed_R = dyn_base_3 - dyn_td3;
                last_valid_state = xunji_state;
                break;
            case 0x01:
                speed_L = dyn_base_4 + dyn_td4;
                speed_R = dyn_base_4 - dyn_td4;
                last_valid_state = xunji_state;
                break;
            case 0x1F:
            case 0x0E:
                speed_L = base_speed;
                speed_R = base_speed;
                break;
            case 0x1B:
                if (last_valid_state == 0x10 || last_valid_state == 0x18 || last_valid_state == 0x08)
                {
                    speed_L = dyn_base_4 - dyn_td4;
                    speed_R = dyn_base_4 + dyn_td4;
                }
                else if (last_valid_state == 0x01 || last_valid_state == 0x03 || last_valid_state == 0x02)
                {
                    speed_L = dyn_base_4 + dyn_td4;
                    speed_R = dyn_base_4 - dyn_td4;
                }
                else
                {
                    speed_L = base_speed;
                    speed_R = base_speed;
                }
                break;
            case 0x00:
                // 彻底丢线时的原地方向拉回，也乘上权重以适配高速惯性
                if (last_valid_state == 0x10 || last_valid_state == 0x18 || last_valid_state == 0x08)
                {
                    speed_L = -0.15f * speed_weight;
                    speed_R = 0.35f * speed_weight;
                }
                else if (last_valid_state == 0x01 || last_valid_state == 0x03 || last_valid_state == 0x02)
                {
                    speed_L = 0.35f * speed_weight;
                    speed_R = -0.15f * speed_weight;
                }
                else
                {
                    speed_L = base_speed;
                    speed_R = base_speed;
                }
                break;
            default:
                speed_L = base_speed;
                speed_R = base_speed;
                break;
            }

            target_physical_L = speed_L;
            target_physical_R = speed_R;
        }
#endif
    }

    /**
     * @brief PID 控制
     */

    int16_t current_speed_L_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    current_speed_L_raw = -current_speed_L_raw;

    int16_t current_speed_R_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    filter_speed_L = PID_filter_alpha * (float)current_speed_L_raw + (1.0f - PID_filter_alpha) * filter_speed_L;
    filter_speed_R = PID_filter_alpha * (float)current_speed_R_raw + (1.0f - PID_filter_alpha) * filter_speed_R;

    float target_pulses_L = (target_physical_L * 0.01f) / WHEEL_PERIMETER_M * PULSES_PER_ROUND;
    float target_pulses_R = (target_physical_R * 0.01f) / WHEEL_PERIMETER_M * PULSES_PER_ROUND;

    float out_pwm_L = PID_Locomotive_Calc(&pid_left, target_pulses_L, filter_speed_L);
    float out_pwm_R = PID_Locomotive_Calc(&pid_right, target_pulses_R, filter_speed_R);

    current_throttle_L += out_pwm_L;
    current_throttle_R += out_pwm_R;

    if (current_throttle_L > 950.0f)
        current_throttle_L = 950.0f;
    if (current_throttle_L < -950.0f)
        current_throttle_L = -950.0f;
    if (current_throttle_R > 950.0f)
        current_throttle_R = 950.0f;
    if (current_throttle_R < -950.0f)
        current_throttle_R = -950.0f;

    Set_Motor_Output('C', (int16_t)current_throttle_L);
    Set_Motor_Output('D', (int16_t)current_throttle_L);
    Set_Motor_Output('A', (int16_t)current_throttle_R);
    Set_Motor_Output('B', (int16_t)current_throttle_R);

#if VOFA_DEBUG == 1
    // 发送数据给 VOFA+
    float vofa_data[4];
    vofa_data[0] = target_pulses_L;
    vofa_data[1] = filter_speed_L;
    vofa_data[2] = target_pulses_R;
    vofa_data[3] = filter_speed_R;
    JustFloat_Send(vofa_data, 4);
#endif
}