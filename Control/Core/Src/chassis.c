#include "chassis.h"
#include "app.h"
#include "debug.h"
#include "motor.h"
#include "pid.h"
#include "sensor.h" // 用于获取循迹传感器状态
#include "usart.h" // 用于 VOFA+ 调试发送

// ==========================================
// 📍 比赛战术速度调节区 (直接在这里修改，下地测试)
// ==========================================
#define SPEED_OUTWARD 0.30f // 出发去寻找目标的循迹速度 (稍慢，符合任务要求)
#define SPEED_RETURN 0.80f // 任务完成后返程的循迹速度 (极速，抢比赛时间)

// ==========================================
// 🛠️ 核心切换开关：循迹算法宏定义
// 1: 使用全新的外环 PD 算法 (丝滑过弯，消灭画龙，带误差滤波消除颤抖)
// 0: 使用原本的 LUT 查表法 (原汁原味，查表暴力匹配)
// ==========================================
#define USE_PD_TRACKING 1

// ==========================================
// 🔒 私有变量：对外部世界完全隐藏
// ==========================================
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
// ==========================================
// 🌟 方案 A：外环巡线 PD 参数 (下地需微调)
// ==========================================
static float track_Kp = 0.08f; // P：控制拐弯的力度
static float track_Kd = 0.35f; // D：阻尼器，抑制直线画龙和过弯摆头
static float track_last_error = 0.0f;

#else
// ==========================================
// 🐢 方案 B：原本 LUT 查表法的专用参数
// ==========================================
// 完全对齐 PD 方案 (Kp = 0.08，误差等级分别为 1, 2, 3, 4)
static float turn_diff_1 = 0.08f;
static float turn_diff_2 = 0.16f;
static float turn_diff_3 = 0.24f;
static float turn_diff_4 = 0.32f;
static uint8_t last_valid_state = 0x04;
uint8_t is_passing_cross = 0;
static uint32_t chassis_cross_timer = 0;
#endif

void Chassis_Init(void)
{
    // 内环增量式 PID：Kp 负责刹车，Ki 负责推力
    PID_Init(&pid_left, 9.5f, 0.8f, 0.0f);
    PID_Init(&pid_right, 8.5f, 0.8f, 0.0f);
}

// 🌟 新增：专门用于修改循迹基准速度的函数 (更安全，不会与物理盲开速度冲突)
void Chassis_SetTrackingBaseSpeed(float speed)
{
    base_speed = speed;
}

void Chassis_SetPhysicalSpeed(float speed_L_m_s, float speed_R_m_s)
{
    // 删掉原来的判断，让这个函数只负责纯粹的物理盲开速度
    target_physical_L = speed_L_m_s;
    target_physical_R = speed_R_m_s;
}

// ==========================================
// 🛑 终极刹车与状态重置大招
// ==========================================
void Chassis_Stop_And_Reset(void)
{
    // 1. 目标速度清零
    target_physical_L = 0.0f;
    target_physical_R = 0.0f;

    // 2. 彻底清空增量式 PID 的“油门蓄水池” (根治抖动的核心)
    current_throttle_L = 0.0f;
    current_throttle_R = 0.0f;

    // 3. 清空滤波器历史数据
    filter_speed_L = 0.0f;
    filter_speed_R = 0.0f;

    // 4. 清空 PID 内部偏差记录
    pid_left.Error = 0;
    pid_left.Last_Error = 0;
    pid_left.Prev_Error = 0;
    pid_right.Error = 0;
    pid_right.Last_Error = 0;
    pid_right.Prev_Error = 0;

    // 5. 清空外环 PD 的历史误差
#if USE_PD_TRACKING == 1
    track_last_error = 0.0f;
#endif

    // 6. 强制物理电机断电死锁
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
        // 停车或视觉模式，清理状态，不干预速度
#if USE_PD_TRACKING == 0
        is_passing_cross = 0;
#endif
    }
    else
    {
#if USE_PD_TRACKING == 1
        // ====================================================
        // 🚀 方案 A：全新外环 PD 算法 (串级 PID) 带误差平滑与速度权重
        // ====================================================
        float current_error_raw = Get_Line_Error(xunji_state);

        static float filtered_error = 0.0f;
        filtered_error = 0.6f * current_error_raw + 0.4f * filtered_error; // (保留你之前调好的滤波比例)

        // 1. 计算原始的 PD 修正量
        float turn_output = track_Kp * filtered_error + track_Kd * (filtered_error - track_last_error);
        track_last_error = filtered_error;

        // 🌟 2. 新增核心逻辑：计算并乘上速度权重
        // 以 0.50f 为基准。速度为 0.50 时权重为 1.0，完全不改变当前的调车手感！
        float speed_weight = base_speed / 0.50f;
        turn_output = turn_output * speed_weight;

        // 获取绝对误差
        float abs_error = filtered_error;
        if (abs_error < 0)
            abs_error = -abs_error;

        // 🌟🌟 新增：动态计算弯道触发阈值 🌟🌟
        // 高速状态(>=0.7m/s)下放宽到 3.0，允许直道上有较大的画龙冗余
        // 中低速状态下保持 2.0，确保进弯敏锐度
        float curve_threshold = (base_speed >= 0.70f) ? 3.0f : 2.0f;

        // ====================================================
        // 🚨 调试专用：进弯亮灯检测 (非阻塞式)
        // 使用动态阈值 curve_threshold 进行判定
        // ====================================================
        // static uint32_t debug_led_timer = 0;

        // if (abs_error >= curve_threshold)
        // {
        //     // 只要还在弯道内，就一直重置计时器，并保持外部 LED 亮起
        //     HAL_GPIO_WritePin(LED_OUTSIDE_GPIO_Port, LED_OUTSIDE_Pin, GPIO_PIN_SET);
        //     debug_led_timer = HAL_GetTick();
        // }

        // // 当误差小于阈值 (出弯) 后，倒计时 300ms 关闭 LED
        // if (debug_led_timer > 0 && (HAL_GetTick() - debug_led_timer >= 300))
        // {
        //     HAL_GPIO_WritePin(LED_OUTSIDE_GPIO_Port, LED_OUTSIDE_Pin, GPIO_PIN_RESET);
        //     debug_led_timer = 0; // 重置定时器状态
        // }
        // ====================================================

        // 🌟 3. 弯道动态降速 (使用动态死区策略)
        float dynamic_base_speed = base_speed;

        // 只有超过了对应的速度阈值，才触发弯道降速逻辑
        if (abs_error >= curve_threshold)
        {
            // 高速时的降速惩罚系数也相应减弱 (0.12)，防止急刹感；低速保持 0.06
            float drop_coef = (base_speed >= 0.70f) ? 0.14f : 0.06f;
            dynamic_base_speed = base_speed - drop_coef * abs_error;

            // 设定保底速度，防止弯道太急导致车子直接停在弯心
            if (dynamic_base_speed < 0.25f)
                dynamic_base_speed = 0.25f;
        }

        // 4. 差速叠加
        target_physical_L = dynamic_base_speed + turn_output;
        target_physical_R = dynamic_base_speed - turn_output;
#else
        // ====================================================
        // 🐢 方案 B：原本的 LUT 查表法 (已完全对齐 PD 的转向与降速逻辑)
        // ====================================================
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

            // 🌟 1. 计算速度权重 (以 0.50f 为基准)
            float speed_weight = base_speed / 0.50f;

            // 🌟 2. 动态放大或缩小四档转弯差速
            float dyn_td1 = turn_diff_1 * speed_weight;
            float dyn_td2 = turn_diff_2 * speed_weight;
            float dyn_td3 = turn_diff_3 * speed_weight;
            float dyn_td4 = turn_diff_4 * speed_weight;

            // 🌟 3. 新增：引入 PD 的“弯道动态降速”灵魂！
            // 根据 4 个误差等级，提前算好 4 个降速后的基准速度 (保底 0.25f)
            float dyn_base_1 = base_speed - 0.06f * 1.0f;
            if (dyn_base_1 < 0.25f)
                dyn_base_1 = 0.25f;
            float dyn_base_2 = base_speed - 0.06f * 2.0f;
            if (dyn_base_2 < 0.25f)
                dyn_base_2 = 0.25f;
            float dyn_base_3 = base_speed - 0.06f * 3.0f;
            if (dyn_base_3 < 0.25f)
                dyn_base_3 = 0.25f;
            float dyn_base_4 = base_speed - 0.06f * 4.0f;
            if (dyn_base_4 < 0.25f)
                dyn_base_4 = 0.25f;

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

    // ========================================================
    // 🔒 无论哪种循迹方案，下面的内环速度 PID 闭环原封不动！
    // ========================================================
    int16_t current_speed_L_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    current_speed_L_raw = -current_speed_L_raw;

    int16_t current_speed_R_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    filter_speed_L = 0.1f * (float)current_speed_L_raw + 0.9f * filter_speed_L;
    filter_speed_R = 0.1f * (float)current_speed_R_raw + 0.9f * filter_speed_R;

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

    // 发送数据给 VOFA+
    float vofa_data[4];
    vofa_data[0] = target_pulses_L;
    vofa_data[1] = filter_speed_L;
    vofa_data[2] = target_pulses_R;
    vofa_data[3] = filter_speed_R;
    JustFloat_Send(vofa_data, 4);
}