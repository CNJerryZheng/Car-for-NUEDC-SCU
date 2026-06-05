#include "pid.h"

/**
 * @brief  PID 参数初始化
 */
void PID_Init(PID_TypeDef *pid, float p, float i, float d)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;

    pid->Error = 0.0f;
    pid->Last_Error = 0.0f;
    pid->Prev_Error = 0.0f;
}

/**
 * @brief  增量式 PID 算法核心计算
 * @param  target:  目标速度（期望值）
 * @param  current: 当前通过编码器测出来的实际速度（反馈值）
 * @retval 占空比增量（增量输出）
 */
float PID_Locomotive_Calc(PID_TypeDef *pid, float target, float current)
{
    float increment = 0.0f;
    pid->Error = target - current;

    float deatharea = 0.0f;
    if (pid->Error > -deatharea && pid->Error < deatharea)
    {
        pid->Error = 0.0f;
    }

    increment = pid->Kp * (pid->Error - pid->Last_Error) +
                pid->Ki * pid->Error +
                pid->Kd * (pid->Error - 2.0f * pid->Last_Error + pid->Prev_Error);

    pid->Prev_Error = pid->Last_Error;
    pid->Last_Error = pid->Error;

    return increment;
}