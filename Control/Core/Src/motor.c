#include "motor.h"
#include "tim.h"
#include "gpio.h"

extern TIM_HandleTypeDef htim4;

/**
 * @brief  电机底层驱动
 * @param  motor_id: 'A'(右前), 'B'(右后), 'C'(左前), 'D'(左后)
 * @param  speed: -99 到 99（正数正转，负数反转）
 */
void Set_Motor_Output(char motor_id, int16_t speed)
{
    // 速度限幅
    if (speed > 999)
        speed = 999;
    if (speed < -999)
        speed = -999;

    uint16_t pwm_val = (speed >= 0) ? speed : (-speed);

    switch (motor_id)
    {
    case 'A': // 右前轮
        if (speed >= 0)
        {
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_val);
        break;

    case 'B': // 右后轮
        if (speed >= 0)
        {
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_val);
        break;

    case 'C': // 左前轮
        if (speed >= 0)
        {
            HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_val);
        break;

    case 'D': // 左后轮
        if (speed >= 0)
        {
            HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_val);
        break;
    }
}

void Car_Go(int16_t speed)
{
    Set_Motor_Output('A', speed);
    Set_Motor_Output('B', speed);
    Set_Motor_Output('C', speed);
    Set_Motor_Output('D', speed);
}

void Car_Stop(void)
{
    Set_Motor_Output('A', 0);
    Set_Motor_Output('B', 0);
    Set_Motor_Output('C', 0);
    Set_Motor_Output('D', 0);
}

void Car_Back(int16_t speed)
{
    Car_Go(-speed);
}

void Car_Left(int16_t speed)
{
    Set_Motor_Output('A', speed);
    Set_Motor_Output('B', speed);
    Set_Motor_Output('C', -speed);
    Set_Motor_Output('D', -speed);
}

void Car_Right(int16_t speed)
{
    Set_Motor_Output('A', -speed);
    Set_Motor_Output('B', -speed);
    Set_Motor_Output('C', speed);
    Set_Motor_Output('D', speed);
}

void Car_Left_Precision(int16_t Lspeed, int16_t Rspeed)
{
    Set_Motor_Output('A', Rspeed);
    Set_Motor_Output('B', Rspeed);
    Set_Motor_Output('C', -Lspeed);
    Set_Motor_Output('D', -Lspeed);
}

void Car_Right_Precision(int16_t Lspeed, int16_t Rspeed)
{
    // 右转：右轮往后退，左轮往前走
    Set_Motor_Output('A', -Rspeed);
    Set_Motor_Output('B', -Rspeed);
    Set_Motor_Output('C', Lspeed);
    Set_Motor_Output('D', Lspeed);
}