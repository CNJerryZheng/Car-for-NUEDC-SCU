#include "sensor.h"
#include "gpio.h"
#include "tim.h"

extern TIM_HandleTypeDef htim1;

static void sensor_delay_us(uint32_t us)
{
    uint32_t delay = us * 10;
    while (delay--)
        ;
}

float Hcsr04GetLength(void)
{
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    uint32_t delta_time = 0;

    // 触发超声波
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
    sensor_delay_us(15);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

    // 等待 Echo 变高
    uint32_t timeout = 60000;
    while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET && timeout--)
    {
    }
    if (timeout == 0)
        return 0.0f;

    start_time = __HAL_TIM_GET_COUNTER(&htim1);

    // 等待 Echo 变低 (加入防多次溢出保护)
    // 💡 降低这里的 timeout 循环次数，确保在 10ms (1.7米) 内强制退出，防止 TIM1 溢出超过1次
    timeout = 15000;
    while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET && timeout--)
    {
    }

    // 如果超时退出，说明距离太远，直接返回一个安全的最大值 (或者 0)
    if (timeout == 0)
        return 1.50f; // 默认超过 1.5 米就返回 1.5

    end_time = __HAL_TIM_GET_COUNTER(&htim1);

    // 计算时间差
    if (end_time >= start_time)
    {
        delta_time = end_time - start_time;
    }
    else
    {
        // 🚨 适配新的 10ms 周期，ARR 为 99
        delta_time = (99 + 1 - start_time) + end_time;
    }

    // 之前的设定：1个 tick = 100us
    float microsecond = (float)delta_time * 100.0f;
    return (microsecond / 58.0f) / 100.0f; // 建议最后再除以 100.0f，直接返回 "米(m)"，方便你统一单位
}

uint8_t Get_XunJi_State(void)
{
    uint8_t state = 0;

    uint8_t l2 = (HAL_GPIO_ReadPin(L2_GPIO_Port, L2_Pin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t l1 = (HAL_GPIO_ReadPin(L1_GPIO_Port, L1_Pin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t m = (HAL_GPIO_ReadPin(M_GPIO_Port, M_Pin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t r1 = (HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t r2 = (HAL_GPIO_ReadPin(R2_GPIO_Port, R2_Pin) == GPIO_PIN_SET) ? 1 : 0;

    state |= (l2 << 4);
    state |= (l1 << 3);
    state |= (m << 2);
    state |= (r1 << 1);
    state |= (r2 << 0);

    return state;
}

static float last_line_error = 0.0f;

float Get_Line_Error(uint8_t state)
{
    float error = 0.0f;

    switch (state)
    {
    // ============ 完美居中 ============
    case 0x04:
        error = 0.0f;
        break; // 00100

    // ============ 车体稍微偏右，黑线在左，需要轻微左转 ============
    case 0x0C:
        error = -1.0f;
        break; // 01100 (黑线在 M 和 L1 之间)
    case 0x08:
        error = -2.0f;
        break; // 01000 (黑线在 L1)
    case 0x18:
        error = -3.0f;
        break; // 11000 (黑线在 L1 和 L2 之间)
    case 0x10:
        error = -4.0f;
        break; // 10000 (黑线到了最左边 L2，即将脱轨)

    // ============ 车体稍微偏左，黑线在右，需要轻微右转 ============
    case 0x06:
        error = 1.0f;
        break; // 00110 (黑线在 M 和 R1 之间)
    case 0x02:
        error = 2.0f;
        break; // 00010 (黑线在 R1)
    case 0x03:
        error = 3.0f;
        break; // 00011 (黑线在 R1 和 R2 之间)
    case 0x01:
        error = 4.0f;
        break; // 00001 (黑线到了最右边 R2，即将脱轨)

    // ============ 遇到十字路口或大块黑斑 (全亮) ============
    case 0x1F:
    case 0x0E: // 01110
    case 0x1B: // 11011
        /*error = 0.0f; // 直接保持直行冲过去*/
        error = last_line_error; // 保持直行冲过去
        break;

    // ============ ⚠️ 致命状态：完全丢线 (00000) ============
    case 0x00:
        // 如果丢线前一瞬间黑线在最左边，说明车往右冲出去了，必须死命左转拉回来
        if (last_line_error < 0)
            error = -5.0f;
        // 反之，死命右转
        else if (last_line_error > 0)
            error = 5.0f;
        else
            error = 0.0f;
        break;

    default:
        error = last_line_error; // 遇到无法识别的杂波状态，保持上一刻的判断
        break;
    }

    last_line_error = error;
    return error;
}