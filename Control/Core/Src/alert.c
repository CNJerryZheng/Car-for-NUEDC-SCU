#include "alert.h"

// ==========================================
// 📢 蜂鸣器硬件电平配置 (需与 main.c 和 app.c 保持一致)
// ==========================================
#define BEEP_ACTIVE_LEVEL 1 // 1:高电平触发鸣笛, 0:低电平触发鸣笛

// 自动电平映射转换 (请勿修改)
#if BEEP_ACTIVE_LEVEL == 1
#define BEEP_ON GPIO_PIN_SET
#define BEEP_OFF GPIO_PIN_RESET
#else
#define BEEP_ON GPIO_PIN_RESET
#define BEEP_OFF GPIO_PIN_SET
#endif
// ==========================================

static AlertType_t current_alert = ALERT_NONE;
static uint32_t alert_start_time = 0;
static uint8_t alert_step = 0;

void Alert_Start(AlertType_t type)
{
    current_alert = type;
    alert_start_time = HAL_GetTick();
    alert_step = 0;

    // 初始状态关闭蜂鸣器和新的外部 LED
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_OFF);
    HAL_GPIO_WritePin(LED_OUTSIDE_GPIO_Port, LED_OUTSIDE_Pin, GPIO_PIN_RESET);
}

uint8_t Alert_Process(void)
{
    if (current_alert == ALERT_NONE)
        return 1;

    uint32_t elapsed_time = HAL_GetTick() - alert_start_time;

    if (current_alert == ALERT_TARGET_FOUND)
    {
        // 比赛要求：闪/响2次，每次大于1秒 (亮1.2s -> 灭0.5s -> 亮1.2s -> 灭0.5s)
        if (elapsed_time < 1200)
        {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_ON);
            HAL_GPIO_WritePin(LED_OUTSIDE_GPIO_Port, LED_OUTSIDE_Pin, GPIO_PIN_SET);
        }
        else if (elapsed_time < 1700)
        {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_OFF);
            HAL_GPIO_WritePin(LED_OUTSIDE_GPIO_Port, LED_OUTSIDE_Pin, GPIO_PIN_RESET);
        }
        else if (elapsed_time < 2900)
        {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_ON);
            HAL_GPIO_WritePin(LED_OUTSIDE_GPIO_Port, LED_OUTSIDE_Pin, GPIO_PIN_SET);
        }
        else if (elapsed_time < 3400)
        {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_OFF);
            HAL_GPIO_WritePin(LED_OUTSIDE_GPIO_Port, LED_OUTSIDE_Pin, GPIO_PIN_RESET);
        }
        else
        {
            current_alert = ALERT_NONE;
            return 1;
        }
    }
    else if (current_alert == ALERT_PARKING)
    {
        // 停车：长鸣 3 秒 (不要求闪灯，安全起见把灯关掉)
        if (elapsed_time < 3000)
        {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_ON);
            HAL_GPIO_WritePin(LED_OUTSIDE_GPIO_Port, LED_OUTSIDE_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_OFF);
            current_alert = ALERT_NONE;
            return 1;
        }
    }
    return 0;
}