#pragma once

#include "main.h"
#include "usart.h"

// 声明外部变量
extern const uint8_t JUSTFLOAT_TAIL[4];

__STATIC_INLINE void JustFloat_SendTail(void)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)JUSTFLOAT_TAIL, sizeof(JUSTFLOAT_TAIL), 100);
}

__STATIC_INLINE void JustFloat_Send(float* data, uint8_t count)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, count * 4, 100);
    JustFloat_SendTail();
}
