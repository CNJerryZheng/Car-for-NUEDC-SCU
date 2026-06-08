#include "openmv.h"
#include "usart.h"

// 全局变量定义
uint8_t openmv_found_target = 0;
int16_t openmv_x_error = 0;
uint8_t openmv_stop_flag = 0;

#define RX_BUF_SIZE 128 // 环形缓冲区大小
uint8_t rx_dma_buffer[RX_BUF_SIZE];
uint16_t rx_process_index = 0;

// 协议解析临时缓存
uint8_t rx_frame[6];
uint8_t rx_index = 0;

// OpenMV 串口通信初始化
void OpenMV_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer, RX_BUF_SIZE);

    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        while (rx_process_index != Size)
        {
            uint8_t byte = rx_dma_buffer[rx_process_index];

            if (rx_index == 0 && byte == 0x55)
            {
                rx_frame[rx_index++] = byte;
            }
            else if (rx_index > 0)
            {
                rx_frame[rx_index++] = byte;

                if (rx_index == 6)
                {
                    // 校验帧尾
                    if (rx_frame[5] == 0xAA)
                    {
                        openmv_found_target = rx_frame[1];
                        openmv_x_error = (int16_t)((rx_frame[2] << 8) | rx_frame[3]);
                        openmv_stop_flag = rx_frame[4];
                    }
                    rx_index = 0;
                }
            }
            else
            {
                rx_index = 0;
            }

            rx_process_index++;

            if (rx_process_index >= RX_BUF_SIZE)
            {
                rx_process_index = 0;
            }
        }
    }
}