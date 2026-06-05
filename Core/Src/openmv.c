#include "openmv.h"
#include "usart.h" // 确保能调用 huart1

// 全局变量定义
uint8_t openmv_found_target = 0;
int16_t openmv_x_error = 0;

// 接收缓存
uint8_t rx_buffer[1]; // 每次中断接收1个字节
uint8_t rx_frame[5];  // 完整数据包缓存
uint8_t rx_index = 0; // 接收索引

/**
 * @brief  OpenMV 串口通信初始化
 */
void OpenMV_Init(void)
{
    // 开启 USART1 的中断接收，每次接收 1 个字节放到 rx_buffer 里
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
}

/**
 * @brief  串口接收完成中断回调函数 (HAL库内部弱函数的重写)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断是不是 USART1 触发的中断
    if (huart->Instance == USART1)
    {
        // === 数据包解析状态机 ===
        if (rx_index == 0 && rx_buffer[0] == 0x55)
        {
            // 等到了帧头 0x55
            rx_frame[rx_index++] = rx_buffer[0];
        }
        else if (rx_index > 0)
        {
            // 存入后续数据
            rx_frame[rx_index++] = rx_buffer[0];

            if (rx_index == 5) // 收满 5 个字节了
            {
                if (rx_frame[4] == 0xAA) // 校验帧尾是不是 0xAA
                {
                    // 数据包合法！提取数据
                    openmv_found_target = rx_frame[1];
                    // 将两个 8 位数据拼回 16 位有符号整数
                    openmv_x_error = (int16_t)((rx_frame[2] << 8) | rx_frame[3]);
                }
                // 解析完毕，索引清零，准备迎接下一个包
                rx_index = 0;
            }
        }
        else
        {
            // 如果第一个字节不是 0x55，说明数据乱了，直接丢弃
            rx_index = 0;
        }

        // 重新开启中断接收，等待下一个字节
        HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    }
}