#ifndef __OPENMV_H
#define __OPENMV_H

#include "main.h"

// 暴露出这两个全局变量，供你的状态机调用
extern uint8_t openmv_found_target; // 1代表找到，0代表没找到
extern int16_t openmv_x_error;      // X轴偏差 (正数偏右，负数偏左)

// 初始化函数
void OpenMV_Init(void);

#endif