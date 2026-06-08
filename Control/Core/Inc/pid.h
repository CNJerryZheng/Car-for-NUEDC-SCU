#pragma once

#include <stdint.h>

typedef struct
{
    float Kp; // P 系数
    float Ki; // I 系数
    float Kd; // D 系数

    float Error; // 当前偏差
    float Last_Error; // 上一次的偏差
    float Prev_Error; // 上上次的偏差
} PID_TypeDef;

/* 导出函数 */
void PID_Init(PID_TypeDef* pid, float p, float i, float d);

float PID_Locomotive_Calc(PID_TypeDef* pid, float target, float current);