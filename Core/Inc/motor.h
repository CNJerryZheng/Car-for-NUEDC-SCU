#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

void Set_Motor_Output(char motor_id, int16_t speed);
void Car_Go(int16_t speed);
void Car_Back(int16_t speed);
void Car_Stop(void);
void Car_Left(int16_t speed);
void Car_Right(int16_t speed);
void Car_Left_Precision(int16_t Lspeed,int16_t Rspeed);
void Car_Right_Precision(int16_t Lspeed,int16_t Rspeed);

#endif /* __MOTOR_H */