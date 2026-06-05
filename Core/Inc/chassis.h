#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"

void Chassis_Init(void);

void Chassis_Update(void);

void Chassis_SetPhysicalSpeed(float speed_L_m_s, float speed_R_m_s);

#endif