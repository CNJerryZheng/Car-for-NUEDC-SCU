#pragma once

#include "main.h"

void Chassis_Init(void);

void Chassis_Update(void);

void Chassis_SetPhysicalSpeed(float speed_L_m_s, float speed_R_m_s);

void Chassis_Stop_And_Reset(void);

void Chassis_SetTrackingBaseSpeed(float speed);
