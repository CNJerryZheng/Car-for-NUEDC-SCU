#ifndef __SENSOR_H
#define __SENSOR_H

#include "main.h"
#include <stdint.h>

float Get_Line_Error(uint8_t state);
float Hcsr04GetLength(void);
uint8_t Get_XunJi_State(void);

#endif /* __SENSOR_H */