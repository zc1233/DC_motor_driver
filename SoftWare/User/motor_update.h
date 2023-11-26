#ifndef MOTOR_UPDATE_H
#define MOTOR_UPDATE_H

#include "motor_controller.h"

extern uint8_t error_flag;

void Error_LED(void const * argument);

void Motor_Update(void const * argument);

#endif // !