#ifndef MOTOR_UPDATE_H
#define MOTOR_UPDATE_H

#include "motor_controller.h"
#include <cmsis_os.h>

extern uint8_t error_flag;
extern osTimerId IDTimerHandle;
extern osTimerId Error_TimerHandle;

void Error_LED(void const * argument);

void Motor_Update(void const * argument);

#endif // !