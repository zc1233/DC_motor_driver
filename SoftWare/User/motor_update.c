#include "motor_update.h"
#include <gpio.h>

uint8_t error_flag = 0;

void Error_LED(void const * argument)
{
    static uint8_t i = 0;
    if (i < 2*error_flag)
    {
        HAL_GPIO_TogglePin(ERRLED_GPIO_Port, ERRLED_Pin);
    }
    else if (i == 2*error_flag + 4)
    {
        i = -1;
    }
    i++;
}

void Motor_Update(void const * argument)
{
	osTimerStart(IDTimerHandle, 500);
  osTimerStart(Error_TimerHandle, 500);
	motor_t *motor = (motor_t *)argument;
	while (1)
	{
			motor->motor_update(motor);
			osDelay(2);
	}
}