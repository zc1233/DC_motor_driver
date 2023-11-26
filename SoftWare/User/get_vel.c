#include <spi.h>
#include <gpio.h>

#include "motor_controller.h"

// AS5047p 地址
#define Com_Ang     0xFFFF //读取角度命令
#define NOP 0x0000 //空命令

#define SPICS(cs) HAL_GPIO_WritePin(SPICS_GPIO_Port, SPICS_Pin, cs)
#define AS5047_SPI hspi2

uint16_t AS5047_read(void);

void TIM3_NVIC_Callback()
{
  static uint16_t angle = 0, last_angle = 0;
  static short diff_angle = 0;
  angle = AS5047_read();
  diff_angle = angle - last_angle;
  last_angle = angle;

  if (diff_angle < -MAXDIFF)
      diff_angle += MAXANGLE;
  else if (diff_angle > MAXDIFF)
      diff_angle -= MAXANGLE;
  
  motor.angle_controller.current += diff_angle;
  motor.speed_controller.current = diff_angle * 1000;
}

uint16_t AS5047_read(void)
{
  uint16_t angle_value;
  static uint16_t command = Com_Ang;
  static uint16_t nop_command = NOP|0x4000;
  SPICS(0);
  HAL_SPI_Transmit(&AS5047_SPI ,(uint8_t *)&command ,1,100);
  SPICS(1);
  SPICS(0);
  HAL_SPI_TransmitReceive(&AS5047_SPI, (uint8_t *)&nop_command, (uint8_t *)&angle_value, 1, 100);
  SPICS(1);
  angle_value &= 0x3FFF;
  return angle_value;
}

