/*  电机控制函数   */

#include <tim.h>
#include <stdint.h>

#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#define MAXANGLE 0x4000
#define MAXDIFF 0x2000

#define MOTOR_MAX_SPEED 120.0F*MAXANGLE   //电机最大速度 线数/秒
#define MOTOR_MAX_ACCELERATION 950.0f    //电机最大加速度

typedef enum 
{
    speed = 0,
    angle = 1
}type_t;

typedef struct motor_controll motor_controll_t;
typedef struct motor motor_t;
typedef struct pid pid_t;

typedef void (*controll_realize_t)(motor_controll_t *motor_controll);

struct pid
{
    float kp;
    float ki;
    float kd;
    float err;
    float err_last;
    float integral;
    float output;
    float max_output;
    float maxIntegral;
    float deadband;
    motor_controll_t* father;
    void (*pid_realize)(struct pid *pid);
} ;

struct motor_controll
{
    pid_t pid;
    int target;
    int current;
    uint8_t type;
    motor_t* father;
    void (*motor_controll_realize)(motor_controll_t *motor_controll);
} ;

struct motor
{
    uint8_t type;
    motor_controll_t speed_controller;
    motor_controll_t angle_controller;
    void (*motor_update)(motor_t *motor);
} ;

extern motor_t motor;
extern motor_controll_t speed_controller, angle_controller;
extern pid_t speed_pid, angle_pid;

void motor_init(motor_t *motor);
void TIM3_NVIC_Callback(void);

#endif // _MOTOR_CONTROLLER_H_