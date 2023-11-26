#include "motor_controller.h"

motor_t motor;
motor_controll_t speed_controller;
motor_controll_t angle_controller;
pid_t speed_pid;
pid_t angle_pid;

void set_acceleration(float acceleration);

void motor_controll_realize(motor_controll_t *motor_controll)
{
    motor_controll->pid.pid_realize(&motor_controll->pid);
}

void motor_controll_init(motor_controll_t *motor_controll, uint8_t type, float kp, float ki, float kd, float max_output, float maxIntegral, float deadband, pid_realize_t pid_realize)
{
    motor_controll->pid.pid_init(&motor_controll->pid, kp, ki, kd, max_output, maxIntegral, deadband, pid_realize);
    motor_controll->target = 0;
    motor_controll->current = 0;
    motor_controll->father = &motor;
    motor_controll->pid.father = motor_controll;
    motor_controll->motor_controll_realize = motor_controll_realize;
    motor_controll->type = type;
}

void get_pid_output(pid_t *pid)
{
    pid->err = pid->father->target - pid->father->current;
    if (pid->err > pid->deadband || pid->err < -pid->deadband)
    {
        pid->integral += pid->err;
        if (pid->integral > pid->maxIntegral)
            pid->integral = pid->maxIntegral;
        else if (pid->integral < -pid->maxIntegral)
            pid->integral = -pid->maxIntegral;
        pid->output = pid->kp * pid->err + pid->ki * pid->integral + pid->kd * (pid->err - pid->err_last);
        if (pid->output > pid->max_output)
            pid->output = pid->max_output;
        else if (pid->output < -pid->max_output)
            pid->output = -pid->max_output;
        pid->err_last = pid->err;
    }
    else
    {
        pid->output = 0;
        pid->integral = 0;
    }
} 

void speed_pid_realize(pid_t *pid)
{
    pid->pid_output(pid);
    set_acceleration(pid->output);
}

void angle_pid_realize(pid_t *pid)
{
    pid->pid_output(pid);

    motor_controll_t* temp_speed_controller = &pid->father->father->speed_controller;
    temp_speed_controller->target = pid->output;
    temp_speed_controller->pid.pid_realize(&temp_speed_controller->pid);
}

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output, float maxIntegral, float deadband, pid_realize_t pid_realize)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->maxIntegral = maxIntegral;
    pid->deadband = deadband;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->pid_init = pid_init;
    pid->pid_realize = pid_realize;
    pid->pid_output = get_pid_output;
}

void motor_update(motor_t *motor)
{
    if (motor->type)
    {
        motor->angle_controller.motor_controll_realize(&motor->angle_controller);
    }
    else
    {
        motor->speed_controller.motor_controll_realize(&motor->speed_controller);
    }
}

void motor_init(motor_t *motor)
{
    motor->type = 0;
    motor->speed_controller = speed_controller;
    motor->angle_controller = angle_controller;
    motor_controll_init(&motor->speed_controller, 0, 0.5f, 0.0f, 0.0f, MOTOR_MAX_ACCELERATION, 1000.0f, 0.1f, speed_pid_realize);
    motor_controll_init(&motor->angle_controller, 1, 0.5f, 0.0f, 0.0f, MOTOR_MAX_SPEED, 1000.0f, 0.1f, angle_pid_realize);
    motor->motor_init = motor_init;
    motor->motor_update = motor_update;
}

void set_acceleration(float acceleration)
{
    if (acceleration >= 0)
    {
        TIM2->CCR3 = acceleration;
        TIM2->CCR4 = 0;
    }
    else
    {
        TIM2->CCR3 = 0;
        TIM2->CCR4 = -acceleration;
    }
}

