#include "stdint.h"
#include "tim.h"
#include "spi.h"
#include "control.h"
#include "flash.h"
#include "adc.h"
#include "gpio.h"
#include "arm_math.h"

#define dt 0.0010f
#define TRUE 1
#define FALSE 0
#define SLIP_K 50000.0f

extern sensor_t sen_l;
extern sensor_t sen_fl;
extern sensor_t sen_front;
extern sensor_t sen_fr;
extern sensor_t sen_r;
extern enc_t enc;
extern gyro_t gyro_z;
extern volatile accel_t accel;

extern volatile uint8_t motion_end_flag;

//straight
target_t straight;
volatile float dist_idial = 0.0f;
float error_sum;
float error_old;

//angle
target_t angle;
loger_t loger;
volatile float ang_idial = 0.0f;
float error_ang_sum;
float error_ang_old;

volatile uint8_t motor_flag;

float slip_ang = 0.0f;
uint8_t slip_cnt = 0;

//control wall
float wall_dif = 0;
unsigned char add_l = 0, add_r = 0;
volatile uint8_t control_wall_flag;
float front_wall_diff = 0;
volatile uint8_t front_wall_flag;

/****************************************************************************************
 * outline  : PID control
 * argument : 
 * return   : control value
********************************************************************************************/
float PID_value(float target, float measured, float *sum, float *old, float Kp, float Ki, float Kd)
{
    float error;
    float p, i, d;

    error = target - measured;
    p = Kp * error;

    *sum += error * dt;
    i = *sum * Ki;

    d = Kd * (*old - error);
    *old = error;

    return (p + i + d);
}

/****************************************************************************************
 * outline  : calcurate accele distance
 * argument : var[mm],velo_m[mm/s],velo_e[mm/s],accel[mm/s],dir
 * return   : void
********************************************************************************************/
void Control_StrCalculator(float var, float velo_s, float velo_m, float velo_e, float accel, float dir)
{
    if (dir != 0)
    {
        straight.up_term = (velo_m * velo_m - velo_s * velo_s) / 2.0f / accel;
        straight.const_term = var - (velo_m * velo_m - velo_e * velo_e) / 2.0f / accel;
        straight.down_term = var;
        straight.accel = accel;
        dist_idial = 0;
    }
    straight.v_now = velo_s;
    straight.dir = dir;
}

/****************************************************************************************
 * outline  : calcurate accele distance
 * argument : var[mm],velo_m[mm/s],velo_e[mm/s],accel[mm/s],dir
 * return   : void
********************************************************************************************/
void Control_AngCalculator(float var, float velo_s, float velo_m, float velo_e, float accel, float dir)
{
    if (dir != 0)
    {
        //gyro_z.degree = 0;
        angle.up_term = (velo_m * velo_m - velo_s * velo_s) / 2.0f / accel;
        angle.const_term = var - (velo_m * velo_m - velo_e * velo_e) / 2.0f / accel;
        angle.down_term = var;
        angle.accel = accel;
        ang_idial = 0;
    }
    angle.v_now = velo_s;
    angle.dir = dir;
}

void UpdateTargets(volatile float *var, float *velo, float accel)
{
    volatile float buff;

    buff = *velo * 1000.0f;
    buff += accel;
    *velo = buff / 1000.0f;

    buff = *var * 1000.0f;
    buff += *velo;
    *var = buff / 1000.0f;
}

/****************************************************************************************
 * outline  : output pwm for trapezoid accele straight by feadbuck control
 * argument : void
 * return   : void
********************************************************************************************/
void UpdateStrTarget()
{
    if (straight.dir == 0.0f)
    {
        //straight.v_now = 0.0f;
    }
    else
    {
        if (dist_idial < straight.up_term)
        {
            UpdateTargets(&dist_idial, &straight.v_now, straight.accel);
        }
        else if (dist_idial < straight.const_term)
        {
            UpdateTargets(&dist_idial, &straight.v_now, 0);
        }
        else if (dist_idial < straight.down_term && straight.v_now > 0.0f)
        {
            UpdateTargets(&dist_idial, &straight.v_now, -(straight.accel));
        }
        else
        {
            motion_end_flag = TRUE;
        }
    }
}

void Control_Side_Wall(void)
{
    float kp = 0.20f;
    if (sen_l.now > sen_l.threshold + add_l && sen_r.now > sen_r.threshold + add_r)
    {

        Gpio_SideLed(LED_L | LED_R);
        wall_dif = kp * ((sen_l.now - sen_l.reference) - (sen_r.now - sen_r.reference));
        add_l = 0;
        add_r = 0;
    }
    else if (sen_l.now > sen_l.threshold + add_l)
    {
        Gpio_SideLed(LED_L);
        wall_dif = 2 * kp * (sen_l.now - sen_l.reference);
        add_l = 0;
    }
    else if (sen_r.now > sen_r.threshold + add_r)
    {
        Gpio_SideLed(LED_R);
        wall_dif = -2 * kp * (sen_r.now - sen_r.reference);
        add_r = 0;
    }
    else
    {
        wall_dif = 0;
        Gpio_SideLed(0);
    }

    if (sen_l.diff < -10)
    {
        add_l = 10;
        wall_dif = 0;
        Gpio_SideLed(0);
    }
    if (sen_r.diff < -10)
    {
        add_r = 10;
        wall_dif = 0;
        Gpio_SideLed(0);
    }

    if (angle.dir != 0 || enc.velocity < 300)
    {
        wall_dif = 0;
        Gpio_SideLed(0);
    }
}

void Control_FrontWall(void)
{
    if (front_wall_flag == TRUE)
    {
        int16_t diff = sen_front.reference - sen_front.now;
        front_wall_diff = 3 * diff;
        if (diff < 2 && diff > -2)
        {
            front_wall_diff = 0;
            front_wall_flag = FALSE;
        }
    }
}

void UpdateAngTarget(void)
{
    if (angle.dir == 0.0f)
    {
        //angle.v_now = 0.0f;
    }
    else
    {
        if (ang_idial < angle.up_term)
        {
            UpdateTargets(&ang_idial, &angle.v_now, angle.accel);
        }
        else if (ang_idial < angle.const_term)
        {
            UpdateTargets(&ang_idial, &angle.v_now, 0);
        }
        else if (ang_idial < angle.down_term && angle.v_now > 0.0f)
        {
            UpdateTargets(&ang_idial, &angle.v_now, -(angle.accel));
        }
        else
        {
            ang_idial = 0.0f;
            angle.error = 0;
            motion_end_flag = TRUE;
        }
    }
}

void UpdateLoger(void)
{
    /*
    loger.target_velo[loger.cnt] = (int16_t)straight.v_now;
    loger.target_velo_ang[loger.cnt] = (int16_t)angle.v_now;
    loger.velo[loger.cnt] = (int16_t)enc.velocity;
    loger.velo_ang[loger.cnt] = (int16_t)gyro_z.velocity;
    */
    //loger.velo[loger.cnt] = (int16_t)accel.y;
    loger.target_velo[loger.cnt] = sen_l.now;
    loger.target_velo_ang[loger.cnt] = sen_r.now;
    loger.velo[loger.cnt] = sen_l.diff;
    loger.velo_ang[loger.cnt] = sen_r.diff;
    loger.cnt++;
    if (loger.cnt > 5000)
    {
        motion_end_flag = TRUE;
        motor_flag = FALSE;
    }
}

void UpdateSlipAngle(void)
{
    if (straight.v_now != 0)
    {
        if (slip_cnt == 4)
        {
            slip_ang = (slip_ang * 200.0 - angle.v_now) / (200.0 + SLIP_K / straight.v_now);
            slip_cnt = 0;
        }
        slip_cnt++;
    }
    else
    {
        slip_ang = 0;
    }
}

void Control_UpdatePwm(void)
{
    if (motor_flag == TRUE)
    {
        //UpdateLoger();
        if (motion_end_flag == FALSE)
        {
            UpdateStrTarget();
            UpdateAngTarget();
        }
        if (control_wall_flag == TRUE)
        {
            Control_Side_Wall();
        }
        Control_FrontWall();
        int16_t str_buff = (int16_t)PID_value(straight.dir * straight.v_now + front_wall_diff, enc.velocity, &error_sum, &error_old, 1.7f, 11.5f, 0);  //1.7f, 11.5f, 0
        int16_t ang_buff = (int16_t)PID_value(angle.dir * angle.v_now - wall_dif, gyro_z.velocity, &error_ang_sum, &error_ang_old, 0.8f, 20.0f, 1.0f); // 1.0f, 52.0f, 1.0f
        /*
        if(straight.dir == -1){
            ang_buff = 0;
        }
        */
        Tim_MotorPwm(str_buff - ang_buff, str_buff + ang_buff);
        if ((angle.dir * angle.v_now - gyro_z.velocity < -400) || (angle.dir * angle.v_now - gyro_z.velocity) > 400)
        {
            motor_flag = FALSE;
        }
        if (accel.y < -60.0f)
        {
            motor_flag = FALSE;
        }
    }
    else if (motor_flag == FALSE)
    {
        Tim_MotorPwm(0, 0);
    }
}

void Control_ResetVelo(void)
{
    straight.v_now = 0.0f;
    dist_idial = 0.0f;
    error_sum = 0.0f;
    error_old = 0.0f;
    angle.v_now = 0.0f;
    ang_idial = 0.0f;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    loger.cnt = 0;
    enc.offset = 0.0f;
    enc.distance = 0;
    front_wall_flag = FALSE;
    front_wall_diff = 0;
}