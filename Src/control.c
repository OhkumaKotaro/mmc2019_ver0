#include "stdint.h"
#include "tim.h"
#include "spi.h"
#include "control.h"

extern enc_t enc;
extern gyro_t gyro_z;

#define dt 0.0010f
#define TRUE 1
#define FALSE 0


extern volatile uint8_t motion_end_flag;

//straight
typedef struct 
{
    float up_term;
    float const_term;
    float down_term;
    float v_now;
    float accel;
    float dir;
}target_t;
target_t straight;
volatile float dist_idial;
float error_sum;
float error_old;

//loger
typedef struct{
    float target_velo[3000];
    float target_velo_ang[3000];
    float velo[3000];
    float velo_ang[3000];
}loger_t;

//angle
target_t angle;
loger_t loger;
uint16_t ms_counter;
volatile float ang_idial;
float error_ang_sum;
float error_ang_old;

volatile uint8_t motor_flag;

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

    d = Kd * (error - *old);
    *old = error;

    return (p + i + d);
}

/****************************************************************************************
 * outline  : calcurate accele distance
 * argument : var[mm],velo_m[mm/s],velo_e[mm/s],accel[mm/s],dir
 * return   : void
********************************************************************************************/
void Control_StrCalculator(float var,float velo_s,float velo_m,float velo_e,float accel,float dir){
    straight.up_term = (velo_m*velo_m - straight.v_now*straight.v_now) / 2.0f / accel;
    straight.const_term = var - (velo_m*velo_m - velo_e*velo_e) / 2.0f / accel;
    straight.down_term = var;
    straight.v_now = velo_s;
    straight.accel = accel;
    straight.dir = dir;
    printf("up:%f,cnt:%f,dwn:%f\r\n",straight.up_term,straight.const_term,straight.down_term);
}

/****************************************************************************************
 * outline  : calcurate accele distance
 * argument : var[mm],velo_m[mm/s],velo_e[mm/s],accel[mm/s],dir
 * return   : void
********************************************************************************************/
void Control_AngCalculator(float var,float velo_s,float velo_m,float velo_e,float accel,float dir){
    angle.up_term = (velo_m*velo_m - angle.v_now*angle.v_now) / 2.0f / accel;
    angle.const_term = var - (velo_m*velo_m - velo_e*velo_e) / 2.0f / accel;
    angle.down_term = var;
    angle.v_now = velo_s;
    angle.accel = accel;
    angle.dir = dir;
}

void UpdateTargets(volatile float *var,float *velo,float accel){
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
void TargetGenerator(volatile float *var_idial,target_t *buff)
{
    if (buff->dir == 0.0f)
    {
        buff->v_now=0.0f;
    }
    else
    {
        if (*var_idial < buff->up_term)
        {
            UpdateTargets(var_idial,&buff->v_now,buff->accel);
        }
        else if (*var_idial < buff->const_term)
        {
            UpdateTargets(var_idial,&buff->v_now,0);
        }
        else if (*var_idial < buff->down_term && buff->v_now > 0.0f)
        {
            UpdateTargets(var_idial,&buff->v_now,-(buff->accel));
        }
        else
        {
            *var_idial = 0.0f;
            motion_end_flag = TRUE;
        }
    }
}

void UpdateLoger(void){
    loger.target_velo[ms_counter] = straight.v_now;
    loger.target_velo_ang[ms_counter] = angle.v_now;
    loger.velo[ms_counter] = enc.velocity;
    loger.velo_ang[ms_counter] = gyro_z.velocity;
    ms_counter++;
    if(ms_counter>3000){
        motion_end_flag = TRUE;
        motor_flag = FALSE;
    }
}

void Control_PrintLoger(void){
    printf("str\tang\r\n");
    for(uint16_t i = 0; i < ms_counter; i++)
    {
        printf("%f\t%f\r\n",loger.target_velo[i],loger.target_velo_ang[i]);
    }
    
}

void Control_UpdatePwm(void){
    if(motor_flag==TRUE){
        TargetGenerator(&dist_idial,&straight);
        TargetGenerator(&ang_idial,&angle);
        int16_t str_buff = (int16_t)PID_value(straight.dir*straight.v_now,enc.velocity,&error_sum,&error_old,0.0f,0,0);
        int16_t ang_buff = (int16_t)PID_value(angle.dir*angle.v_now,gyro_z.velocity,&error_ang_sum,&error_ang_old,1.0f,0.0f,0);
        Tim_MotorPwm(str_buff-ang_buff,str_buff+ang_buff);
        //UpdateLoger();
    }else{
        Tim_MotorPwm(0,0);
    }
}

void Control_ResetVelo(void){
    straight.v_now = 0.0f;
    dist_idial = 0.0f;
    angle.v_now = 0.0f;
    ang_idial = 0.0f;
    ms_counter = 0;
}