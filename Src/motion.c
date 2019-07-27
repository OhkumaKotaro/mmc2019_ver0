#include "motion.h"
#include "stdint.h"
#include "control.h"
#include "tim.h"
#include "spi.h"
#include "MazeCon.h"

#define TRUE 1
#define FALSE 0

#define VELO_S 400.0f
#define ACCEL_S 4000.0f
#define VELO_ANG_S 400.0f
#define ACCEL_ANG_S 8000.0f
#define VELO_ANG_S_SLALOM 600.0f
#define ACCEL_ANG_S_SLALOM 10000.0f

#define OFFSET_IN 31.0f
#define OFFSET_OUT 47.0f

extern enc_t enc;
extern volatile uint8_t motor_flag;
extern target_t straight, angle;
extern volatile accel_t accel;

volatile uint8_t motion_end_flag;
float gain_velo = 150;

void Motion_enkai(void)
{
    Control_ResetVelo();
    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    while (1)
    {
    }
}
void Motion_Start(void)
{
    Control_ResetVelo();
    Control_StrCalculator(130.0f, 0.0f, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_Restart(uint8_t wall_is)
{
    if (wall_is == FALSE)
    {
        Control_StrCalculator(90.0f, 0.0f, VELO_S, VELO_S, ACCEL_S, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset -= 90.0f;
    }
    else
    {
        Control_StrCalculator(0, 0, 0, 0, 1, 0);
        Control_AngCalculator(185.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        straight.v_now = 0;
        angle.v_now = 0;
        HAL_Delay(800);

        Control_StrCalculator(100.0f, 0, 200, 0, 1000, -1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        straight.v_now = 0;
        angle.v_now = 0;
        HAL_Delay(800);

        Control_StrCalculator(130.0f, 0, VELO_S, VELO_S, ACCEL_S, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }

        enc.offset = 0;
    }
}

void Motion_End(void)
{
    Control_StrCalculator(90.0f - enc.offset, VELO_S, VELO_S, 0.0f, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    enc.offset = 0;
    HAL_Delay(800);
}

void Motion_Straight(void)
{
    //Control_ResetVelo();
    Control_StrCalculator(180.0f - enc.offset, VELO_S, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_TestTurn(void)
{
    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(185.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    //Tim_MotorPwm(0, 0);
    HAL_Delay(800);
}

void Motion_SpinTurn(void)
{
    Control_StrCalculator(90.0f - enc.offset, VELO_S, VELO_S, 0, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    Tim_MotorPwm(0, 0);
    HAL_Delay(500);
    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(185.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    Tim_MotorPwm(0, 0);
    HAL_Delay(500);
    Control_StrCalculator(90.0f, 0, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_WallSpinTurn(void)
{
    float buff = enc.offset;
    Control_StrCalculator(90.0f - buff, VELO_S, VELO_S, 0, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    HAL_Delay(800);

    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(185.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    HAL_Delay(800);

    Control_StrCalculator(100.0f, 0, 200, 0, 1000, -1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    enc.offset = 0;
    HAL_Delay(800);

    Control_StrCalculator(130.0f, 0, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }

    enc.offset -= 130.0f;
}

void Motion_LeftSpinTurn(void)
{
    Control_StrCalculator(90.0f - enc.offset, VELO_S, VELO_S, 0.0f, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }

    Tim_MotorPwm(0, 0);
    HAL_Delay(500);
    Control_StrCalculator(0, 0, 0, 0, 0, 1);
    Control_AngCalculator(90.0f, 0, VELO_ANG_S, 0, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }

    Tim_MotorPwm(0, 0);
    HAL_Delay(500);
    Control_StrCalculator(90.0f, 0.0f, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_RightSpinTurn(void)
{
    volatile unsigned int i = 0;
    Control_StrCalculator(90.0f - enc.offset, VELO_S, VELO_S, 0.0f, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }

    motor_flag = FALSE;
    for (i = 0; i < 5000; i++)
    {
    }
    motor_flag = TRUE;
    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(90.0f, 0, VELO_ANG_S, 0, ACCEL_ANG_S, -1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    motor_flag = FALSE;
    enc.offset = 0;
    for (i = 0; i < 5000; i++)
    {
    }
    motor_flag = TRUE;
    Control_StrCalculator(90.0f, 0.0f, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset -= 90.0f;
}

void Motion_LeftTurn(void)
{
    //control_wall_flag = FALSE;
    //offset in
    Control_StrCalculator(OFFSET_IN - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(200.0f, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(90.88f, 0, VELO_ANG_S_SLALOM, 0, ACCEL_ANG_S_SLALOM, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(OFFSET_OUT, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_RightTurn(void)
{
    //control_wall_flag = FALSE;
    //offset in
    Control_StrCalculator(OFFSET_IN + 2.0f - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(180.0f, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(90.80f, 0, VELO_ANG_S_SLALOM, 0, ACCEL_ANG_S_SLALOM, -1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(OFFSET_OUT - 4.0f, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_FastStart(uint8_t step){
    Control_ResetVelo();
    float velo = VELO_S + gain_velo * step;
    if(velo>2500){
        velo=2500;
    }
    Control_StrCalculator(130 + 180 * step, 0, velo, VELO_S, ACCEL_S,1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_FastStraight(uint8_t step)
{
    float velo = VELO_S + gain_velo * step;
    Control_StrCalculator(180 * step - enc.offset, VELO_S, velo, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_FastGoal(uint8_t step)
{
    float velo = VELO_S + gain_velo * step;
    if(velo>2500){
        velo=2500;
    }
    Control_StrCalculator(90 + 180 * step - enc.offset, VELO_S, velo, 0, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1 ,0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    enc.offset = 0;
    HAL_Delay(800);
}



void Motion_Switch(uint8_t motion)
{
    switch (motion)
    {
    case FRONT:
        Motion_Straight();
        break;
    case RIGHT:
        Motion_RightTurn();
        break;
    case LEFT:
        Motion_LeftTurn();
        break;
    case REAR:
        Motion_SpinTurn();
        break;
    case PIVO_REAR:
        Motion_WallSpinTurn();
        break;
    default:
        motor_flag = FALSE;
        break;
    }
}
