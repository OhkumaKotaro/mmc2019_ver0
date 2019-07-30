#include "motion.h"
#include "stdint.h"
#include "control.h"
#include "tim.h"
#include "spi.h"
#include "MazeCon.h"
#include "arm_math.h"
#include "adc.h"

#define TRUE 1
#define FALSE 0

#define VELO_ANG_B 380.0f
#define ACCEL_ANG_B 4000.0f

#define OFFSET_IN 31.0f
#define OFFSET_OUT 47.0f

extern enc_t enc;
extern volatile uint8_t motor_flag;
extern target_t straight, angle;
extern volatile accel_t accel;
extern sensor_t sen_front;
extern int16_t sen_front_reference_f;

extern float error_sum;
extern float error_old;

extern float error_ang_sum;
extern float error_ang_old;

extern volatile uint8_t front_wall_flag;

volatile uint8_t motion_end_flag;
float gain_velo = 150;
float diagonal_step = 0;

uint8_t Reset_Position(uint8_t next_motion)
{
    uint8_t flag = 1;
    if (sen_front.is_wall)
    {
        uint8_t diff = sen_front_reference_f - sen_front.now;
        if (diff > 10 || diff < -5)
        {
            flag = 0;
            Control_StrCalculator(90.0f - enc.offset, VELO_S, VELO_S, 0, ACCEL_S, 1);
            Control_AngCalculator(0, 0, 0, 0, 1, 0);
            motion_end_flag = FALSE;
            while (motion_end_flag == FALSE)
            {
            }

            straight.v_now = 0;
            angle.v_now = 0;
            error_ang_sum = 0.0f;
            error_ang_old = 0.0f;
            HAL_Delay(500);

            front_wall_flag = TRUE;
            while (front_wall_flag == TRUE)
            {
            }

            straight.v_now = 0;
            angle.v_now = 0;
            error_ang_sum = 0.0f;
            error_ang_old = 0.0f;
            HAL_Delay(500);
            if (next_motion == LEFT)
            {
                Control_StrCalculator(0, 0, 0, 0, 1, 0);
                Control_AngCalculator(90.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
            }
            else if (next_motion == RIGHT)
            {
                Control_StrCalculator(0, 0, 0, 0, 1, 0);
                Control_AngCalculator(90.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, -1);
            }
            motion_end_flag = FALSE;
            while (motion_end_flag == FALSE)
            {
            }
            straight.v_now = 0;
            angle.v_now = 0;
            HAL_Delay(800);

            Control_StrCalculator(90.0f, 0.0f, VELO_S, VELO_S, ACCEL_S, 1);
            Control_AngCalculator(0, 0, 0, 0, 1, 0);
            motion_end_flag = FALSE;
            while (motion_end_flag == FALSE)
            {
            }

            enc.offset = 0.0f;
        }
    }
    return flag;
}

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
    diagonal_step = 180.0f * sqrt(2);
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
    Control_ResetVelo();
    if (wall_is == FALSE)
    {
        Control_StrCalculator(90.0f, 0.0f, VELO_S, VELO_S, ACCEL_S, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0;
    }
    else
    {
        front_wall_flag = TRUE;
        while (front_wall_flag == TRUE)
        {
        }
        straight.v_now = 0;
        angle.v_now = 0;
        error_ang_sum = 0.0f;
        error_ang_old = 0.0f;
        HAL_Delay(500);

        Control_StrCalculator(0, 0, 0, 0, 1, 0);
        Control_AngCalculator(187.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        straight.v_now = 0;
        angle.v_now = 0;
        error_ang_sum = 0.0f;
        error_ang_old = 0.0f;
        HAL_Delay(500);

        Control_StrCalculator(50.0f, 0, 200, 0, 1000, -1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        straight.v_now = 0;
        angle.v_now = 0;
        error_ang_sum = 0.0f;
        error_ang_old = 0.0f;
        HAL_Delay(500);

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
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    straight.v_now = 0;
    angle.v_now = 0;
    enc.offset = 0;
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
    Control_ResetVelo();
    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(187.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
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
    straight.v_now = 0.0f;
    angle.v_now = 0.0f;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);
    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(187.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0.0f;
    angle.v_now = 0.0f;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(800);
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
    Control_StrCalculator(90.0f - enc.offset, VELO_S, VELO_S, 0, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }

    straight.v_now = 0;
    angle.v_now = 0;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);

    front_wall_flag = TRUE;
    while (front_wall_flag == TRUE)
    {
    }

    straight.v_now = 0;
    angle.v_now = 0;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);

    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(187.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(800);

    Control_StrCalculator(50.0f, 0, 100, 0, 1000, -1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);

    Control_StrCalculator(130.0f, 0, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }

    enc.offset = 0.0f;
}

void Motion_LeftTurn(void)
{
    if (Reset_Position(LEFT))
    {
        //offset in
        Control_StrCalculator(35.0f - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        //turn
        Control_StrCalculator(180.0f, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(90.88f, 0, VELO_ANG_S_SLALOM, 0, ACCEL_ANG_S_SLALOM, 1); //90.88f
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        //offset out
        Control_StrCalculator(40.0f, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
    }
}

void Motion_RightTurn(void)
{
    if (Reset_Position(RIGHT))
    {
        //offset in
        Control_StrCalculator(33.0f - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        //turn
        Control_StrCalculator(180.0f, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(90.8f, 0, VELO_ANG_S_SLALOM, 0, ACCEL_ANG_S_SLALOM, -1); //90.80f
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        //offset out
        Control_StrCalculator(48.0f, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
    }
}

void Motion_FastStart(uint8_t step, float velo_end)
{
    diagonal_step = 180.0f * sqrt(2);
    Control_ResetVelo();
    float velo = VELO_F + gain_velo * step;
    if (velo > 2500)
    {
        velo = 2500;
    }
    Control_StrCalculator(130 + 180 * step, 0, velo, velo_end, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
}

void Motion_FastStraight(uint8_t step,float v_start,float v_end)
{
    float velo = VELO_F + gain_velo * step;
    Control_StrCalculator(180 * step, VELO_S, velo, VELO_S, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
}

void Motion_FastGoal(uint8_t step,float v_start)
{
    float velo = VELO_F + gain_velo * step;
    if (velo > 2500)
    {
        velo = 2500;
    }
    Control_StrCalculator(90 + 180 * step, VELO_S, velo, 0, ACCEL_F, 1);
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
void Motion_Diagonal(uint8_t step)
{
    float velo = VELO_F + gain_velo * step;
    Control_StrCalculator(diagonal_step * step, VELO_S, velo, VELO_S, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
}

void Motion_DiagonalStop(void)
{
    Control_StrCalculator(diagonal_step, VELO_F, VELO_F, 0, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);
}

void Motion_Left45Turn(float v_start,float v_end)
{
    //offset in
    Control_StrCalculator(102.0f, v_start, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(800.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(45.0f, 0, VELO_ANG_D, 0, ACCEL_ANG_D, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(65.0f, VELO_F, VELO_F, v_end, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
}

void Motion_Right45Turn(float v_start,float v_end)
{
    //offset in
    Control_StrCalculator(102.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(800.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(45.0f, 0, VELO_ANG_D, 0, ACCEL_ANG_D, -1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(65.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
}

void Motion_Left90Turn(void)
{
    //control_wall_flag = FALSE;
    //offset in
    Control_StrCalculator(OFFSET_IN, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(400.0f, VELO_S, VELO_S, VELO_S, 1, 1);
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
    //error_ang_sum = 0.0f;
    //error_ang_old = 0.0f;
}

void Motion_Right90Turn(void)
{
    //control_wall_flag = FALSE;
    //offset in
    Control_StrCalculator(OFFSET_IN + 2.0f, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(400.0f, VELO_S, VELO_S, VELO_S, 1, 1);
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
    //error_ang_sum = 0.0f;
    //error_ang_old = 0.0f;
}

void Motion_Left90BigTurn(float v_start,float v_end)
{
    //control_wall_flag = FALSE;
    //offset in
    Control_StrCalculator(103.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(600.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(90.88f, 0, VELO_ANG_B, 0, ACCEL_ANG_B, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(156.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //error_ang_sum = 0.0f;
    //error_ang_old = 0.0f;
}

void Motion_Right90BigTurn(float v_start,float v_end)
{
    //control_wall_flag = FALSE;
    //offset in
    Control_StrCalculator(100.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(600.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(90.80f, 0, VELO_ANG_B, 0, ACCEL_ANG_B, -1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(165.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //error_ang_sum = 0.0f;
    //error_ang_old = 0.0f;
}