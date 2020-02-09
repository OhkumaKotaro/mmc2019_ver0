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

#define OFFSET_SEN 135

extern enc_t enc;
extern volatile uint8_t motor_flag;
extern target_t straight, angle;
extern volatile accel_t accel;
extern sensor_t sen_front;
extern sensor_t sen_l;
extern sensor_t sen_r;
extern int16_t sen_front_reference_f;

extern float error_sum;
extern float error_old;

extern float error_ang_sum;
extern float error_ang_old;

extern volatile uint8_t control_wall_flag;

extern volatile uint8_t front_wall_flag;

volatile uint8_t motion_end_flag;
float gain_velo = 0;
float diagonal_step = 0;
uint16_t prefront_flag = 0;

void Motion_MaxVeloSet(float gain)
{
    //0~160
    gain_velo = gain;
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

void Motion_TestTurn(void)
{
    Control_ResetVelo();
    Control_StrCalculator(0, 0, 0, 0, 1, 0);
    Control_AngCalculator(180.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    //Tim_MotorPwm(0, 0);
    HAL_Delay(500);
}

void Motion_Start(void)
{
    Control_StrCalculator(130.0f, 0.0f, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
    //prefront_flag = TRUE;
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
        prefront_flag = TRUE;
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
        Control_AngCalculator(184.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        straight.v_now = 0;
        angle.v_now = 0;
        error_ang_sum = 0.0f;
        error_ang_old = 0.0f;
        HAL_Delay(500);

        Control_StrCalculator(52.0f, 0, 100.0f, 0, 1000.0f, -1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        straight.v_now = 0;
        angle.v_now = 0;
        error_sum = 0.0f;
        error_old = 0.0f;
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
        prefront_flag = TRUE;
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
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
}

void Motion_Straight(uint16_t step)
{
    if (step < 2)
    {
        //if (prefront_flag)
        //{
            Control_WallEdgeAble(0b11);
        //}
        Control_StrCalculator(180.0f - enc.offset, VELO_S, VELO_S, VELO_S, ACCEL_S, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    else
    {
        step--;
        float velo = VELO_S + (float)(120 * step);
        if (velo > 2400)
        {
            velo = 2400;
        }
        Control_StrCalculator(180.0f * (float)step - enc.offset, VELO_S, velo, VELO_S, ACCEL_S, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        Control_WallEdgeAble(0b11);
        Control_StrCalculator(180.0f, VELO_S, VELO_S, VELO_S, ACCEL_S, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    enc.offset = 0;
    //prefront_flag = TRUE;
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
    Control_AngCalculator(184.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0.0f;
    angle.v_now = 0.0f;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);

    Control_StrCalculator(60.0f, 0, 100, 0, 1000, -1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    error_sum = 0.0f;
    error_old = 0.0f;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);

    Control_WallEdgeAble(0b11);
    Control_StrCalculator(130.0f, 0, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
    //prefront_flag = TRUE;
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
    Control_AngCalculator(184.0f, 0.0f, VELO_ANG_S, 0.0f, ACCEL_ANG_S, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);

    Control_StrCalculator(52.0f, 0, 100, 0, 1000, -1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    error_sum = 0.0f;
    error_old = 0.0f;
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    HAL_Delay(500);
    Control_WallEdgeAble(0b11);
    Control_StrCalculator(130.0f, 0, VELO_S, VELO_S, ACCEL_S, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
    prefront_flag = TRUE;
}

void Motion_LeftTurn(void)
{
    //offset in
    if (sen_front.is_wall == TRUE)
    {
        /*while (sen_front.now < 122)
            ;*/
        Control_WallEdgeAble(0b10000);
        Control_StrCalculator(90.0f - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    else
    {
        Control_StrCalculator(35.0f - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    //turn
    Control_StrCalculator(300.0f, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(90.0f, 0, 800.0, 0, 20000.0, 1); //90.125f
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(44.0f, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
    prefront_flag = FALSE;
}

void Motion_RightTurn(void)
{
    //offset in
    if (sen_front.is_wall == TRUE)
    {
        /*
        while (sen_front.now < 122)
            ;*/
        Control_WallEdgeAble(0b10000);
        Control_StrCalculator(90.0f - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    else
    {
        Control_StrCalculator(34.0f - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    //turn
    Control_StrCalculator(300.0f, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(90.0f, 0, 800.0, 0, 20000.0, -1); //90.125f
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(46.0f, VELO_S, VELO_S, VELO_S, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
    prefront_flag = FALSE;
}

void Motion_Switch(uint8_t motion)
{
    switch (motion & 0xf)
    {
    case FRONT:
        Motion_Straight(motion >> 4);
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

void Motion_FastStart(uint8_t step, float velo_end)
{
    diagonal_step = 90.0f * (float)sqrt(2);
    Control_ResetVelo();
    float velo = VELO_F;
    if (step > 1)
    {
        velo = VELO_F + gain_velo * (float)(step - 1);
    }
    if (velo > 2400)
    {
        velo = 2400;
    }
    Control_StrCalculator(40.0f + 90.0f * (float)step, 0.0f, velo, velo_end, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_FastStraight(uint8_t step, float v_start, float v_end)
{
    float velo = VELO_F + gain_velo * (float)step;
    if (velo > 2400)
    {
        velo = 2400;
    }
    Control_StrCalculator(90.0f * (float)step - enc.offset, v_start, velo, v_end, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_Adjust(uint16_t step, float velo_s)
{
    if (step == 1)
    {
        Control_StrCalculator(110.0f - enc.offset, velo_s, VELO_F, VELO_F, ACCEL_F, 1); //110
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    else
    {
        Control_StrCalculator(180.0f - enc.offset, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_Adjust_L(uint16_t step, float velo_s)
{
    if (step == 1)
    {
        Control_WallEdgeAble(0b10);
        Control_StrCalculator(90.0f, velo_s, VELO_F, VELO_F, ACCEL_F, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    else
    {
        Control_WallEdgeAble(0b10);
        Control_StrCalculator(90.0f, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        Control_StrCalculator(90.0f, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_Adjust_R(uint16_t step, float velo_s)
{
    if (step == 1)
    {
        Control_WallEdgeAble(0b01);
        Control_StrCalculator(90.0f, velo_s, VELO_F, VELO_F, ACCEL_F, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    else
    {
        Control_WallEdgeAble(0b01);
        Control_StrCalculator(90.0f, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        Control_StrCalculator(90.0f, VELO_S, VELO_S, VELO_S, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_FastGoal(uint8_t step, float v_start)
{
    float velo = VELO_F;
    if (step > 1)
    {
        velo = VELO_F + gain_velo * (step - 1);
    }
    if (velo > 2400)
    {
        velo = 2400;
    }
    Control_StrCalculator(90.0f + 90.0f * (float)step - enc.offset, v_start, velo, 0, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    HAL_Delay(800);
}

void Motion_Diagonal(uint8_t step)
{
    Control_StrCalculator(diagonal_step * (float)step - enc.offset, VELO_F, VELO_F, VELO_F, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_DiagonalLeft(uint8_t step)
{
    if (step > 1)
    {
        step--;
        float velo = VELO_F + gain_velo * 0.5f * step;
        if (velo > 2400.0f)
        {
            velo = 2400.0f;
        }
        Control_StrCalculator(diagonal_step * (float)step - enc.offset, VELO_F, velo, VELO_F, 6000.0f, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    Control_WallEdgeAble(0b1000);
    Control_StrCalculator(diagonal_step, VELO_F, VELO_F, VELO_F, 6000.0f, 1); //26
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_DiagonalRight(uint8_t step)
{
    if (step > 1)
    {
        step--;
        float velo = VELO_F + gain_velo * 0.5f * (float)step;
        if (velo > 2400.0f)
        {
            velo = 2400.0f;
        }
        Control_StrCalculator(diagonal_step * (float)step - enc.offset, VELO_F, velo, VELO_F, 6000.0f, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    Control_WallEdgeAble(0b0100);
    Control_StrCalculator(diagonal_step, VELO_F, VELO_F, VELO_F, 6000.0f, 1); //20
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_DiagonalStart(void)
{
    Control_ResetVelo();
    diagonal_step = 90.0f * sqrt(2);
    Control_StrCalculator(diagonal_step * 2.0f, 0, VELO_F, VELO_F, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_DiagonalStop(void)
{
    Control_StrCalculator(diagonal_step * 2.0f - enc.offset, VELO_F, VELO_F, 0, ACCEL_F, 1);
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

void Motion_InLeft45Turn(void)
{
    //offset in
    Control_StrCalculator(41.0f - enc.offset, VELO_F, VELO_F, VELO_F, ACCEL_F, 1); //
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(800.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(45.0f, 0, 600.0f, 0, 20000.0f, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    control_wall_flag = 3;
    Control_WallEdgeAble(0b0100);
    Control_StrCalculator(90.0f, VELO_F, VELO_F, VELO_F, 1, 1); //
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_InRight45Turn(void)
{
    //offset in
    Control_StrCalculator(43.0f - enc.offset, VELO_F, VELO_F, VELO_F, ACCEL_F, 1); //43
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(800.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(45.0f, 0, 600.0f, 0, 20000.0f, -1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    control_wall_flag = 3;
    Control_WallEdgeAble(0b1000);
    Control_StrCalculator(93.0f, VELO_F, VELO_F, VELO_F, 1, 1); //17
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_OutLeft45Turn(float v_end)
{
    //offset in
    Control_StrCalculator(79.0f - enc.offset, VELO_F, VELO_F, VELO_F, 1, 1); //77
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(800.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(45.5f, 0, 600.0f, 0, 20000.0f, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    control_wall_flag = 2;
    //offset out
    if (v_end == 0.0f)
    {
        Control_StrCalculator(50.0f, VELO_F, VELO_F, 0.0f, 10000.0f, 1); //110
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
        straight.v_now = 0;
        angle.v_now = 0;
        HAL_Delay(800);
    }
    else
    {
        Control_WallEdgeAble(0b11);
        Control_StrCalculator(50.0f, VELO_F, VELO_F, v_end, 1, 1); //110
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
    }
}

void Motion_OutRight45Turn(float v_end)
{
    //offset in
    Control_StrCalculator(78.0f - enc.offset, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(800.0f, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(45.5f, 0, 600.0f, 0, 20000.0f, -1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    control_wall_flag = 2;
    //offset out
    if (v_end == 0.0f)
    {
        Control_StrCalculator(60.0f, VELO_F, VELO_F, 0.0f, 10000.0f, 1); //54
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
        straight.v_now = 0;
        angle.v_now = 0;
        HAL_Delay(800);
    }
    else
    {
        Control_WallEdgeAble(0b11);
        Control_StrCalculator(60.0f, VELO_F, VELO_F, v_end, 1, 1); //54
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
    }
}

void Motion_Left90Turn(float v_end)
{
    //offset in
    Control_StrCalculator(29.0f - enc.offset, VELO_F, VELO_F, VELO_F, ACCEL_F, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(VELO_F, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(90.0f, 0, 400.0f, 0, 5000.0f, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    error_ang_sum = 0.0f;
    error_ang_old = 0.0f;
    //offset out
    if (v_end == 0.0f)
    {
        Control_StrCalculator(40.0f, VELO_F, VELO_F, 0.0f, 10000.0f, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
        straight.v_now = 0;
        angle.v_now = 0;
        HAL_Delay(800);
    }
    else
    {
        Control_WallEdgeAble(0b11);
        Control_StrCalculator(40.0f, VELO_F, VELO_F, v_end, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
    }
}

void Motion_Right90Turn(float v_end)
{
    //offset in
    Control_StrCalculator(28.5f - enc.offset, VELO_F, VELO_F, VELO_F, ACCEL_F, 1); //29
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(VELO_F, VELO_F, VELO_F, VELO_F, 1, 1);
    Control_AngCalculator(90.5f, 0, 400.0f, 0, 5000.0f, -1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    if (v_end == 0.0f)
    {
        Control_StrCalculator(38.0f, VELO_F, VELO_F, 0.0f, 10000.0f, 1); //41
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
        straight.v_now = 0;
        angle.v_now = 0;
        HAL_Delay(800);
    }
    else
    {
        Control_WallEdgeAble(0b11);
        Control_StrCalculator(38.0f, VELO_F, VELO_F, v_end, 1, 1); //41
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
    }
}

void Turn(float offset_in, float offset_out, uint8_t d_flag, float velo, float v_end, float deg, float velo_ang, float accel_ang, uint8_t dir, uint8_t wall_flag)
{
    //offset in
    Control_StrCalculator(offset_in - enc.offset, velo, velo, velo, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(800.0f, velo, velo, velo, 1.0f, 1.0f); //600
    Control_AngCalculator(deg, 0, velo_ang, 0, accel_ang, dir);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    control_wall_flag = wall_flag;
    if (d_flag == 0)
    {
        if (v_end == 0.0f)
        {
            Control_StrCalculator(offset_out, velo, velo, 0.0f, 8000.0f, 1);
            Control_AngCalculator(0, 0, 0, 0, 1, 0);
            motion_end_flag = FALSE;
            while (motion_end_flag == FALSE)
            {
            }
            straight.v_now = 0;
            angle.v_now = 0;
            HAL_Delay(800);
        }
        else
        {
            Control_WallEdgeAble(0b11);
            Control_StrCalculator(offset_out, velo, velo, v_end, 1, 1);
            Control_AngCalculator(0, 0, 0, 0, 1, 0);
            motion_end_flag = FALSE;
            while (motion_end_flag == FALSE)
            {
            }
            enc.offset = 0.0f;
        }
    }
    else
    {
        if (dir == 1)
        {
            Control_WallEdgeAble(0b0100);
        }
        else
        {
            Control_WallEdgeAble(0b1000);
        }
        Control_StrCalculator(offset_out, velo, velo, v_end, 16000.0f, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        enc.offset = 0.0f;
    }
}

void Motion_FastLeftTurn(uint8_t type, float v_end)
{
    switch (type)
    {
    case SEARCH:
        Motion_LeftTurn();
        break;
    case T_90:
        Motion_Left90Turn(v_end);
        break;
    case T_180:
        Turn(36.0f, 43.0f, 0, 800.0f, v_end, 180.0f, 520.0f, 5000.0f, 1, 2); //0ut = 53
        break;
    case T_45IN:
        Motion_InLeft45Turn();
        break;
    case T_45OUT:
        control_wall_flag = 0;
        Motion_OutLeft45Turn(v_end);
        break;
    case T_135IN:
        Turn(49.0f, 46.0f, 1, 800.0f, 800.0f, 135.0f, 600.0f, 8000.0f, 1, 3);
        break;
    case T_135OUT:
        control_wall_flag = 0;
        Turn(28.0f, 57.5f, 0, 800.0f, v_end, 135.0f, 600.0f, 8000.0f, 1, 2); //in=34 out=67
        break;
    case T_V90:
        control_wall_flag = 0;
        Turn(46.0f, 67.0f, 1, 800.0f, 800.0f, 90.25f, 800.0f, 20000.0f, 1, 3); //in=67 out=23
        break;
    }
}

void Motion_FastRightTurn(uint8_t type, float v_end)
{
    switch (type)
    {
    case SEARCH:
        Motion_RightTurn();
        break;
    case T_90:
        Motion_Right90Turn(v_end);
        break;
    case T_180:
        Turn(30.0f, 48.0f, 0, 800.0f, v_end, 180.0f, 530.0f, 5000.0f, -1, 2); //out=50
        break;
    case T_45IN:
        Motion_InRight45Turn();
        break;
    case T_45OUT:
        control_wall_flag = 0;
        Motion_OutRight45Turn(v_end);
        break;
    case T_135IN:
        Turn(49.0f, 45.0f, 1, 800.0f, 800.0f, 135.0f, 600.0f, 8000.0f, -1, 3); //in:50 out=26
        break;
    case T_135OUT:
        control_wall_flag = 0;
        Turn(29.0f, 55.0f, 0, 800.0f, v_end, 135.0f, 600.0f, 8000.0f, -1, 2); //in=40.5 out = 66
        break;
    case T_V90:
        control_wall_flag = 0;
        Turn(52.0f, 68.0f, 1, 800.0f, 800.0f, 90.125f, 800.0f, 20000.0f, -1, 3); //in=63 out=25,23
        break;
    }
}

void Motion_FastestStart(uint8_t step, float velo_end)
{
    diagonal_step = 90.0f * sqrt(2);
    float velo = VELO_EST;
    if (step > 2)
    {
        velo = VELO_EST + gain_velo * (float)(step - 1);
    }
    if (velo > 4000.0f)
    {
        velo = 4000.0f;
    }
    Control_StrCalculator(40.0f + 90.0f * (float)step, 0.0f, velo, velo_end, ACCEL_EST, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_FastestStraight(uint8_t step, float v_start, float v_end)
{
    float velo = VELO_EST + gain_velo * (float)step;
    if (velo > 4000.0f)
    {
        velo = 4000.0f;
    }
    Control_StrCalculator(90.0f * (float)step - enc.offset, v_start, velo, v_end, ACCEL_EST, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_FastestGoal(uint8_t step, float v_start)
{
    float velo = VELO_EST;
    if (step > 2)
    {
        velo = VELO_EST + gain_velo * (step - 1);
    }
    if (velo > 4000.0f)
    {
        velo = 4000.0f;
    }
    Control_StrCalculator(90.0f + 90.0f * (float)step - enc.offset, v_start, velo, 0, ACCEL_EST, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    straight.v_now = 0;
    angle.v_now = 0;
    HAL_Delay(100);
}

void Motion_FastestAdjust_L(uint16_t step, float velo_s)
{
    if (step == 1)
    {
        Control_WallEdgeAble(0b0010);
        Control_StrCalculator(90.0f, velo_s, VELO_EST, VELO_EST, ACCEL_EST, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    else
    {
        Control_WallEdgeAble(0b0010);
        Control_StrCalculator(90.0f, velo_s, VELO_EST, VELO_EST, ACCEL_EST, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        Control_StrCalculator(90.0f, velo_s, VELO_EST, VELO_EST, ACCEL_EST, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_FastestAdjust_R(uint16_t step, float velo_s)
{
    if (step == 1)
    {
        Control_WallEdgeAble(0b0001);
        Control_StrCalculator(90.0f, VELO_EST, VELO_EST, VELO_EST, ACCEL_EST, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    else
    {
        Control_WallEdgeAble(0b0001);
        Control_StrCalculator(90.0f, VELO_EST, VELO_EST, VELO_EST, ACCEL_EST, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
        Control_StrCalculator(90.0f, VELO_EST, VELO_EST, VELO_EST, ACCEL_EST, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
    }
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Fastest_StrTurn(float offset_in, float offset_out, float velo, float v_end, float deg, float velo_ang, float accel_ang, uint8_t dir)
{
    //offset in
    Control_StrCalculator(offset_in - enc.offset, velo, velo, velo, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(1200.0f, velo, velo, velo, 1, 1);
    Control_AngCalculator(deg, 0, velo_ang, 0, accel_ang, dir);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_WallEdgeAble(0b0011);
    Control_StrCalculator(offset_out, velo, velo, v_end, 20000.0f, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Fastest_DgnInTurn(float offset_in, float offset_out, float velo, float v_end, float deg, float velo_ang, float accel_ang, uint8_t dir)
{
    //offset in
    Control_StrCalculator(offset_in - enc.offset, velo, velo, velo, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(1200.0f, velo, velo, velo, 1, 1);
    Control_AngCalculator(deg, 0, velo_ang, 0, accel_ang, dir);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    control_wall_flag = 0;
    if (dir == 1)
    {
        Control_WallEdgeAble(0b0100);
    }
    else if (dir == -1)
    {
        Control_WallEdgeAble(0b1000);
    }
    Control_StrCalculator(offset_out, VELO_EST, VELO_EST, VELO_EST, ACCEL_EST, 1); //26
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    control_wall_flag = 3;
    enc.offset = 0.0f;
}

void Fastest_DgnOutTurn(float offset_in, float offset_out, float velo, float v_end, float deg, float velo_ang, float accel_ang, uint8_t dir)
{
    //offset in
    control_wall_flag = 0;
    Control_StrCalculator(offset_in - enc.offset, velo, velo, velo, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(1200.0f, velo, velo, velo, 1, 1);
    Control_AngCalculator(deg, 0, velo_ang, 0, accel_ang, dir);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    control_wall_flag = 4;
    Control_WallEdgeAble(0b0011);
    Control_StrCalculator(offset_out, velo, velo, v_end, 20000.0f, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Fastest_DgnTurn(float offset_in, float offset_out, float velo, float v_end, float deg, float velo_ang, float accel_ang, uint8_t dir)
{
    //offset in
    control_wall_flag = 0;
    Control_StrCalculator(offset_in - enc.offset, velo, velo, velo, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //turn
    Control_StrCalculator(1200.0f, velo, velo, velo, 1, 1);
    Control_AngCalculator(deg, 0, velo_ang, 0, accel_ang, dir);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    if (dir == 1)
    {
        Control_WallEdgeAble(0b0100);
    }
    else if (dir == -1)
    {
        Control_WallEdgeAble(0b1000);
    }
    Control_StrCalculator(offset_out, VELO_EST, VELO_EST, VELO_EST, ACCEL_EST, 1); //26
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
    control_wall_flag = 3;
}

void Motion_FastestLeftSearch(void)
{
    //offset in
    if (sen_front.is_wall == TRUE)
    {
        while (sen_front.now < 115)
            ;
    }
    else
    {
        Control_StrCalculator(16.5 - enc.offset, 1200.0f, 1200.0f, 1200.0f, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    //turn
    Control_StrCalculator(1200.0f, 1200.0f, 1200.0f, 1200.0f, 1, 1);
    Control_AngCalculator(90.0f, 0, 1500.0f, 0, 50000.0f, 1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(52.5f, 1200.0f, 1200.0f, 1200.0f, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_FastestRightSearch(void)
{
    //offset in
    if (sen_front.is_wall == TRUE)
    {
        while (sen_front.now < 115)
            ;
    }
    else
    {
        Control_StrCalculator(16.0 - enc.offset, 1200.0f, 1200.0f, 1200.0f, 1, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    //turn
    Control_StrCalculator(1200.0f, 1200.0f, 1200.0f, 1200.0f, 1, 1);
    Control_AngCalculator(90.0f, 0, 1500.0f, 0, 50000.0f, -1);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    //offset out
    Control_StrCalculator(60.0f, 1200.0f, 1200.0f, 1200.0f, 1, 1);
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0.0f;
}

void Motion_FastestLeftTurn(uint8_t type, float v_end)
{
    //K=24000
    switch (type)
    {
    case SEARCH:
        Motion_FastestLeftSearch(); //16.5f, 55.0f, 1200.0f, 1200.0f, 90.0f, 1500.0f, 50000.0f, 1, 0
        break;
    case T_90:
        Fastest_StrTurn(36.0f, 57.0f, 1200.0f, v_end, 90.0f, 600.0f, 20000.0f, 1);
        break;
    case T_180:
        Fastest_StrTurn(50.0f, 70.0f, 1200.0f, v_end, 180.0f, 760.0f, 20000.0f, 1);
        break;
    case T_45IN:
        Fastest_DgnInTurn(21.0f, 99.0f, 1200.0f, 1200.0, 45.0f, 800.0f, 40000.0f, 1);
        break;
    case T_45OUT:
        Fastest_DgnOutTurn(64.0f, 55.0f, 1200.0f, v_end, 45.0f, 800.0f, 40000.0f, 1);
        break;
    case T_135IN:
        Fastest_DgnInTurn(28.0f, 44.0f, 1200.0f, 1200.0f, 135.0f, 800.0f, 20000.0f, 1);
        break;
    case T_135OUT:
        Fastest_DgnOutTurn(10.0f, 54.0f, 1200.0f, v_end, 135.0f, 800.0f, 20000.0f, 1);
        break;
    case T_V90:
        Fastest_DgnTurn(29.0f, 60.0f, 1200.0f, 1200.0f, 90.0f, 1000.0f, 40000.0f, 1);
        break;
    }
}

void Motion_FastestRightTurn(uint8_t type, float v_end)
{
    switch (type)
    {
    case SEARCH:
        Motion_FastestRightSearch(); //16.0f, 63.5f, 1200.0f, 1200.0f, 90.0f, 1500.0f, 50000.0f, -1, 0
        break;
    case T_90:
        Fastest_StrTurn(38.0f, 62.0f, 1200.0f, v_end, 90.0f, 600.0f, 20000.0f, -1);
        break;
    case T_180:
        Fastest_StrTurn(50.0f, 70.0f, 1200.0f, v_end, 180.0f, 745.0f, 20000.0f, -1);
        break;
    case T_45IN:
        Fastest_DgnInTurn(30.0f, 89.0f, 1200.0f, 1200.0f, 45.0f, 800.0f, 40000.0f, -1);
        break;
    case T_45OUT:
        Fastest_DgnOutTurn(74.0f, 52.0f, 1200.0f, v_end, 45.0f, 800.0f, 40000.0f, -1);
        break;
    case T_135IN:
        Fastest_DgnInTurn(28.0f, 42.0f, 1200.0f, 1200.0f, 135.0f, 800.0f, 20000.0f, -1); //in:41 out:40
        break;
    case T_135OUT:
        Fastest_DgnOutTurn(28.0f, 65.0f, 1200.0f, v_end, 135.0f, 800.0f, 20000.0f, -1);
        break;
    case T_V90:
        Fastest_DgnTurn(30.0f, 62.0f, 1200.0f, 1200.0f, 90.0f, 1000.0f, 40000.0f, -1);
        break;
    }
}
void Motion_FastestDiagonalLeft(uint8_t step)
{
    if (step > 1)
    {
        step--;
        float velo = VELO_EST + gain_velo * step;
        if (velo > 2400.0f)
        {
            velo = 2400.0f;
        }
        Control_StrCalculator(diagonal_step * (float)step - enc.offset, VELO_EST, velo, VELO_EST, ACCEL_EST, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    Control_WallEdgeAble(0b1000);
    Control_StrCalculator(diagonal_step, VELO_EST, VELO_EST, VELO_EST, ACCEL_EST, 1); //26
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_FastestDiagonalRight(uint8_t step)
{
    if (step > 1)
    {
        step--;
        float velo = VELO_EST + gain_velo * (float)step;
        if (velo > 2400.0f)
        {
            velo = 2400.0f;
        }
        Control_StrCalculator(diagonal_step * (float)step - enc.offset, VELO_EST, velo, VELO_EST, ACCEL_EST, 1);
        Control_AngCalculator(0, 0, 0, 0, 1, 0);
        motion_end_flag = FALSE;
        while (motion_end_flag == FALSE)
        {
        }
    }
    Control_WallEdgeAble(0b0100);
    Control_StrCalculator(diagonal_step, VELO_EST, VELO_EST, VELO_EST, ACCEL_EST, 1); //20
    Control_AngCalculator(0, 0, 0, 0, 1, 0);
    motion_end_flag = FALSE;
    while (motion_end_flag == FALSE)
    {
    }
    enc.offset = 0;
}