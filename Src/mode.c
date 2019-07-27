#include "mode.h"
#include "spi.h"
#include "tim.h"
#include "adc.h"
#include "stdint.h"
#include "gpio.h"
#include "motion.h"
#include "control.h"
#include "Maze.h"
#include "Map.h"
#include "flash.h"
#include "MazeCon.h"

#define TRUE 1
#define FALSE 0

extern volatile accel_t accel;
extern volatile float gyro_y;
extern gyro_t gyro_z;
extern sensor_t sen_l;
extern sensor_t sen_fl;
extern sensor_t sen_front;
extern sensor_t sen_fr;
extern sensor_t sen_r;
extern enc_t enc;
extern volatile uint8_t motor_flag;
extern loger_t loger;
extern volatile uint8_t control_wall_flag;

unsigned char gx = 1, gy = 0;

//Prototype Function
void SensorCheck(void);
void SetRunMode(void);
void CheckStraight(uint8_t block);
void CheckTurn(void);
void PrintOutPut(void);
void AdjustStraight(void);
void AdjustTurn(void);
void SearchRun(void);
void Mode_FastRun(void);
void Circuit(uint8_t row, uint8_t colum, uint8_t num, uint8_t dir);

/****************************************************************************************
 * outline  : wright mode 
 * argument : mode nomber
 * return   : void
********************************************************************************************/
void Mode_Mouse(int8_t mode)
{
    switch (mode)
    {
    case 0:
        /*
        control_wall_flag = TRUE;
        CheckStraight(6);
        break;
        */
        SearchRun();
        //AdjustStraight();
        //CheckStraight(1);
        break;
    case 1:
        Mode_FastRun();
        //AdjustTurn();
        /*
        control_wall_flag = FALSE;
        CheckTurn();
        */
        break;
    case 2:
        /*
        SetRunMode();
        Motion_enkai();
        */
        control_wall_flag = FALSE;
        SetRunMode();
        Motion_Start();
        Motion_LeftTurn();
        Motion_End();
        motor_flag = FALSE;
        HAL_Delay(500);
        Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
        Tim_BuzzerPwm(HZ_C_H, 300);
        HAL_Delay(200);
        Tim_BuzzerPwm(HZ_NORMAL, 0);
        break;
    case 3:
        control_wall_flag = FALSE;
        SetRunMode();
        Motion_Start();
        Motion_RightTurn();
        Motion_End();
        motor_flag = FALSE;
        HAL_Delay(500);
        Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
        Tim_BuzzerPwm(HZ_C_H, 300);
        HAL_Delay(200);
        Tim_BuzzerPwm(HZ_NORMAL, 0);
        break;

    //adjust
    case 4:
        //SensorCheck();
        control_wall_flag = TRUE;
        CheckStraight(6);
        break;
    case 5:
        control_wall_flag = FALSE;
        SetRunMode();
        Circuit(1, 1, 3, 1);
        break;
    case 6:
        control_wall_flag = FALSE;
        SetRunMode();
        Circuit(1, 1, 3, -1);
        break;
    case 7:
        //PrintOutPut();
        Maze_Printf();
        break;
    default:
        break;
    }
}

char Mode_Select(void)
{
    int8_t mode = 0;
    uint8_t state = 0;
    Control_ResetVelo();

    while (1)
    {
        if (gyro_y > 300.0f || gyro_y < -300.0f)
        {
            if (gyro_y < -300.0f)
            {
                mode++;
            }
            else if (gyro_y > 300.0f)
            {
                mode--;
            }
            if (mode > 3)
            {
                mode = 0;
            }
            else if (mode < 0)
            {
                mode = 3;
            }
            Tim_BuzzerPwm(170 - 10 * mode, 300);
            HAL_Delay(100);
            Tim_BuzzerPwm(HZ_NORMAL, 0);
            HAL_Delay(500);
        }
        if (enc.offset > 20)
        {
            state = 4 - state;
            enc.offset = 0;
        }
        if (accel.z > 25.0f)
        {
            Tim_BuzzerPwm(HZ_C_H, 300);
            HAL_Delay(100);
            Tim_BuzzerPwm(HZ_NORMAL, 0);
            HAL_Delay(500);
            break;
        }
        if (state == 0)
        {
            Gpio_FullColor(CYAN);
        }
        else
        {
            Gpio_FullColor(YELLOW);
        }
        Gpio_SideLed(mode);
        printf("%d\r", mode + state);
    }
    return mode + state;
}

void SensorCheck(void)
{
    Adc_IrSensorStart();

    while (1)
    {
        if (sen_r.is_wall == TRUE)
        {
            HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
        }
        if (sen_l.is_wall == TRUE)
        {
            HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
        }
        if (sen_front.is_wall == TRUE)
        {
            Gpio_FullColor(WHITE);
        }
        else
        {
            Gpio_FullColor(DARK);
        }
        printf("l:%d fl:%d f:%d fr:%d r:%d \r", sen_l.now, sen_fl.now, sen_front.now, sen_fr.now, sen_r.now);

        if (accel.x > 60)
        {
            Adc_IrSensorFinish();
            Tim_BuzzerPwm(HZ_C_H, 300);
            HAL_Delay(300);
            Tim_BuzzerPwm(HZ_NORMAL, 0);
            HAL_Delay(500);
            break;
        }
    }
}

void SetRunMode(void)
{
    Adc_IrSensorStart();
    while (sen_front.is_wall != TRUE)
    {
    }
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(100);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
    HAL_Delay(5000);
    Spi_GyroReset();
    HAL_Delay(1200);
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(100);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
    motor_flag = TRUE;
}

void Get_WallData(uint8_t *n_wall, uint8_t *e_wall, uint8_t *w_wall, uint8_t *s_wall, uint8_t x, uint8_t y, uint8_t direction)
{
    // 方向別に壁の状態を取得
    switch (direction)
    {
    case NORTH:
        *n_wall = sen_front.is_wall;
        *e_wall = sen_r.is_wall;
        *w_wall = sen_l.is_wall;
        *s_wall = 0;
        break;
    case EAST:
        *e_wall = sen_front.is_wall;
        *s_wall = sen_r.is_wall;
        *n_wall = sen_l.is_wall;
        *w_wall = 0;
        break;
    case SOUTH:
        *s_wall = sen_front.is_wall;
        *w_wall = sen_r.is_wall;
        *e_wall = sen_l.is_wall;
        *n_wall = 0;
        break;
    case WEST:
        *w_wall = sen_front.is_wall;
        *n_wall = sen_r.is_wall;
        *s_wall = sen_l.is_wall;
        *e_wall = 0;
        break;
    default:
        break;
    }
}

void SearchRun(void)
{
    //壁情報を持った構造体を定義
    wallData_t wall_data;
    wallData_t wall_data_fast;

    //ポジションを定義、初期化
    pos_t mypos;
    mypos.dir = 0;
    mypos.x = 0;
    mypos.y = 1;

    //マップを扱うクラスを定義、壁情報を初期化
    Map_Init(&wall_data);
    Map_InitFast(&wall_data_fast);

    //歩数マップの作製
    unsigned char nextdir = FRONT;
    //最初の区画は進んでいるものとする
    unsigned char flag_goal = 0;
    //enable control side wall
    control_wall_flag = TRUE;
    SetRunMode();
    Motion_Start();
    while (1)
    {
        unsigned char n_wall = 0, e_wall = 0, w_wall = 0, s_wall = 0;
        // 現在の座標の壁情報を読み込み、壁を探索データに追加する。
        Get_WallData(&n_wall, &e_wall, &w_wall, &s_wall, mypos.x, mypos.y, mypos.dir);

        Map_addWall(&wall_data, &mypos, n_wall, e_wall, w_wall, s_wall);
        Map_addWall(&wall_data_fast, &mypos, n_wall, e_wall, w_wall, s_wall);

        Maze_UpdateStepMap(&flag_goal, gx, gy, &wall_data);
        nextdir = Maze_GetNextMotion(&mypos, &wall_data);
        Maze_UpdatePosition(&nextdir, &mypos);
        Motion_Switch(nextdir);
        if (mypos.x == gx && mypos.y == gy)
        {
            if (flag_goal == 0)
            {
                Get_WallData(&n_wall, &e_wall, &w_wall, &s_wall, mypos.x, mypos.y, mypos.dir);
                Map_addWall(&wall_data, &mypos, n_wall, e_wall, w_wall, s_wall);
                Map_addWall(&wall_data_fast, &mypos, n_wall, e_wall, w_wall, s_wall);
                flag_goal = 1;
                Motion_End();
                Flash_Write(start_address, (uint8_t *)&wall_data_fast, sizeof(wallData_t));
                Tim_BuzzerPwm(HZ_A, 300);
                HAL_Delay(200);
                Tim_BuzzerPwm(HZ_NORMAL, 0);
                if (sen_front.is_wall == FALSE)
                {
                    nextdir = FRONT;
                    Maze_UpdatePosition(&nextdir, &mypos);
                    Motion_Restart(FALSE);
                }
                else
                {
                    nextdir = PIVO_REAR;
                    Maze_UpdatePosition(&nextdir, &mypos);
                    Motion_Restart(TRUE);
                }
            }
        }
        if (flag_goal == 2)
        {

            Motion_End();
            motor_flag = FALSE;
            Flash_Write(start_address, (uint8_t *)&wall_data_fast, sizeof(wallData_t));
            Tim_BuzzerPwm(HZ_A, 300);
            HAL_Delay(200);
            Tim_BuzzerPwm(HZ_NORMAL, 0);
            break;
        }
    }
}

void Mode_FastRun(void)
{
    wallData_t wallData;
    uint8_t motion[255];
    uint8_t tail = 0;
    uint8_t head = 0;
    uint8_t goal_flag = 0;
    Flash_Load(start_address, (uint8_t *)&wallData, sizeof(wallData_t));
    Maze_UpdateStepMap(&goal_flag, gx, gy, &wallData);
    Plan_Root(motion, &wallData, &tail);
    Plan_Compress(motion, &head, &tail);
    control_wall_flag = TRUE;
    SetRunMode();
    while (head != tail)
    {
        switch (motion[head] >> 4)
        {
        case START:
            Motion_FastStart(motion[head] & 0b1111);
            //printf("\r\nSTART:%d\r\n",motion[head]&0b1111);
            break;
        case LEFT:
            Motion_LeftTurn();
            //printf("LEFT:%d\n\r",motion[head]&0b1111);
            break;
        case FRONT:
            Motion_FastStraight(motion[head] & 0b1111);
            //printf("FRONT:%d\n\r",motion[head]&0b1111);
            break;
        case RIGHT:
            Motion_RightTurn();
            //printf("RIGHT\n\r");
            break;
        case GOAL:
            Motion_FastGoal(motion[head] & 0b1111);
            //printf("GOAL:%d\n\r",motion[head]&0b1111);
            break;
        default:
            break;
        }
        head++;
    }
    motor_flag = FALSE;
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void CheckStraight(uint8_t block)
{
    SetRunMode();
    Motion_Start();
    for (uint8_t i = 0; i < block; i++)
    {
        Motion_Straight();
    }
    Motion_End();
    motor_flag = FALSE;
    loger.velo[loger.cnt] = enc.distance;
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void CheckTurn(void)
{
    SetRunMode();
    gyro_z.degree = 0;
    Motion_TestTurn();
    motor_flag = FALSE;
    loger.velo_ang[loger.cnt] = (int16_t)gyro_z.degree;
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void PrintOutPut(void)
{
    Flash_Load(start_address, (uint8_t *)&loger, sizeof(loger_t));
    for (uint16_t i = 0; i <= loger.cnt; i++)
    {
        //printf("%d\r\n",loger.velo[i]);
        //printf("%d\r\n", loger.velo_ang[i]);
        printf("%d\t%d\t%d\t%d\r\n", loger.target_velo[i], loger.velo[i], loger.target_velo_ang[i], loger.velo_ang[i]);
    }
}

void AdjustStraight(void)
{
    SetRunMode();
    Tim_MotorPwm(96, 96);
    while (motor_flag == TRUE)
    {
    }
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void AdjustTurn(void)
{
    SetRunMode();
    Tim_MotorPwm(-180, 180);
    while (motor_flag == TRUE)
    {
    }
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void Circuit(uint8_t row, uint8_t colum, uint8_t num, uint8_t dir)
{
    Motion_Start();
    for (uint8_t i = 0; i < num; i++)
    {
        if (row > 0)
        {
            Motion_FastStraight(row - 1);
        }

        if (dir == 1)
        {
            Motion_LeftTurn();
        }
        else
        {
            Motion_RightTurn();
        }

        if (colum > 0)
        {
            Motion_FastStraight(colum - 1);
        }

        if (dir == 1)
        {
            Motion_LeftTurn();
        }
        else
        {
            Motion_RightTurn();
        }

        if (row > 0)
        {
            Motion_FastStraight(row - 1);
        }

        if (dir == 1)
        {
            Motion_LeftTurn();
        }
        else
        {
            Motion_RightTurn();
        }

        if (colum > 0)
        {
            Motion_FastStraight(colum - 1);
        }
        if (dir == 1)
        {
            Motion_LeftTurn();
        }
        else
        {
            Motion_RightTurn();
        }
    }
    Motion_End();
    motor_flag = FALSE;
}