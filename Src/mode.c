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
extern uint8_t counter_s;

unsigned char gx = 1, gy = 0;
walledge_t walledge_search={-8,-8,0,0,107.0f,109.0f,0,0};
walledge_t walledge_800 = {-10,-10,-10,-10,13.0f,13.0f,20.0f,21.0f};
walledge_t walledge_1200 = {-11,-19,-10,-10,11.0f,19.0f,20.0f,20.0f};

//Prototype Function
void SensorCheck(void);
void SetRunMode(uint8_t fan_flag);
void CheckStraight(uint8_t block);
void CheckFastStraight(uint8_t block);
void CheckFastestStraight(uint8_t block);
void CheckDiagonal(uint8_t block, int8_t dir);
void CheckTurn(uint8_t num);
void PrintOutPut(void);
void PrintWallData(void);
void PrintMotion(uint16_t *motion, uint32_t *velocity, uint8_t tail);
void AdjustStraight(void);
void AdjustTurn(void);
void SearchRun(void);
void Mode_FastRun(uint8_t diagonal_flag, float gain, uint8_t w_str, uint8_t w_turn);
void Mode_FastestRun(uint8_t diagonal_flag, float gain, uint8_t w_str, uint8_t w_turn);
void Circuit(uint8_t row, uint8_t colum, uint8_t num, uint8_t dir);
void TestComb(uint8_t num, uint8_t dir);
void TestSteps(uint8_t num);
void Turn45_Test(uint8_t in_dir, uint8_t out_dir, uint8_t step);
void Turn135_Test(uint8_t in_dir, uint8_t out_dir, uint8_t step);
void TurnBig90_Test(int8_t dir);
void TurnBig180_Test(int8_t dir);
void TurnV90_Test(int8_t dir);
void FastestTurnTest(uint8_t dir, uint8_t deg);

/****************************************************************************************
 * outline  : wright mode 
 * argument : mode nomber
 * return   : void
********************************************************************************************/
void Mode_Mouse(int8_t mode)
{
    switch (mode)
    {
    //WHITE
    case 0:
        SearchRun();
        break;
    case 1:
        //Tim_FanPwm(4);
        //Mode_FastRun(FALSE, 160.0f, 1, 3);
        Mode_FastRun(TRUE, 0.0f, 7, 5);
        break;
    case 2:
        /*
        SetRunMode();
        control_wall_flag = FALSE;
        Motion_enkai();
        */
        Mode_FastRun(TRUE, 120.0f, 7, 5);
        //Mode_FastestRun(FALSE, 100.0f, 1, 3);
        break;
    case 3:
        Mode_FastRun(FALSE, 120.0f, 1, 3);
        //Mode_FastRun(TRUE, 160.0f, 7, 5);
        //SensorCheck();
        break;
    //CYAN
    case 4:
        PrintOutPut();
        break;
    case 5:
        PrintWallData();
        break;
    case 6: //straight
        /*
        control_wall_flag = 0;
        CheckDiagonal(4, 1);
        */
        control_wall_flag = 1;
        CheckStraight(0);
        /*
       control_wall_flag = 2;
       CheckFastStraight(6);
       */
        /*
        control_wall_flag = 0;
        CheckFastestStraight(4);
        */
        break;
    case 7: //turn
            //control_wall_flag = 3;
            //CheckDiagonal(4, 1);
        control_wall_flag = 0;
        CheckTurn(1);
        break;
    //YELLOW
    case 8: //left turn
        control_wall_flag = FALSE;
        SetRunMode(0);
        Motion_Start();
        Motion_LeftTurn();
        Motion_End();
        motor_flag = FALSE;
        HAL_Delay(500);
        Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
        Tim_BuzzerPwm(HZ_C_H, 300);
        HAL_Delay(200);
        Tim_BuzzerPwm(HZ_NORMAL, 0);
        //control_wall_flag = FALSE;
        //FastestTurnTest(1, SEARCH);
        break;
    case 9: //right turn
        control_wall_flag = FALSE;
        SetRunMode(0);
        Motion_Start();
        Motion_RightTurn();
        Motion_End();
        motor_flag = FALSE;
        HAL_Delay(500);
        Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
        Tim_BuzzerPwm(HZ_C_H, 300);
        HAL_Delay(200);
        Tim_BuzzerPwm(HZ_NORMAL, 0);
        //control_wall_flag = FALSE;
        //FastestTurnTest(-1, SEARCH);
        break;
    case 10: //circuit left
        control_wall_flag = FALSE;
        SetRunMode(0);
        //Circuit(2, 2, 3, 1);
        TestComb(6, LEFT);
        break;
    case 11: //circuit right
        control_wall_flag = FALSE;
        SetRunMode(0);
        //Circuit(2, 2, 3, -1);
        TestComb(6, RIGHT);
        break;
    //GREEN
    case 12: //left quarter turn
        control_wall_flag = 3;
        //Turn45_Test(FRONT, LEFT, 1);
        Turn135_Test(FRONT, LEFT, 0);
        break;
    case 13: //right quarter turn
        control_wall_flag = 3;
        //Turn45_Test(FRONT, RIGHT, 1);
        Turn135_Test(FRONT, RIGHT, 0);
        break;
    case 14:
        control_wall_flag = 2;
        //Turn45_Test(LEFT, FRONT, 1);
        Turn135_Test(LEFT, FRONT, 1);
        break;
    case 15:
        control_wall_flag = 2;
        //Turn45_Test(RIGHT, FRONT, 1);
        Turn135_Test(RIGHT, FRONT, 1);
        break;
    //MAGENTA
    case 16:
        Mode_FastestRun(TRUE, 0.0f, 7, 5);
        break;
    case 17:
        Mode_FastestRun(TRUE, 100.0f, 7, 5);
        break;
    case 18:
        Mode_FastestRun(TRUE, 180.0f, 7, 5);
        break;
    case 19:
        control_wall_flag = 0;
        //TurnBig180_Test(RIGHT);
        //TurnV90_Test(RIGHT);
        FastestTurnTest(-1, T_180);
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
    accel.z = 0;

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
            if (state == 16)
            {
                state = 0;
            }
            else
            {
                state += 4;
            }
            enc.offset = 0;
        }
        else if (enc.offset < -20)
        {
            if (state == 0)
            {
                state = 16;
            }
            else
            {
                state -= 4;
            }
            enc.offset = 0;
        }
        if (accel.z > 30.0f)
        {
            Tim_BuzzerPwm(HZ_C_H, 300);
            HAL_Delay(100);
            Tim_BuzzerPwm(HZ_NORMAL, 0);
            HAL_Delay(500);
            break;
        }
        Gpio_FullColor(state % 7);
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
        printf("l:%d fl:%d f:%d fr:%d r:%d \r\n", sen_l.now, sen_fl.now, sen_front.now, sen_fr.now, sen_r.now);

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

void SetRunMode(uint8_t fan_flag)
{
    Adc_IrSensorStart();
    while (sen_front.is_wall != TRUE)
    {
    }
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(100);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
    Gpio_FullColor(DARK);
    Gpio_SideLed(0);
    HAL_Delay(5000);
    Spi_GyroReset();
    HAL_Delay(1200);
    if (fan_flag == 0)
    {
        Tim_BuzzerPwm(HZ_C_H, 300);
        HAL_Delay(100);
        Tim_BuzzerPwm(HZ_NORMAL, 0);
    }
    else
    {
        Control_SetupStraightPID(4.0f, 11.5f, 0);
        Control_SetupTurnPID(1.2f, 21.0f, 1.0f);
        //Tim_SetTireRadius(12.37f);
        Tim_FanPwm(3);
        HAL_Delay(3000);
    }
    Control_ResetVelo();
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
    uint16_t nextdir = FRONT;
    //最初の区画は進んでいるものとする
    unsigned char flag_goal = 0;
    //enable control side wall
    control_wall_flag = 1;
    //counter set
    counter_s = 0;
    Control_WallEdgeSet(walledge_search);
    SetRunMode(0);
    Motion_Start();
    while (1)
    {
        unsigned char n_wall = 0, e_wall = 0, w_wall = 0, s_wall = 0;
        // 現在の座標の壁情報を読み込み、壁を探索データに追加する。
        Get_WallData(&n_wall, &e_wall, &w_wall, &s_wall, mypos.x, mypos.y, mypos.dir);

        Map_addWall(&wall_data, &mypos, n_wall, e_wall, w_wall, s_wall);
        Map_DelWall(&wall_data_fast, &mypos, n_wall, e_wall, w_wall, s_wall);
        Map_AddUnknownWall(mypos.x,mypos.y,&wall_data,Maze_GetGoalSize(),gx,gy);

        Maze_UpdateStepMap(&flag_goal, gx, gy, &wall_data);

        if(flag_goal==0 && Maze_GetStep(mypos.x,mypos.y)==MAX_STEP){
            Maze_ResetWall(&wall_data,mypos);
        }
        
        nextdir = Maze_GetNextMotion(&mypos, &wall_data);
        nextdir = Maze_KnownStepAccel(&mypos, &wall_data, nextdir);
        //Maze_UpdatePosition(nextdir, &mypos);
        Motion_Switch(nextdir);
        if (mypos.x == gx && mypos.y == gy)
        {
            if (flag_goal == 0)
            {
                Get_WallData(&n_wall, &e_wall, &w_wall, &s_wall, mypos.x, mypos.y, mypos.dir);
                Map_addWall(&wall_data, &mypos, n_wall, e_wall, w_wall, s_wall);
                Map_DelWall(&wall_data_fast, &mypos, n_wall, e_wall, w_wall, s_wall);
                Motion_End();
                motor_flag = FALSE;
                flag_goal = 1;
                HAL_Delay(1000);
                Flash_Write(start_address, (uint8_t *)&wall_data_fast, sizeof(wallData_t));
                HAL_Delay(2000);
                Tim_BuzzerPwm(HZ_A, 300);
                HAL_Delay(200);
                Tim_BuzzerPwm(HZ_NORMAL, 0);
                if (sen_front.is_wall == FALSE)
                {
                    nextdir = FRONT;
                    Maze_UpdatePosition(nextdir, &mypos);
                    motor_flag = TRUE;
                    Motion_Restart(FALSE);
                }
                else
                {
                    nextdir = PIVO_REAR;
                    Maze_UpdatePosition(nextdir, &mypos);
                    motor_flag = TRUE;
                    Motion_Restart(TRUE);
                }
            }
        }
        if (counter_s > 150 && flag_goal == 1)
        {
            flag_goal = 2;
        }
        if (flag_goal == 2)
        {
            Motion_End();
            motor_flag = FALSE;
            HAL_Delay(1000);
            Flash_Write(start_address, (uint8_t *)&wall_data_fast, sizeof(wallData_t));
            HAL_Delay(2000);
            Tim_BuzzerPwm(HZ_A, 300);
            HAL_Delay(200);
            Tim_BuzzerPwm(HZ_NORMAL, 0);
            break;
        }
    }
}

void Mode_FastRun(uint8_t diagonal_flag, float gain, uint8_t w_str, uint8_t w_turn)
{
    wallData_t wallData;
    pos_t pos;
    uint16_t motion[255];
    uint32_t velocity[255];
    uint8_t tail = 0;
    uint8_t head = 0;
    Flash_Load(start_address, (uint8_t *)&wallData, sizeof(wallData_t));
    Maze_UpdateStepMapEx(&wallData, w_str, w_turn, gx, gy);
    Motion_MaxVeloSet(gain);
    Control_WallEdgeSet(walledge_800);
    pos.dir = NORTH;
    pos.x = 0;
    pos.y = 1;
    motion[tail] = START;
    tail++;
    motion[tail] = ((1 << 4) | FRONT);
    tail++;
    head += 2;
    while (1)
    {
        if (Maze_GetGoalSize() == 1)
        {
            if (pos.x == gx && pos.y == gy)
            {
                break;
            }
        }
        else
        {
            if (pos.x == gx && pos.y == gy)
            {
                break;
            }
            if (pos.x == (gx + 1) && pos.y == gy)
            {
                break;
            }
            if (pos.x == gx && pos.y == (gy + 1))
            {
                break;
            }
            if (pos.x == (gx + 1) && pos.y == (gy + 1))
            {
                break;
            }
        }
        uint16_t buff = Maze_GetNextMotionEx(&pos, &wallData);
        motion[tail] = buff;
        tail++;
        Maze_UpdatePosition(buff & 0xf, &pos);
    }
    motion[tail] = GOAL;
    tail++;
    head = 0;
    Maze_Compress(diagonal_flag, motion, velocity, &tail, VELO_S, VELO_F);

    control_wall_flag = 2;
    SetRunMode(0);
    while (head != tail)
    {
        switch (motion[head] & 0xf)
        {
        case START:
            Motion_FastStart(motion[head] >> 4, (float)(velocity[head] & 0xffff));
            break;
        case LEFT:
            Motion_FastLeftTurn(motion[head] >> 4, (float)(velocity[head] & 0xffff));
            break;
        case FRONT:
            Motion_FastStraight(motion[head] >> 4, (float)(velocity[head] >> 16), (float)(velocity[head] & 0xffff));
            break;
        case DIAGONAL_L:
            Motion_DiagonalLeft(motion[head] >> 4);
            break;
        case DIAGONAL_R:
            Motion_DiagonalRight(motion[head] >> 4);
            break;
        case RIGHT:
            Motion_FastRightTurn(motion[head] >> 4, (float)(velocity[head] & 0xffff));
            break;
        case GOAL:
            Motion_FastGoal(motion[head] >> 4, (float)(velocity[head] >> 16));
            break;
        case ADJUST:
            Motion_Adjust(motion[head] >> 4, (float)(velocity[head] >> 16));
            break;
        case ADJUST_L:
            Motion_Adjust_L(motion[head] >> 4, (float)(velocity[head] >> 16));
            break;
        case ADJUST_R:
            Motion_Adjust_R(motion[head] >> 4, (float)(velocity[head] >> 16));
            break;
        default:
            break;
        }
        head++;
    }
    //PrintMotion(motion, velocity, tail);
    HAL_Delay(500);
    motor_flag = FALSE;
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void Mode_FastestRun(uint8_t diagonal_flag, float gain, uint8_t w_str, uint8_t w_turn) //GainMax=193
{
    wallData_t wallData;
    pos_t pos;
    uint16_t motion[255];
    uint32_t velocity[255];
    uint8_t tail = 0;
    uint8_t head = 0;
    Flash_Load(start_address, (uint8_t *)&wallData, sizeof(wallData_t));
    Maze_UpdateStepMapEx(&wallData, w_str, w_turn, gx, gy);
    Motion_MaxVeloSet(gain);
    Control_WallEdgeSet(walledge_1200);
    pos.dir = NORTH;
    pos.x = 0;
    pos.y = 1;
    motion[tail] = START;
    tail++;
    motion[tail] = ((1 << 4) | FRONT);
    tail++;
    head += 2;
    while (1)
    {
        if (Maze_GetGoalSize() == 1)
        {
            if (pos.x == gx && pos.y == gy)
            {
                break;
            }
        }
        else
        {
            if (pos.x == gx && pos.y == gy)
            {
                break;
            }
            if (pos.x == (gx + 1) && pos.y == gy)
            {
                break;
            }
            if (pos.x == gx && pos.y == (gy + 1))
            {
                break;
            }
            if (pos.x == (gx + 1) && pos.y == (gy + 1))
            {
                break;
            }
        }
        uint16_t buff = Maze_GetNextMotionEx(&pos, &wallData);
        motion[tail] = buff;
        tail++;
        Maze_UpdatePosition(buff & 0xf, &pos);
    }
    motion[tail] = GOAL;
    tail++;
    head = 0;
    Maze_Compress(diagonal_flag, motion, velocity, &tail, VELO_EST, VELO_EST);

    control_wall_flag = 4;
    SetRunMode(1);
    while (head != tail)
    {
        switch (motion[head] & 0xf)
        {
        case START:
            Motion_FastestStart(motion[head] >> 4, (float)(velocity[head] & 0xffff));
            break;
        case LEFT:
            Motion_FastestLeftTurn(motion[head] >> 4, (float)(velocity[head] & 0xffff));
            break;
        case FRONT:
            Motion_FastestStraight(motion[head] >> 4, (float)(velocity[head] >> 16), (float)(velocity[head] & 0xffff));
            break;
        case DIAGONAL_L:
            Motion_FastestDiagonalLeft(motion[head] >> 4);
            break;
        case DIAGONAL_R:
            Motion_FastestDiagonalRight(motion[head] >> 4);
            break;
        case RIGHT:
            Motion_FastestRightTurn(motion[head] >> 4, (float)(velocity[head] & 0xffff));
            break;
        case GOAL:
            Motion_FastestGoal(motion[head] >> 4, (float)(velocity[head] >> 16));
            break;
        case ADJUST:
            Motion_Adjust(motion[head] >> 4, (float)(velocity[head] >> 16));
            break;
        case ADJUST_L:
            Motion_FastestAdjust_L(motion[head] >> 4, (float)(velocity[head] >> 16));
            break;
        case ADJUST_R:
            Motion_FastestAdjust_R(motion[head] >> 4, (float)(velocity[head] >> 16));
            break;
        default:
            break;
        }
        head++;
    }
    //PrintMotion(motion, velocity, tail);
    Tim_MotorBrake();
    HAL_Delay(500);
    motor_flag = FALSE;
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void CheckStraight(uint8_t block)
{
    SetRunMode(0);
    enc.distance = 0;
    Motion_Start();
    for (uint8_t i = 0; i < block; i++)
    {
        Motion_Straight(0);
    }
    Motion_End();
    motor_flag = FALSE;
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void CheckFastStraight(uint8_t block)
{
    SetRunMode(0);
    enc.distance = 0;
    Motion_FastStart(1, 800.0f);
    block--;
    for (uint8_t i = 0; i < block; i++)
    {
        Motion_FastStraight(2, 800.0f, 800.0f);
    }
    Motion_Adjust(1, 800.0f);
    Motion_FastGoal(1, 800.0f);
    motor_flag = FALSE;
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void CheckFastestStraight(uint8_t block)
{
    SetRunMode(1);
    enc.distance = 0;
    Motion_FastestStart(2 * (block - 1) + 1, VELO_EST);
    //Motion_FastestAdjust(1, VELO_EST);
    Motion_FastestGoal(1, VELO_EST);
    motor_flag = FALSE;
    Tim_FanPwm(0);
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void CheckDiagonal(uint8_t block, int8_t dir)
{
    SetRunMode(0);
    enc.distance = 0;
    Motion_DiagonalStart();
    if (dir == 1)
    {
        //Motion_DiagonalLeft(block);
        Motion_Diagonal(block);
    }
    else
    {
        Motion_DiagonalRight(block);
    }
    Motion_DiagonalStop();
    motor_flag = FALSE;
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void CheckTurn(uint8_t num)
{
    SetRunMode(0);
    gyro_z.degree = 0;
    for (uint8_t i = 0; i < num; i++)
    {
        Motion_TestTurn();
    }
    motor_flag = FALSE;
    //loger.velo_ang[loger.cnt] = (int16_t)gyro_z.degree;
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

void PrintWallData(void)
{
    wallData_t walldata;
    Flash_Load(start_address, (uint8_t *)&walldata, sizeof(wallData_t));
    printf("\r\n");
    for (unsigned char i = 0; i < MAZE_SIZE; i++)
    {
        printf("+");
        printf("---");
    }
    printf("+\r\n");
    for (unsigned char j = MAZE_SIZE; j > 0; j--)
    {
        for (unsigned char i = 0; i < MAZE_SIZE; i++)
        {
            if (walldata.vertical[i] & 0b1 << (j - 1))
            {
                printf("|");
            }
            else
            {
                printf(" ");
            }
            printf("   ");
        }
        printf("|");
        printf("\r\n");
        for (unsigned i = 0; i < MAZE_SIZE; i++)
        {
            printf("+");
            if (walldata.horizontal[j - 1] & 0b1 << i)
            {
                printf("---");
            }
            else
            {
                printf("   ");
            }
        }
        printf("+\r\n");
    }
}

void PrintMotion(uint16_t *motion, uint32_t *velocity, uint8_t tail)
{
    uint8_t head = 0;
    while (head < tail)
    {
        switch (motion[head] & 0xf)
        {
        case START:
            printf("\r\nSTART:%d\tv_start:%ld\tv_end:%ld\r\n", motion[head] >> 4, velocity[head] >> 16, velocity[head] & 0xffff);
            break;
        case LEFT:
            switch (motion[head] >> 4)
            {
            case SEARCH:
                printf("LEFT:SEARCH\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_90:
                printf("LEFT:T_90\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_180:
                printf("LEFT:T_180\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_45IN:
                printf("LEFT:T_45IN\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_45OUT:
                printf("LEFT:T_45OUT\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_135IN:
                printf("LEFT:T_135IN\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_135OUT:
                printf("LEFT:T_135OUT\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_V90:
                printf("LEFT:T_V90\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            default:
                printf("\r\n");
                break;
            }
            break;
        case RIGHT:
            switch (motion[head] >> 4)
            {
            case SEARCH:
                printf("RIGHT:SEARCH\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_90:
                printf("RIGHT:T_90\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_180:
                printf("RIGHT:T_180\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_45IN:
                printf("RIGHT:T_45IN\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_45OUT:
                printf("RIGHT:T_45OUT\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_135IN:
                printf("RIGHT:T_135IN\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_135OUT:
                printf("RIGHT:T_135OUT\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            case T_V90:
                printf("RIGHT:T_V90\tv_start:%ld\tv_end:%ld\n\r", velocity[head] >> 16, velocity[head] & 0xffff);
                break;
            default:
                printf("\r\n");
                break;
            }
            break;
        case FRONT:
            printf("FRONT:%d\tv_start:%ld\tv_end:%ld\n\r", motion[head] >> 4, velocity[head] >> 16, velocity[head] & 0xffff);
            break;
        case DIAGONAL:
            printf("DIAGONAL:%d\tv_start:%ld\tv_end:%ld\n\r", motion[head] >> 4, velocity[head] >> 16, velocity[head] & 0xffff);
            break;
        case GOAL:
            printf("GOAL:%d\tv_start:%ld\tv_end:%ld\n\r", motion[head] >> 4, velocity[head] >> 16, velocity[head] & 0xffff);
            break;
        case ADJUST:
            printf("ADJUST:%d\tv_start:%ld\tv_end:%ld\n\r", motion[head] >> 4, velocity[head] >> 16, velocity[head] & 0xffff);
            break;
        default:
            printf("\r\n");
            break;
        }
        head++;
    }
}

void AdjustStraight(void)
{
    SetRunMode(0);
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
    SetRunMode(0);
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
        if (row > 1)
        {
            Motion_Straight(1);
        }

        if (dir == 1)
        {
            Motion_LeftTurn();
        }
        else
        {
            Motion_RightTurn();
        }

        if (colum > 1)
        {
            Motion_Straight(1);
        }

        if (dir == 1)
        {
            Motion_LeftTurn();
        }
        else
        {
            Motion_RightTurn();
        }

        if (row > 1)
        {
            Motion_Straight(1);
        }

        if (dir == 1)
        {
            Motion_LeftTurn();
        }
        else
        {
            Motion_RightTurn();
        }

        if (colum > 1)
        {
            Motion_Straight(1);
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

void TestComb(uint8_t num, uint8_t dir)
{
    if (dir == LEFT)
    {
        Motion_Start();
        for (uint8_t i = 0; i < num; i++)
        {
            Motion_LeftTurn();
            Motion_WallSpinTurn();
            Motion_LeftTurn();
        }
        Motion_End();
    }
    else if (dir == RIGHT)
    {
        Motion_Start();
        for (uint8_t i = 0; i < num; i++)
        {
            Motion_RightTurn();
            Motion_WallSpinTurn();
            Motion_RightTurn();
        }
        Motion_End();
    }
    motor_flag = FALSE;
}

void TestSteps(uint8_t num)
{
    Motion_Start();
    for (uint8_t i = 0; i < num; i++)
    {
        Motion_RightTurn();
        Motion_LeftTurn();
    }
    Motion_End();
    motor_flag = FALSE;
}

void Turn45_Test(uint8_t in_dir, uint8_t out_dir, uint8_t step)
{
    SetRunMode(0);

    switch (in_dir)
    {
    case FRONT:
        Motion_DiagonalStart();
        break;
    case LEFT:
        Motion_FastStart(1, VELO_F);
        Motion_Adjust_L(1, VELO_F);
        Motion_InLeft45Turn();
        break;
    case RIGHT:
        Motion_FastStart(1, VELO_F);
        Motion_Adjust_R(1, VELO_F);
        Motion_InRight45Turn();
        break;
    default:
        break;
    }

    switch (out_dir)
    {
    case FRONT:
        Motion_DiagonalStop();
        break;
    case LEFT:
        if (step > 0)
        {
            Motion_DiagonalLeft(step);
        }
        Motion_OutLeft45Turn(VELO_F);
        Motion_FastGoal(1, VELO_F);
        break;
    case RIGHT:
        if (step > 0)
        {
            Motion_DiagonalRight(step);
        }
        Motion_OutRight45Turn(VELO_F);
        Motion_FastGoal(1, VELO_F);
        break;
    default:
        break;
    }

    motor_flag = FALSE;
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    HAL_Delay(300);
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void Turn135_Test(uint8_t in_dir, uint8_t out_dir, uint8_t step)
{
    SetRunMode(0);

    switch (in_dir)
    {
    case FRONT:
        Motion_DiagonalStart();
        break;
    case LEFT:
        Motion_FastStart(1, VELO_F);
        Motion_Adjust_L(1, VELO_F);
        Motion_FastLeftTurn(T_135IN, 800.0f);
        break;
    case RIGHT:
        Motion_FastStart(1, VELO_F);
        Motion_Adjust_R(1, VELO_F);
        Motion_FastRightTurn(T_135IN, 800.0f);
        break;
    default:
        break;
    }

    switch (out_dir)
    {
    case FRONT:
        Motion_DiagonalStop();
        break;
    case LEFT:
        Motion_DiagonalLeft(step);
        Motion_FastLeftTurn(T_135OUT, 800.0f);
        Motion_FastGoal(1, VELO_F);
        break;
    case RIGHT:
        Motion_DiagonalRight(step);
        Motion_FastRightTurn(T_135OUT, 800.0f);
        Motion_FastGoal(1, VELO_F);
        break;
    default:
        break;
    }

    motor_flag = FALSE;
    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    HAL_Delay(300);
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void TurnBig90_Test(int8_t dir)
{
    SetRunMode(0);
    Motion_FastStart(1, VELO_F);
    if (dir == LEFT)
    {
        Motion_Adjust_L(1, VELO_F);
        Motion_Left90Turn(VELO_F);
    }
    else if (dir == RIGHT)
    {
        Motion_Adjust_R(1, VELO_F);
        Motion_Right90Turn(VELO_F);
    }
    Motion_FastGoal(1, VELO_F);
    motor_flag = FALSE;
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void TurnBig180_Test(int8_t dir)
{
    SetRunMode(0);
    Motion_FastStart(1, VELO_F);
    Motion_Adjust(1, VELO_F);
    if (dir == LEFT)
    {
        Motion_Adjust_L(1, VELO_F);
        Motion_FastLeftTurn(T_180, VELO_F);
    }
    else if (dir == RIGHT)
    {
        Motion_Adjust_R(1, VELO_F);
        Motion_FastRightTurn(T_180, VELO_F);
    }
    Motion_FastGoal(1, VELO_F);
    motor_flag = FALSE;
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void TurnV90_Test(int8_t dir)
{
    SetRunMode(0);
    Motion_DiagonalStart();

    if (dir == LEFT)
    {
        Motion_Adjust_L(1, VELO_F);
        Motion_FastLeftTurn(T_V90, 800.0f);
    }
    else
    {
        Motion_Adjust_R(1, VELO_F);
        Motion_FastRightTurn(T_V90, 800.0f);
    }
    Motion_DiagonalStop();
    motor_flag = FALSE;
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}

void FastestTurnTest(uint8_t dir, uint8_t deg)
{
    SetRunMode(1);
    enc.distance = 0;
    switch (deg)
    {
    case SEARCH:
        if (dir == 1)
        {
            Motion_FastestStart(3, VELO_EST);
            Motion_FastestLeftTurn(deg, VELO_EST);
            Motion_FastestGoal(2, VELO_EST);
            motor_flag = FALSE;
            Tim_FanPwm(0);
        }
        else
        {
            Motion_FastestStart(3, VELO_EST);
            Motion_FastestRightTurn(deg, VELO_EST);
            Motion_FastestGoal(2, VELO_EST);
            motor_flag = FALSE;
            Tim_FanPwm(0);
        }
        break;
    case T_90:
        if (dir == 1)
        {
            Motion_FastestStart(4, VELO_EST);
            Motion_FastestLeftTurn(deg, VELO_EST);
            Motion_FastestGoal(2, VELO_EST);
            motor_flag = FALSE;
            Tim_FanPwm(0);
        }
        else
        {
            Motion_FastestStart(4, VELO_EST);
            Motion_FastestRightTurn(deg, VELO_EST);
            Motion_FastestGoal(2, VELO_EST);
            motor_flag = FALSE;
            Tim_FanPwm(0);
        }
        break;
    case T_180:
        if (dir == 1)
        {
            Motion_FastestStart(4, VELO_EST);
            Motion_FastestLeftTurn(deg, VELO_EST);
            Motion_FastestGoal(2, VELO_EST);
            motor_flag = FALSE;
            Tim_FanPwm(0);
        }
        else
        {
            Motion_FastestStart(4, VELO_EST);
            Motion_FastestRightTurn(deg, VELO_EST);
            Motion_FastestGoal(2, VELO_EST);
            motor_flag = FALSE;
            Tim_FanPwm(0);
        }
        break;
    case T_45IN:
        if (dir == 1)
        {
            Motion_FastestStart(4, VELO_EST);
            Motion_FastestLeftTurn(deg, VELO_EST);
            Motion_FastestGoal(1, VELO_EST);
            motor_flag = FALSE;
            Tim_FanPwm(0);
        }
        else
        {
            Motion_FastestStart(4, VELO_EST);
            Motion_FastestRightTurn(deg, VELO_EST);
            Motion_FastestGoal(1, VELO_EST);
            motor_flag = FALSE;
            Tim_FanPwm(0);
        }
        break;
    default:
    break;
    }

    HAL_Delay(500);
    Flash_Write(start_address, (uint8_t *)&loger, sizeof(loger_t));
    Tim_BuzzerPwm(HZ_C_H, 300);
    HAL_Delay(200);
    Tim_BuzzerPwm(HZ_NORMAL, 0);
}