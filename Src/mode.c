#include "mode.h"
#include "spi.h"
#include "tim.h"
#include "adc.h"
#include "stdint.h"
#include "gpio.h"
#include "motion.h"
#include "control.h"

#define TRUE 1
#define FALSE 0

extern volatile accel_t accel;
extern volatile float gyro_y;
extern sensor_t sen_l;
extern sensor_t sen_fl;
extern sensor_t sen_front;
extern sensor_t sen_fr;
extern sensor_t sen_r;
extern enc_t enc;
extern volatile uint8_t motor_flag;

//Prototype Function
void SensorCheck(void);

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
        Adc_IrSensorStart();
        while(sen_front.is_wall!=TRUE){}
        Tim_BuzzerPwm(HZ_C_H,300);
        HAL_Delay(100);
        Tim_BuzzerPwm(HZ_NORMAL,0);
        HAL_Delay(5000);
        Spi_GyroReset();
        HAL_Delay(1200);
        Tim_BuzzerPwm(HZ_C_H,300);
        HAL_Delay(100);
        Tim_BuzzerPwm(HZ_NORMAL,0);
        motor_flag = TRUE;
        Motion_Straight();
        motor_flag = FALSE;
        break;
    case 1:
        Adc_IrSensorStart();
        while(sen_front.is_wall!=TRUE){}
        Tim_BuzzerPwm(HZ_C_H,300);
        HAL_Delay(100);
        Tim_BuzzerPwm(HZ_NORMAL,0);
        HAL_Delay(5000);
        Spi_GyroReset();
        HAL_Delay(1200);
        Tim_BuzzerPwm(HZ_C_H,300);
        HAL_Delay(100);
        Tim_BuzzerPwm(HZ_NORMAL,0);
        motor_flag=TRUE;
        Motion_Start();
        Motion_End();
        motor_flag=FALSE;
        break;
    case 2:
        Adc_IrSensorStart();
        while(sen_front.is_wall!=TRUE){}
        Tim_BuzzerPwm(HZ_C_H,300);
        HAL_Delay(100);
        Tim_BuzzerPwm(HZ_NORMAL,0);
        HAL_Delay(5000);
        Spi_GyroReset();
        HAL_Delay(1200);
        Tim_BuzzerPwm(HZ_C_H,300);
        HAL_Delay(100);
        Tim_BuzzerPwm(HZ_NORMAL,0);
        motor_flag = TRUE;
        Motion_enkai();
        while(1){}
        Tim_MotorPwm(0,0);
        motor_flag = FALSE;
        break;
    case 3:
        break;

    //adjust
    case 4:
        SensorCheck();
        break;
    case 5:
        break;
    case 6:
        break;
    case 7:
        Control_PrintLoger();
        break;
    default:
        break;
    }
}


char Mode_Select(void){
    int8_t mode = 0;
    uint8_t state = 0;

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
            Tim_BuzzerPwm(170 - 10 * mode,300);
            HAL_Delay(100);
            Tim_BuzzerPwm(HZ_NORMAL,0);
            HAL_Delay(500);
        }
        if(enc.distance > 20){
            state = 4 - state;
            enc.distance = 0;
        }
        if (accel.z > 30.0f)
        {
            Tim_BuzzerPwm(HZ_C_H,300);
            HAL_Delay(100);
            Tim_BuzzerPwm(HZ_NORMAL,0);
            HAL_Delay(500);
            break;
        }
        if(state==0){
            Gpio_FullColor(CYAN);
        }else{
            Gpio_FullColor(YELLOW);
        }
        Gpio_SideLed(mode);
        printf("%d\r", mode+state);
    }
    return mode+state;
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
        printf("l:%d fl:%d f:%d fr:%d r:%d \r", sen_l.now, sen_fl.now , sen_front.now, sen_fr.now , sen_r.now);

        if (accel.x>60)
        {
            Adc_IrSensorFinish();
            Tim_BuzzerPwm(HZ_C_H,300);
            HAL_Delay(300);
            Tim_BuzzerPwm(HZ_NORMAL,0);
            HAL_Delay(500);
            break;
        }
    }
}