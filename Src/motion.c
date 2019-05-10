#include "motion.h"
#include "stdint.h"
#include "control.h"

#define TRUE 1
#define FALSE 0

volatile uint8_t motion_end_flag;

void Motion_enkai(void){
    Control_ResetVelo();
    Control_StrCalculator(0,0,0,0,1,0);
    Control_AngCalculator(0,0,0,0,1,0);
}

void Motion_Start(void){
    Control_ResetVelo();
    Control_StrCalculator(90.0f,0.0f,400.0f,400.0f,4000.0f,1);
    Control_AngCalculator(0,0,0,0,1,0);
    motion_end_flag = FALSE;
    while(motion_end_flag==FALSE);
}

void Motion_End(void){
    Control_StrCalculator(90.0f,400.0f,400.0f,0.0f,4000.0f,1);
    Control_AngCalculator(0,0,0,0,1,0);
    motion_end_flag = FALSE;
    while(motion_end_flag==FALSE);
}

void Motion_Straight(void){
    Control_ResetVelo();
    Control_StrCalculator(180.0f,0.0f,400.0f,0.0f,4000.0f,1);
    Control_AngCalculator(0,0,0,0,1,0);
    motion_end_flag = FALSE;
    while(motion_end_flag==FALSE);
}

void Motion_SpinTurn(void){
    Control_StrCalculator(180.0f,0.0f,400.0f,0.0f,4000.0f,-1);
    Control_AngCalculator(0,0,0,0,1,0);
    motion_end_flag = FALSE;
    while(motion_end_flag==FALSE);
}
