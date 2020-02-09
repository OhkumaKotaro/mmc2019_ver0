/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __motion_H
#define __motion_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
    void Motion_MaxVeloSet(float gain);
    //adjust
    void Motion_enkai(void);
    void Motion_TestTurn(void);
    //search
    void Motion_Start(void);
    void Motion_Restart(uint8_t wall_is);
    void Motion_End(void);
    void Motion_Straight(uint16_t step);
    void Motion_SpinTurn(void);
    void Motion_WallSpinTurn(void);
    void Motion_LeftTurn(void);
    void Motion_RightTurn(void);
    void Motion_Switch(uint8_t motion);
    //fast
    void Motion_FastStart(uint8_t step, float velo_end);
    void Motion_FastStraight(uint8_t step, float v_start, float v_end);
    void Motion_Adjust(uint16_t step,float velo_s);
    void Motion_Adjust_L(uint16_t step, float velo_s);
    void Motion_Adjust_R(uint16_t step, float velo_s);
    void Motion_FastGoal(uint8_t step, float v_start);
    void Motion_Diagonal(uint8_t step);
    void Motion_DiagonalLeft(uint8_t step);
    void Motion_DiagonalRight(uint8_t step);
    void Motion_DiagonalStart(void);
    void Motion_DiagonalStop(void);
    void Motion_InLeft45Turn(void);
    void Motion_InRight45Turn(void);
    void Motion_OutLeft45Turn(float v_end);
    void Motion_OutRight45Turn(float v_end);
    void Motion_Left90Turn(float v_end);
    void Motion_Right90Turn(float v_end);
    void Motion_FastLeftTurn(uint8_t type, float v_end);
    void Motion_FastRightTurn(uint8_t type, float v_end);
    //fastest
    void Motion_FastestStart(uint8_t step, float velo_end);
    void Motion_FastestStraight(uint8_t step, float v_start, float v_end);
    void Motion_FastestGoal(uint8_t step, float v_start);
    void Motion_FastestAdjust_L(uint16_t step, float velo_s);
    void Motion_FastestAdjust_R(uint16_t step, float velo_s);
    void Motion_FastestLeftTurn(uint8_t type, float v_end);
    void Motion_FastestRightTurn(uint8_t type, float v_end);
    void Motion_FastestDiagonalLeft(uint8_t step);
    void Motion_FastestDiagonalRight(uint8_t step);

#ifdef __cplusplus
}
#endif
#endif /*__ motion_H */