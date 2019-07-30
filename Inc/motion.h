/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __motion_H
#define __motion_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

//search
#define VELO_S 400.0f
#define ACCEL_S 4000.0f
#define VELO_ANG_S 400.0f
#define ACCEL_ANG_S 8000.0f
#define VELO_ANG_S_SLALOM 600.0f
#define ACCEL_ANG_S_SLALOM 10000.0f

//fast
#define ACCEL_F 5000.0f
#define VELO_F 800.0f
#define VELO_ANG_D 400.0f
#define ACCEL_ANG_D 8000.0f

    void Motion_enkai(void);
    void Motion_Start(void);
    void Motion_Restart(uint8_t wall_is);
    void Motion_End(void);
    void Motion_Straight(void);
    void Motion_TestTurn(void);
    void Motion_SpinTurn(void);
    void Motion_WallSpinTurn(void);
    void Motion_LeftTurn(void);
    void Motion_RightTurn(void);
    void Motion_FastStart(uint8_t step,float velo_end);
    void Motion_FastStraight(uint8_t step,float v_start,float v_end);
    void Motion_FastGoal(uint8_t step,float v_start);
    void Motion_Switch(uint8_t motion);
    void Motion_Diagonal(uint8_t step);
    void Motion_DiagonalStop(void);
    void Motion_Left45Turn(float v_start,float v_end);
    void Motion_Right45Turn(float v_start,float v_end);
    void Motion_Left90Turn(void);
    void Motion_Right90Turn(void);
    void Motion_Left90BigTurn(float v_start,float v_end);
    void Motion_Right90BigTurn(float v_start,float v_end);

#ifdef __cplusplus
}
#endif
#endif /*__ motion_H */