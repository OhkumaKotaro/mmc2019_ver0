/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __motion_H
#define __motion_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

    void Motion_enkai(void);
    void Motion_Start(void);
    void Motion_Restart(uint8_t wall_is);
    void Motion_End(void);
    void Motion_Straight(void);
    void Motion_TestTurn(void);
    void Motion_SpinTurn(void);
    void Motion_WallSpinTurn(void);
    void Motion_LeftSpinTurn(void);
    void Motion_RightSpinTurn(void);
    void Motion_LeftTurn(void);
    void Motion_RightTurn(void);
    void Motion_FastStart(uint8_t step);
    void Motion_FastStraight(uint8_t step);
    void Motion_FastGoal(uint8_t step);
    void Motion_Switch(uint8_t motion);

#ifdef __cplusplus
}
#endif
#endif /*__ motion_H */