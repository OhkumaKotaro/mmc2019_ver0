/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __control_H
#define __control_H
#ifdef __cplusplus
 extern "C" {
#endif

typedef struct
{
    float up_term;
    float const_term;
    float down_term;
    float v_now;
    float accel;
    int8_t dir;
    float error;
} target_t;

void Control_StrCalculator(float var,float velo_s,float velo_m,float velo_e,float accel,float dir);
void Control_AngCalculator(float var,float velo_s,float velo_m,float velo_e,float accel,float dir);
void Control_EdgeSet(float offset);
void Control_PrintLoger(void);
void Control_UpdatePwm(void);
void Control_ResetVelo(void);
void Control_SetupStraightPID(float Kp, float Ki, float Kd);
void Control_SetupTurnPID(float Kp, float Ki, float Kd);

#ifdef __cplusplus
}
#endif
#endif /*__ control_H */