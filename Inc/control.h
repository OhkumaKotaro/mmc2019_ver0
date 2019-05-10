/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __control_H
#define __control_H
#ifdef __cplusplus
 extern "C" {
#endif


void Control_StrCalculator(float var,float velo_s,float velo_m,float velo_e,float accel,float dir);
void Control_AngCalculator(float var,float velo_s,float velo_m,float velo_e,float accel,float dir);
void Control_PrintLoger(void);
void Control_UpdatePwm(void);
void Control_ResetVelo(void);


#ifdef __cplusplus
}
#endif
#endif /*__ control_H */