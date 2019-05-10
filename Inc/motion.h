/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __motion_H
#define __motion_H
#ifdef __cplusplus
 extern "C" {
#endif


void Motion_enkai(void);
void Motion_Start(void);
void Motion_End(void);
void Motion_Straight(void);
void Motion_SpinTurn(void);

#ifdef __cplusplus
}
#endif
#endif /*__ motion_H */