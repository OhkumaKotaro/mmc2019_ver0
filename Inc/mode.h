/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __mode_H
#define __mode_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"

void Mode_Mouse(int8_t mode);
char Mode_Select(void);

#ifdef __cplusplus
}
#endif
#endif /*__ mode_H */