/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __flash_H
#define __flash_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"


//loger
typedef struct
{
    uint16_t cnt;
    int16_t target_velo[3000];
    int16_t target_velo_ang[3000];
    int16_t velo[3000];
    int16_t velo_ang[3000];
} loger_t;

// flash use address ( sector11 )
extern const uint32_t start_address; //sentor11 start address
extern const uint32_t end_adress;

void Flash_Erase(void);
void Flash_Write(uint32_t address, uint8_t *data, uint32_t size);
void Flash_Load(uint32_t address, uint8_t *data, uint32_t size);


#ifdef __cplusplus
}
#endif
#endif /*__flash_H */