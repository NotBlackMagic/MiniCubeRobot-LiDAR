#ifndef HAL_CRC_H_
#define HAL_CRC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"

void CRCInit(uint16_t* crc);
void CRCWrite(uint8_t data, uint16_t* crc);
uint16_t CRCRead(uint16_t* crc);
void CRCReset(uint16_t *crc);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CRC_H_ */
