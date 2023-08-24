#ifndef ALT_MAIN_H_
#define ALT_MAIN_H_

#include "can.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif






int altMain();
int8_t usbReceive(uint8_t* Buf, uint32_t *Len);
void canRxInt(CAN_HandleTypeDef *_hcan, uint8_t fifo);












#ifdef __cplusplus
}
#endif

#endif
