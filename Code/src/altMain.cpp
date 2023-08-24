#include "altMain.h"
#include "slcan.hpp"

SLCAN::CANIface slCan;

int altMain(){

    HAL_CAN_Start(&hcan1);
    SLCAN::CANFrame frame;
    SLCAN::CanIOFlags flags;
    uint64_t time;
    while(1){
        
        slCan.receive(frame,time,flags);
    }
}

int8_t usbReceive(uint8_t* Buf, uint32_t *Len){
    //HAL_Delay(10);
    slCan.storeSerialMessage(Buf,Len);
    return 0;
}

void canRxInt(CAN_HandleTypeDef *_hcan, uint8_t fifo){
    slCan.storeCanMessage(fifo, SLCAN::CANIface::native_micros64());
}

