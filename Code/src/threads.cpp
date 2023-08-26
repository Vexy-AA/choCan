#include "threads.hpp"

namespace chibios_rt {

void usbTransmitThread::main(void){
    while (true){
        uint32_t bytesReceived = 0;
        if (mRxUSB->available()){
          if (mRxUSB->block()){
            chnWriteTimeout(&SDU1, mRxUSB->readptr(bytesReceived), bytesReceived, TIME_IMMEDIATE);
            mRxUSB->clear();
            mRxUSB->unblock();
            mLeds.on(tPinNames::ledRed,1,TIME_I2MS(System::getTime()));
          }
        }
        sleep_ms(1);
      }
}

void usbReceiveThread::main(void){
    while (true){
    if (mDrvUsb->state == USB_ACTIVE){
        if (mRxUSB->block()){
        uint8_t buf[100];
        uint32_t bytesReceived = chnReadTimeout(&SDU1, buf, sizeof(buf),TIME_IMMEDIATE);
        if (bytesReceived){
            mLeds.on(tPinNames::ledGreen,1,TIME_I2MS(System::getTime()));
            mRxUSB->write(buf,bytesReceived);
        }
        mRxUSB->unblock();
        }
    }
    sleep_ms(1);
    }
}

void ledsUpdateThread::main(void){
    while (true){
    mLeds.clock(TIME_I2MS(System::getTime()));
    sleep_ms(1);
    }
    
}


}