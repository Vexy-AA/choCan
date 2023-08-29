#include "threads.hpp"

namespace chibios_rt {

void usbHandlerThread::main(void){
  setName("usbHandler");
  uint32_t bytesReceived = 0;
  while(true){
    if (mDrvUsb->state == USB_ACTIVE){
      if (mToTransmitUSB->available()){
        if (mToTransmitUSB->block()){
          chnWriteTimeout(&SDU1, mToTransmitUSB->readptr(bytesReceived), bytesReceived, TIME_IMMEDIATE);
          mToTransmitUSB->clear();
          mToTransmitUSB->unblock();
          mLeds.on(tPinNames::ledRed,1,TIME_I2MS(System::getTime()));
        }
      }
      if (mToTransmitUSB->block()){
      uint8_t buf[100];
      uint32_t bytesReceived = chnReadTimeout(&SDU1, buf, sizeof(buf),TIME_IMMEDIATE);
      if (bytesReceived){
          mLeds.on(tPinNames::ledGreen,1,TIME_I2MS(System::getTime()));
          mToTransmitUSB->write(buf,bytesReceived);
      }
      mToTransmitUSB->unblock();
      }
    }
    sleep_ms(1);
    stackUsage(remainingStack);
  }
}

/* 
void usbTransmitThread::main(void){
  setName("usbTxThread");
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
      stackUsage(remainingStack);
    }
}

void usbReceiveThread::main(void){
  setName("usbRxThread");
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
    stackUsage(remainingStack);
  }
}
 */
void ledsUpdateThread::main(void){
  setName("ledThread");
  while (true){
    mLeds.clock(TIME_I2MS(System::getTime()));
    sleep_ms(1);
    stackUsage(remainingStack);
  }
    
}

void can1HandlerThread::main(void){
  setName("can1RxThread");
  event_listener_t el;
  CANRxFrame canRxFrame;
  CANTxFrame canTxFrame;
  chEvtRegister(&CAND1.rxfull_event, &el, 0);
  while(true){
	  stackUsage(remainingStack);
    if (mReceivedCAN1->block()){
      if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_IMMEDIATE) != 0){
        while (canReceive(&CAND1, CAN_ANY_MAILBOX, &canRxFrame, TIME_IMMEDIATE) == MSG_OK){
          mReceivedCAN1->push(canRxFrame);
          mLeds.on(tPinNames::ledWhite,1,TIME_I2MS(System::getTime()));
        }
      }
      mReceivedCAN1->unblock();
    }
    if (mToTransmitCAN1->block()){
      if(mToTransmitCAN1->peek(canTxFrame)){
        while (canTransmit(&CAND1,CAN_ANY_MAILBOX,&canTxFrame,TIME_IMMEDIATE) == MSG_OK){
          mToTransmitCAN1->pop();
          mLeds.on(tPinNames::ledBlue,1,TIME_I2MS(System::getTime()));
          if(!mToTransmitCAN1->peek(canTxFrame)){
            break;
          }
        }
      }
      mToTransmitCAN1->unblock();
    }
    sleep_ms(1);
    stackUsage(remainingStack);
  }
}

}
