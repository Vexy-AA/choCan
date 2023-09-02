#include "threads.hpp"

namespace chibios_rt {
void ledsUpdateThread::main(void){
  setName("ledThread");
  while (true){
    mLeds.clock(TIME_I2MS(System::getTime()));
    sleep_ms(1);
    stackUsage(remainingStack);
  }
    
}


void usbHandlerThread::main(void){
  ByteBuffer mReceivedUSB(256);
  setName("usbHandler");
  while(true){
    while(mSlCan->sendUsb()){
    }
    mSlCan->serialToCan();
    //if (mUsbMutex->tryLock()){

      /* if (mSlCan->ptxSerial()->available()){
        chnWriteTimeout(&SDU1, mSlCan->ptxSerial()->readptr(bytesReceived), bytesReceived, TIME_IMMEDIATE);
        mSlCan->ptxSerial()->clear();
        mLeds.on(tPinNames::ledRed,1,TIME_I2MS(System::getTime()));
      } */
      /* if (mToTransmitUSB->mutex.tryLock()){
        if (mToTransmitUSB->available()){
          chnWriteTimeout(&SDU1, mToTransmitUSB->readptr(bytesReceived), bytesReceived, TIME_IMMEDIATE);
          mToTransmitUSB->clear();
          mLeds.on(tPinNames::ledRed,1,TIME_I2MS(System::getTime()));
        }
        mToTransmitUSB->mutex.unlock();
      } */

      bytesReceived = chnReadTimeout(&SDU1, buf, sizeof(buf),TIME_IMMEDIATE);
      if (bytesReceived){
        mLeds.on(tPinNames::ledGreen,1,TIME_I2MS(System::getTime()));
        mReceivedUSB.write(buf,bytesReceived);
      }
     // mUsbMutex->unlock();
    //}

    if (mSlCan->storeSerialMessage(mReceivedUSB.readptr(bytesReceived),&bytesReceived)){
      mReceivedUSB.clear();
    }
    
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

    if (mCanMutex->tryLock()){
      /* if (mToTransmitCAN1->mutex->tryLock()){
        while(mToTransmitCAN1->pop(canTxFrame) &&
        (canTransmit(&CAND1,CAN_ANY_MAILBOX,&canTxFrame,TIME_IMMEDIATE) == MSG_OK)){
          mLeds.on(tPinNames::ledBlue,1,TIME_I2MS(System::getTime()));
        }
        mToTransmitCAN1->mutex->unlock();
      } */

      //if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_IMMEDIATE) != 0)
      {
        while(canReceive(&CAND1, CAN_ANY_MAILBOX, &canRxFrame, TIME_IMMEDIATE) == MSG_OK){
          mReceivedCAN1->push(canRxFrame);
        }
      }
      while(mSlCan->sendCan()){
        mLeds.on(tPinNames::ledBlue,1,TIME_I2MS(System::getTime()));
      }

      mCanMutex->unlock();
    }

    if (mSlCan->getRxMutex()->tryLock()){
      while(mReceivedCAN1->pop(canRxFrame)){
        mSlCan->storeCanMessage(canRxFrame);
        mLeds.on(tPinNames::ledWhite,1,TIME_I2MS(System::getTime()));
      }
      mSlCan->getRxMutex()->unlock();
    }
    sleep_ms(1);
    stackUsage(remainingStack);
  }
}
void slCanHandlerThread::main(void){
  setName("slCanThread");

  while(true){

    sleep_ms(1);
    stackUsage(remainingStack);
  }
}
}
