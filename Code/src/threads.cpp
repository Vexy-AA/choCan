#include "threads.hpp"

namespace chibios_rt {

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
  }
}

void ledsUpdateThread::main(void){
  setName("ledThread");
  while (true){
  mLeds.clock(TIME_I2MS(System::getTime()));
  sleep_ms(1);
  }
    
}

void can1ReceiveThread::main(void){
  while(true){
    
  }
  /* setName("can1RxThread");
  event_listener_t el;
  CANRxFrame rxmsg;
  chEvtRegister(&CAND1.rxfull_event, &el, 0);
  while(true){
	  if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0)
	        continue;
    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK){
      mRxCAN->push(rxmsg);
      mLeds.on(tPinNames::ledWhite,1,TIME_I2MS(System::getTime()));
    }
  } */
}

void can1TransmitThread::main(void){
  setName("can1TxThread");
  while(true){
    
  }
}



}
