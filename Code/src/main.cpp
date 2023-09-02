/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "main.h"

using namespace chibios_rt;

/*
 * Message server thread class. It receives messages and does nothing except
 * reply after the specified time.
 */
#define usb1 (BaseSequentialStream*)&SDU1

static const CANConfig cancfg1000 = {
  CAN_MCR_DBF | CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_SJW(0) |
  CAN_BTR_BRP(2) | CAN_BTR_TS1(10) | CAN_BTR_TS2(1)
};

static const CANConfig cancfg500 = {
  CAN_MCR_DBF | CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_SJW(0) |
  CAN_BTR_BRP(2) | CAN_BTR_TS1(10) | CAN_BTR_TS2(1)
};

class MainThread : public CustomizedThread<16384> {
  public:
    MainThread() : CustomizedThread<16384>(){
  
  }
  protected:
    void main(void) override {
      int32_t test1 = 34;
      Mutex usbMutex;
      Mutex canMutex;
      ByteBuffer toTransmitUSB(64);
      ObjectBuffer<CANRxFrame> receivedCAN1(256);
      ObjectBuffer<CANTxFrame> toTransmitCAN1(32);
      SLCAN::CANIface slCan;
      /*--------- INDICATION -----------*/
      STM32F405::Pins<tPinNames::totalNumber> leds;
      leds.add(ledsBoard,tPinNames::totalNumber);
      ledsUpdateThread ledsUpdateTh(leds);
      ledsUpdateTh.start(NORMALPRIO + 1);
      /*--------- CAN -------------*/
      canStart(&CAND1, &cancfg1000);
      canStart(&CAND2, &cancfg1000);

      can1HandlerThread can1Handler(&canMutex,&receivedCAN1,&toTransmitCAN1,&slCan,leds);
      can1Handler.start(NORMALPRIO + 30);
      /*--------- USB -----------*/
      sduObjectInit(&SDU1);
      sduStart(&SDU1, &serusbcfg);

      usbDisconnectBus(serusbcfg.usbp);
      BaseThread::sleep(TIME_MS2I(1000));
      usbStart(serusbcfg.usbp, &usbcfg);
      usbConnectBus(serusbcfg.usbp);

      usbHandlerThread usbHandler(&usbMutex,&toTransmitUSB,serusbcfg.usbp,&slCan,leds);

      usbHandler.start(NORMALPRIO + 15);




      slCanHandlerThread slCanHandler(&usbMutex,&canMutex,&receivedCAN1,&toTransmitUSB,&slCan,leds);
      //slCanHandler.start(NORMALPRIO + 10);
      while(1){
        test1++;
        leds.on(tPinNames::ledGreen2,500,TIME_I2MS(System::getTime()));
        BaseThread::sleep(TIME_MS2I(1000));
        stackUsage(remainingStack);
      }
    }
  private:
    int remainingStack;
    void sleep_ms(sysinterval_t interval){
      sleep(TIME_MS2I(interval));
    }
};

/*
 * Application entry point.
 */
/* Static threads instances.*/
static MainThread mainThread;

int32_t testGG = 34;
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  System::init();
  testGG++;
  

  mainThread.start(NORMALPRIO + 1);

  while (true) {
     testGG++;
    //sleep(TIME_MS2I(1000));
    //BaseThread::sleep(TIME_MS2I(1000));
  }

  return 0;
}
