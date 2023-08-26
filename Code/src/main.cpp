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



class MainThread : public BaseStaticThread<2048> {
  public:
    MainThread() : BaseStaticThread<2048>(){
  
  }
  protected:
    void main(void) override {
      int32_t test1 = 34;
      ByteBuffer rxUSB(100);
      sduObjectInit(&SDU1);
      sduStart(&SDU1, &serusbcfg);

      usbDisconnectBus(serusbcfg.usbp);
      BaseThread::sleep(TIME_MS2I(1000));
      usbStart(serusbcfg.usbp, &usbcfg);
      usbConnectBus(serusbcfg.usbp);

      
      
      STM32F405::Pins<tPinNames::totalNumber> leds;
      leds.add(ledsBoard,tPinNames::totalNumber);
      ledsUpdateThread ledsUpdateTh(leds);

      ledsUpdateTh.start(NORMALPRIO + 20);



      usbTransmitThread usbTransmit(&rxUSB, serusbcfg.usbp, leds);
      usbReceiveThread usbReceive(&rxUSB, serusbcfg.usbp, leds);

      usbTransmit.start(NORMALPRIO + 10);
      usbReceive.start(NORMALPRIO + 10);

      while(1){
        test1++;
        leds.on(tPinNames::ledGreen2,500,TIME_I2MS(System::getTime()));
        BaseThread::sleep(TIME_MS2I(1000));
      }
    }
  private:
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
   
    BaseThread::sleep(TIME_MS2I(1000));
  }

  return 0;
}
