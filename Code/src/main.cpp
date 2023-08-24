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
#include <stdint.h>
#include <atomic>
#include <stdlib.h>
#include <string.h>

#include "ch.hpp"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"


#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "usbcfg.h"
#include "ringBuffer.hpp"

using namespace chibios_rt;

/*
 * Message server thread class. It receives messages and does nothing except
 * reply after the specified time.
 */
#define usb1 (BaseSequentialStream*)&SDU1

class MessageServerThread : public BaseStaticThread<256> {

protected:
  void main(void) override {

    setName("server");

    while (true) {
      ThreadReference sender = waitMessage();
      time_msecs_t msecs = (time_msecs_t)sender.getMessage();
      sleep(TIME_MS2I(msecs));
      sender.releaseMessage(0);
    }
  }

public:
  MessageServerThread(void) : BaseStaticThread<256>() {
  }
};

/* Reference to the server thread.*/
static ThreadReference sref;

/*
 * LED blink sequences.
 * NOTE: Sequences must always be terminated by a GOTO instruction.
 * NOTE: The sequencer language could be easily improved but this is outside
 *       the scope of this demo.
 */
#define SLEEP           0
#define GOTO            1
#define STOP            2
#define BITCLEAR        3
#define BITSET          4
#define MESSAGE         5

typedef struct {
  uint8_t       action;
  union {
    msg_t       msg;
    uint32_t    value;
    ioline_t    line;
  };
} seqop_t;

// Flashing sequence for LED3.
static const seqop_t LED3_sequence[] =
{
  {BITSET,      LINE_LED13},
  {SLEEP,       800},
  {BITCLEAR,    LINE_LED13},
  {SLEEP,       200},
  {GOTO,        0}
};

// Flashing sequence for LED4.
static const seqop_t LED4_sequence[] =
{
  {BITSET,      LINE_LED4},
  {SLEEP,       600},
  {BITCLEAR,    LINE_LED4},
  {SLEEP,       400},
  {GOTO,        0}
};

// Flashing sequence for LED5.
static const seqop_t LED5_sequence[] =
{
  {BITSET,      LINE_LED5},
  {SLEEP,       400},
  {BITCLEAR,    LINE_LED5},
  {SLEEP,       600},
  {GOTO,        0}
};

// Flashing sequence for LED6.
static const seqop_t LED6_sequence[] =
{
  {BITSET,      LINE_LED6},
  {SLEEP,       200},
  {BITCLEAR,    LINE_LED6},
  {SLEEP,       800},
  {GOTO,        0}
};

// Message sequence.
static const seqop_t msg_sequence[] =
{
  {MESSAGE,     50},
  {SLEEP,       1000},
  {GOTO,        0}
};

/* 
 * Sequencer thread class. It can drive LEDs or other output pins.
 * Any sequencer is just an instance of this class, all the details are
 * totally encapsulated and hidden to the application level.
 */
class usbTransmitThread : public BaseStaticThread<128>{
  public:
  usbTransmitThread(ByteBuffer* rxUSB, USBDriver* drvUSB) : BaseStaticThread<128>(), mRxUSB(rxUSB),mDrvUsb(drvUSB){
  
  }
  protected:
    void main(void) override {
      
      while (true){
        uint32_t bytesReceived = 0;
        if (mRxUSB->available()){
          if (mRxUSB->block()){
            chnWriteTimeout(&SDU1, mRxUSB->readptr(bytesReceived), bytesReceived, TIME_IMMEDIATE);
            mRxUSB->clear();
            mRxUSB->unblock();
          }
        }
        sleep_ms(1);
      }
      
    }
  private:
  void sleep_ms(sysinterval_t interval){
    sleep(TIME_MS2I(interval));
  }
  ByteBuffer* mRxUSB;
  USBDriver* mDrvUsb;
};
class usbReceiveThread : public BaseStaticThread<128>{
  public:
  usbReceiveThread(ByteBuffer* rxUSB, USBDriver* drvUSB) : BaseStaticThread<128>(), mRxUSB(rxUSB),mDrvUsb(drvUSB){
  
  }
  protected:
    void main(void) override {
      
      while (true){
        if (mDrvUsb->state == USB_ACTIVE){
          if (mRxUSB->block()){
            uint8_t buf[100];
            uint32_t bytesReceived = chnReadTimeout(&SDU1, buf, sizeof(buf),TIME_IMMEDIATE);
            if (bytesReceived)
            mRxUSB->write(buf,bytesReceived);
            mRxUSB->unblock();
          }
        }
        sleep_ms(1);
      }
      
    }
  private:
  void sleep_ms(sysinterval_t interval){
    sleep(TIME_MS2I(interval));
  }
  ByteBuffer* mRxUSB;
  USBDriver* mDrvUsb;
};
class SequencerThread : public BaseStaticThread<128> {
private:
  const seqop_t *base, *curr;                   // Thread local variables.

protected:
  void main(void) override {

    setName("sequencer");

    while (true) {
      switch(curr->action) {
      case SLEEP:
        sleep(TIME_MS2I(curr->value));
        break;
      case GOTO:
        curr = &base[curr->value];
        continue;
      case STOP:
        return;
      case BITCLEAR:
        palClearLine(curr->line);
        break;
      case BITSET:
        palSetLine(curr->line);
        break;
      case MESSAGE:
        sref.sendMessage(curr->msg);
        break;
      }
      curr++;
    }
  }

public:
  SequencerThread(const seqop_t *sequence) : BaseStaticThread<128>() {

    base = curr = sequence;
  }
};



static SequencerThread blinker1(LED3_sequence);
static SequencerThread sender1(msg_sequence);
class MainThread : public BaseStaticThread<1024> {
  public:
    MainThread() : BaseStaticThread<1024>(){
  
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

      
      blinker1.start(NORMALPRIO + 20);

      usbTransmitThread usbTransmit(&rxUSB, serusbcfg.usbp);
      usbReceiveThread usbReceive(&rxUSB, serusbcfg.usbp);

      usbTransmit.start(NORMALPRIO + 10);
      usbReceive.start(NORMALPRIO + 10);
      while(1){
        test1++;
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
static MessageServerThread server_thread;

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
  
  sref = server_thread.start(NORMALPRIO + 20);

  mainThread.start(NORMALPRIO + 1);

  while (true) {
    testGG++;
   
    BaseThread::sleep(TIME_MS2I(1000));
  }

  return 0;
}
