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
static const seqop_t ledGreen1[] =
{
  {BITCLEAR,    LINE_LED_GREEN_1},
  {SLEEP,       0},
  {BITSET,      LINE_LED_GREEN_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_GREEN_1},
  {SLEEP,       400},
  {GOTO,        0}
};

// Flashing sequence for LED4.
static const seqop_t ledRed1[] =
{
  {BITCLEAR,    LINE_LED_RED_1},
  {SLEEP,       50},
  {BITSET,      LINE_LED_RED_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_RED_1},
  {SLEEP,       350},
  {GOTO,        0}
};

// Flashing sequence for LED5.
static const seqop_t ledBlue1[] =
{
  {BITCLEAR,    LINE_LED_BLUE_1},
  {SLEEP,       100},
  {BITSET,      LINE_LED_BLUE_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_BLUE_1},
  {SLEEP,       300},
  {GOTO,        0}
};

// Flashing sequence for LED6.
static const seqop_t ledYellow1[] =
{
  {BITCLEAR,    LINE_LED_YELLOW_1},
  {SLEEP,       150},
  {BITSET,      LINE_LED_YELLOW_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_YELLOW_1},
  {SLEEP,       250},
  {GOTO,        0}
};

static const seqop_t ledWhite1[] =
{
  {BITCLEAR,    LINE_LED_WHITE_1},
  {SLEEP,       200},
  {BITSET,      LINE_LED_WHITE_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_WHITE_1},
  {SLEEP,       200},
  {GOTO,        0}
};

static const seqop_t ledGreen2[] =
{
  {BITCLEAR,    LINE_LED_GREEN_2},
  {SLEEP,       250},
  {BITSET,      LINE_LED_GREEN_2},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_GREEN_2},
  {SLEEP,       150},
  {GOTO,        0}
};

/* 
 * Sequencer thread class. It can drive LEDs or other output pins.
 * Any sequencer is just an instance of this class, all the details are
 * totally encapsulated and hidden to the application level.
 */
class usbTransmitThread : public CustomizedThread<128>{
public:
  usbTransmitThread(ByteBuffer* rxUSB, USBDriver* drvUSB) : CustomizedThread<128>(), mRxUSB(rxUSB),mDrvUsb(drvUSB){
  
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
  ByteBuffer* mRxUSB;
  USBDriver* mDrvUsb;
};
class usbReceiveThread : public CustomizedThread<128>{
public:
  usbReceiveThread(ByteBuffer* rxUSB, USBDriver* drvUSB) : CustomizedThread<128>(), mRxUSB(rxUSB),mDrvUsb(drvUSB){
  
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
  ByteBuffer* mRxUSB;
  USBDriver* mDrvUsb;
};
class SequencerThread : public BaseStaticThread<256> {
private:
  const seqop_t *base, *curr;                   // Thread local variables.

protected:
  void main(void) override {
    setName("sequencer");
    systime_t tNow = 0;
    while (true) {
      switch(curr->action) {
      case SLEEP:
        //sleepUntil(TIME_MS2I(curr->value) + tNow);
        if (curr->value == 0) break;
        sleep(TIME_MS2I(curr->value));
        break;
      case GOTO:
        curr = &base[curr->value];
        if (curr->value == 0)
          tNow = System::getTime();
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
      
        break;
      }
      curr++;
    }
  }

public:
  SequencerThread(const seqop_t *sequence) : BaseStaticThread<256>() {

    base = curr = sequence;
  }
};



static SequencerThread threadLightGreen1(ledGreen1);
static SequencerThread threadLightRed(ledRed1);
static SequencerThread threadLightBlue(ledBlue1);
static SequencerThread threadLightYellow(ledYellow1);
static SequencerThread threadLightWhite(ledWhite1);
static SequencerThread threadLightGreen2(ledGreen2);

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

      
      threadLightGreen1.start(NORMALPRIO + 20);
      threadLightRed.start(NORMALPRIO + 20);
      threadLightBlue.start(NORMALPRIO + 20);
      threadLightYellow.start(NORMALPRIO + 20);
      threadLightWhite.start(NORMALPRIO + 20);
      threadLightGreen2.start(NORMALPRIO + 20);

      usbTransmitThread usbTransmit(&rxUSB, serusbcfg.usbp);
      usbReceiveThread usbReceive(&rxUSB, serusbcfg.usbp);

      usbTransmit.start(NORMALPRIO + 10);
      usbReceive.start(NORMALPRIO + 10);

      /* palSetLine(LINE_LED_GREEN_1);
      palSetLine(LINE_LED_RED_1);
      palSetLine(LINE_LED_BLUE_1);
      palSetLine(LINE_LED_YELLOW_1);
      palSetLine(LINE_LED_WHITE_1);
      palSetLine(LINE_LED_GREEN_2); */
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
