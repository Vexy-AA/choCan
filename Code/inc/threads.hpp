#pragma once

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
#include "deviceSpecific.hpp"
#include "slcan.hpp"

namespace chibios_rt {
template <int N>
class CustomizedThread : public BaseThread {
protected:
  THD_WORKING_AREA(wa, N);

  void stackUsage(int& remainingStack){
    // calculate real size for stack
    stackSize = sizeof(wa) - PORT_WA_SIZE(0);
    int limit = stackSize;
    if (limit <= 0)
      limit = 4096;
    thread_t *th = chThdGetSelfX();
    uint32_t *p = (uint32_t *) th->wabase;
    // searching for end of stack usage
    stackFree = 0;
    while (stackFree < limit)
    {
      if (*p++ != 0x55555555)
        break;
      stackFree += sizeof(uint32_t);
    }
    remainingStack = stackFree;
  }
public:
    void sleep_ms(sysinterval_t interval){
    sleep(TIME_MS2I(interval));
    }
    ThreadReference start(tprio_t prio) override {
    void _thd_start(void *arg);
    return ThreadReference(chThdCreateStatic(wa, sizeof(wa), prio,
                                            _thd_start, this));
    }
private:
  int stackSize;
  int stackFree;
};

class usbHandlerThread : public CustomizedThread<512>{
public:
  usbHandlerThread(
    Mutex* usbMutex,
    ByteBuffer* transmitUSB,
    USBDriver* drvUSB, 
    SLCAN::CANIface* slCan,
    STM32F405::Pins<6>& leds) : 
        CustomizedThread<512>(),
        mUsbMutex(usbMutex),
        mToTransmitUSB(transmitUSB),
        mDrvUsb(drvUSB),
        mSlCan(slCan),
        mLeds(leds){
  }
protected:
    void main(void) override;
private:
  int remainingStack;
  Mutex* mUsbMutex;
  ByteBuffer* mToTransmitUSB;
  USBDriver* mDrvUsb;
  SLCAN::CANIface* mSlCan;
  STM32F405::Pins<6>& mLeds;
  uint32_t bytesReceived = 0;
  uint8_t buf[100];
};


class ledsUpdateThread : public CustomizedThread<512>{
public:
    ledsUpdateThread(
      STM32F405::Pins<6>& leds) : 
        CustomizedThread<512>(), 
        mLeds(leds){
  
    }
protected:
    void main(void) override;
private:
    int remainingStack;
    STM32F405::Pins<6>& mLeds;
};

class can1HandlerThread : public CustomizedThread<512>{
public:
    can1HandlerThread(
      Mutex* canMutex,
      ObjectBuffer<CANRxFrame>* receivedCAN1,
      ObjectBuffer<CANTxFrame>* toTransmitCAN1,
      SLCAN::CANIface* slCan,
      STM32F405::Pins<6>& leds) : 
        CustomizedThread<512>(), 
        mCanMutex(canMutex),
        mReceivedCAN1(receivedCAN1),
        mToTransmitCAN1(toTransmitCAN1),
        mSlCan(slCan),
        mLeds(leds){
  
    }
protected:
    void main(void) override;
private:
    int remainingStack;
    Mutex* mCanMutex;
    ObjectBuffer<CANRxFrame>* mReceivedCAN1;
    ObjectBuffer<CANTxFrame>* mToTransmitCAN1;
    SLCAN::CANIface* mSlCan;
    STM32F405::Pins<6>& mLeds;
};

class slCanHandlerThread : public CustomizedThread<512>{
public:
    slCanHandlerThread(
      Mutex* usbMutex,
      Mutex* canMutex,
      ObjectBuffer<CANRxFrame>* receivedCAN1,
      ByteBuffer* transmitUSB,
      SLCAN::CANIface* slCan) : 
        CustomizedThread<512>(), 
        mUsbMutex(usbMutex),
        mCanMutex(canMutex),
        mReceivedCAN1(receivedCAN1),
        mToTransmitUSB(transmitUSB),
        mSlCan(slCan){
  
    }
protected:
    void main(void) override;
private:
    int remainingStack;
    Mutex* mUsbMutex;
    Mutex* mCanMutex;
    ObjectBuffer<CANRxFrame>* mReceivedCAN1;
    ByteBuffer* mToTransmitUSB;
    SLCAN::CANIface* mSlCan;

};



}
