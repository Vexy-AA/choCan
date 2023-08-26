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


namespace chibios_rt {
template <int N>
class CustomizedThread : public BaseThread {
protected:
    THD_WORKING_AREA(wa, N);
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
};

class usbTransmitThread : public CustomizedThread<128>{
public:
  usbTransmitThread(
    ByteBuffer* rxUSB, 
    USBDriver* drvUSB, 
    STM32F405::Pins<6>& leds) : 
        CustomizedThread<128>(), 
        mRxUSB(rxUSB),
        mDrvUsb(drvUSB),
        mLeds(leds){
  
  }
protected:
    void main(void) override;
private:
  ByteBuffer* mRxUSB;
  USBDriver* mDrvUsb;
  STM32F405::Pins<6>& mLeds;
};

class usbReceiveThread : public CustomizedThread<128>{
public:
  usbReceiveThread(
    ByteBuffer* rxUSB, 
    USBDriver* drvUSB, 
    STM32F405::Pins<6>& leds) : 
        CustomizedThread<128>(), 
        mRxUSB(rxUSB),
        mDrvUsb(drvUSB),
        mLeds(leds){
  
  }
protected:
    void main(void) override;
private:
  ByteBuffer* mRxUSB;
  USBDriver* mDrvUsb;
  STM32F405::Pins<6>& mLeds;
};

class ledsUpdateThread : public CustomizedThread<128>{
public:
    ledsUpdateThread(
      STM32F405::Pins<6>& leds) : 
        CustomizedThread<128>(), 
        mLeds(leds){
  
    }
protected:
    void main(void) override;
private:
    STM32F405::Pins<6>& mLeds;
};


}
