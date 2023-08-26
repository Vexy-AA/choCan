#pragma once

#ifndef DEVICESPECIFIC_HPP
#define DEVICESPECIFIC_HPP


//#include "stm32f405xx.h"
#include <vector>
#include "inline.hpp"
#include "pin.hpp"



/* #define onTestPin LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9)
#define offTestPin LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9)

#define getTim2 LL_TIM_GetCounter(TIM2)
#define getTim3 LL_TIM_GetCounter(TIM4)

#define digitsUpdate LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);\
                    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);\
                    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);\
                     LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7) */

typedef enum
{
    ledGreen,
    ledRed,
    ledBlue,
    ledYellow,
    ledWhite,
    ledGreen2,
    totalNumber
} tPinNames;

typedef struct{
    tPinNames  pinNumber;
    stm32_gpio_t* port;
    uint32_t pin;
} tPinDef;

const tPinDef ledsBoard[] =
{
  {tPinNames::ledGreen,     GPIOC, 1},
  {tPinNames::ledRed,       GPIOC, 3},
  {tPinNames::ledBlue,      GPIOA, 1},
  {tPinNames::ledYellow,    GPIOA, 3},
  {tPinNames::ledWhite,     GPIOC, 5},
  {tPinNames::ledGreen2,    GPIOB, 1}
};




static void setPad(stm32_gpio_t* port, uint32_t pin){
    palSetPad(port,pin);
}
static void clearPad(stm32_gpio_t* port, uint32_t pin){
    palClearPad(port,pin);
}

static uint32_t readOutputPad(stm32_gpio_t* port, uint32_t pin){
    return  (((palReadLatch(port) >> (pin)) & 1U));
}

static uint32_t readInputPad(stm32_gpio_t* port, uint32_t pin){
    return  (((palReadPort(port) >> (pin)) & 1U));
}

namespace STM32F405{






class Pin : public PinDriver::Pin<stm32_gpio_t *, uint32_t>
{
using Parent = PinDriver::Pin<stm32_gpio_t *, uint32_t>; 
public:
    Pin(tPinNames id,
        stm32_gpio_t * gpio,
        uint32_t pin,
        bool invert=false) :
            Parent(
                static_cast<uint8_t>(id),
                gpio,
                pin,
                setPad,
                clearPad,
                readOutputPad,
                invert)
    {
    }
    Pin(tPinNames id,
        stm32_gpio_t * gpio,
        uint32_t pin,
		uint16_t pulse,
        bool invert = false
        ) :
            Parent(
                static_cast<uint8_t>(id),
                gpio,
                pin,
                setPad,
                clearPad,
                readOutputPad,
                invert,
                pulse)
    {
    }
    Pin(tPinNames id,
        stm32_gpio_t * gpio,
        uint32_t pin,
        uint32_t readPin (stm32_gpio_t *, uint32_t),
        bool invert=false) :
            Parent(
                static_cast<uint8_t>(id),
                gpio,
                pin,
                setPad,
                clearPad,
                readPin,
                invert)
    {
    }
};




template <int number>
class Pins
{
private:

    uint8_t mPinsNumber = number;

public:
    std::vector<STM32F405::Pin> mPins;
    //bool status[number];
    uint8_t status = 0;
    Pins()
    {
        mPins.reserve(number);
    }
    void add(const tPinDef* pin, tPinNames pinNumber){
        for (uint8_t i= 0; i < pinNumber; i++){
            mPins.push_back(Pin(pin[i].pinNumber, pin[i].port, pin[i].pin));
        }
    }
    void add(const tPinDef* pin, uint32_t pinNumber){
        for (uint8_t i= 0; i < pinNumber; i++){
            mPins.push_back(Pin(pin[i].pinNumber, pin[i].port, pin[i].pin));
        }
    }
    void add(tPinDef pin){
        mPins.push_back(Pin(pin.pinNumber, pin.port, pin.pin));
    }
    void add(tPinNames id, stm32_gpio_t* gpio, uint32_t pin){
        mPins.push_back(Pin(id,gpio,pin));
    }
    void add(tPinNames id, stm32_gpio_t* gpio, uint32_t pin,bool invert){
        mPins.push_back(Pin(id,gpio,pin,invert));
    }

    void addInput(tPinDef pin){
        mPins.push_back(Pin(pin.pinNumber, pin.port, pin.pin,readInputPad));
    }
    void addInput(tPinNames id, stm32_gpio_t* gpio, uint32_t pin){
        mPins.push_back(Pin(id,gpio,pin,readInputPad));
    }
    void addInput(tPinNames id, stm32_gpio_t* gpio, uint32_t pin,bool invert){
        mPins.push_back(Pin(id,gpio,pin,readInputPad,invert));
    }
    
    uint8_t getStatus(){
        for (uint8_t i= 0; i < mPins.size(); i++){
            if (mPins.at(i).read())
                status |= 1 << mPins.at(i).getId();
            else
                status &= ~(1 << mPins.at(i).getId());
        }
        return status;
    }
    bool getStatus(tPinNames id){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                return pin.read();
            }
        }
        return false;
    }
    void on(){
        for (Pin& pin : mPins){
            pin.on();
        }
    }
    bool on(tPinNames id){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                pin.on();
                return true;
            }
        }
        return false;
    }
    void off(){
        for (Pin& pin : mPins){
            pin.off();
        }
    }
    void toggle(){
        for (Pin& pin : mPins){
            pin.toggle();
        }
    }
    bool off(tPinNames id){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                pin.off();
                return true;
            }
        }
        return false;
    }
    bool on(tPinNames id, uint32_t currentTime){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                pin.on(currentTime);
                return true;
            }
        }
        return false;
    }
    bool on(tPinNames id, uint16_t pulseTime, uint32_t currentTime){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                pin.on(pulseTime, currentTime);
                return true;
            }
        }
        return false;
    }
    void clock(uint32_t currentTime){
        for (Pin& pin : mPins){
            pin.clock(currentTime);
        }
    }
};










}




#endif
