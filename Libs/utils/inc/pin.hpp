#pragma once

#ifndef PIN_HPP
#define PIN_HPP

#include <stdint.h>
#include "inline.hpp"

namespace PinDriver{

typedef enum
{
  LOW = 0,
  HIGH = !LOW
} State;

typedef enum
{
  IDLE,
  SHORT_PRESS,
  LONG_PRESS,
  DOUBLE_PRESS
} PressType;



template<typename gpioType,typename pinType>
class Pin
{
protected:

	using validateSetPin = void (*)(gpioType,pinType);
	using validateResetPin = void (*)(gpioType,pinType);
	using validateReadPin = uint32_t (*)(gpioType,pinType);
private:
    uint8_t mId;
	gpioType mGpio;
	pinType mPin;
	validateSetPin pSetPin;
	validateResetPin pResetPin;
	validateReadPin pReadPin;
    uint32_t mReleaseTime;
protected:
	bool mInvert = false;
	uint16_t mPulseTime = 1000;
	State mState;

public:
    Pin(	uint8_t id,
            gpioType gpio,
    		pinType pin,
			validateSetPin setpin,
			validateResetPin resetpin,
			validateReadPin readpin,
            bool invert,
            uint16_t pulse=0);


    __forceinlinemod void on();
    __forceinlinemod void on(uint32_t& currentTime);
    __forceinlinemod void on(uint16_t& pulseTime, uint32_t& currentTime);
    __forceinlinemod void off();
    __forceinlinemod bool read();
    __forceinlinemod uint8_t getId();
    __forceinlinemod State getState();
    void toggle();
    void clock(uint32_t& ticks);



};

/*****************************************************************************
    * Function Name: Constructor
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
Pin<gpioType,pinType>::Pin(	uint8_t id,
                            gpioType gpio,
                            pinType pin,
                            validateSetPin setpin,
                            validateResetPin resetpin,
                            validateReadPin readpin,
                            bool invert,
                            uint16_t pulse)
                            :   mId(id),
                                mGpio(gpio),
                                mPin(pin),
                                pSetPin(setpin),
                                pResetPin(resetpin),
								pReadPin(readpin),
                                mInvert(invert),
                                mPulseTime(pulse){
    read();
}
/*****************************************************************************
    * Function Name: on()
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
__forceinlinemod void Pin<gpioType,pinType>::clock(uint32_t& ticks){
    if ((mReleaseTime > 0) && (ticks > mReleaseTime)){
        off();
    }
}
/*****************************************************************************
    * Function Name: on()
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
__forceinlinemod void Pin<gpioType,pinType>::on(uint32_t& currentTime){
	(mInvert)? pResetPin(mGpio,mPin) : pSetPin(mGpio,mPin);
    mState = (mInvert)? LOW : HIGH;
    mReleaseTime = currentTime + mPulseTime;
}
/*****************************************************************************
    * Function Name: on()
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
__forceinlinemod void Pin<gpioType,pinType>::on(uint16_t& pulseTime, uint32_t& currentTime){
	(mInvert)? pResetPin(mGpio,mPin) : pSetPin(mGpio,mPin);
    mState = (mInvert)? LOW : HIGH;
    mReleaseTime = currentTime + pulseTime;
}
/*****************************************************************************
    * Function Name: on()
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
__forceinlinemod void Pin<gpioType,pinType>::on(){
	(mInvert)? pResetPin(mGpio,mPin) : pSetPin(mGpio,mPin);
    mState = (mInvert)? LOW : HIGH;
}
/*****************************************************************************
    * Function Name: off()
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
__forceinlinemod void Pin<gpioType,pinType>::off(){
	(mInvert)? pSetPin(mGpio,mPin) : pResetPin(mGpio,mPin);
    mState = (mInvert)? HIGH : LOW;
    mReleaseTime = 0;
}
/*****************************************************************************
    * Function Name: read()
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
__forceinlinemod bool Pin<gpioType,pinType>::read(){
    if (mInvert){
        if (pReadPin(mGpio,mPin)){
            mState = LOW;
            return false;
        }else{
            mState = HIGH;
            return true;
        }
    }else{
        if (pReadPin(mGpio,mPin)){
            mState = HIGH;
            return true;
        }else{
            mState = LOW;
            return false;
        }
    }
}
/*****************************************************************************
    * Function Name: toggle()
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
void Pin<gpioType,pinType>::toggle(){
    if (mInvert){
        if (mState == HIGH){
            mState = LOW;
            on();
        }else{
            mState = HIGH;
            off();
        }
    }else{
        if (mState == HIGH){
            mState = LOW;
            off();
        }else{
            mState = HIGH;
            on();
        }
    }
}
/*****************************************************************************
    * Function Name: toggle()
    * Description: 
    *  
    *************************************************************************/
template<typename gpioType,typename pinType>
__forceinlinemod uint8_t Pin<gpioType,pinType>::getId(){
    return mId;
}

template<typename gpioType,typename pinType>
__forceinlinemod State Pin<gpioType,pinType>::getState(){
    return mState;
}

}
#endif // PIN_HPP
