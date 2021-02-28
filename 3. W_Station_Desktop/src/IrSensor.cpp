#include "IrSensor.h"

IrSensor::IrSensor()
    : mReadPin(-1)
    , mLightPin(-1)
    , mIsLightOn(false)
{

}

void IrSensor::SetReadPin(const int8_t readPin)
{
    if (readPin < 0)
    {
        return;    
    }

    pinMode(readPin, INPUT_PULLUP);
    mReadPin = readPin;
}

bool IrSensor::GetLight() const
{
    return mIsLightOn;
}

void IrSensor::SetLightPin(int8_t lightPin)
{
    if (lightPin < 0)
    {
        return;
    }

    pinMode(lightPin, OUTPUT);
    mLightPin = lightPin;
}

void IrSensor::SetLight(const bool state)
{
    if (mReadPin < 0) 
    {
        return;
    }

    if (mIsLightOn == state) {
        return;
    }

    mIsLightOn = state;
    digitalWrite(mLightPin, state);
}

bool IrSensor::GetState() const
{
    if (mReadPin < 0) 
    {
        return false;
    }

    return !digitalRead(mReadPin);
}
