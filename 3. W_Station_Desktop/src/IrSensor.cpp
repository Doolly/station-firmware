#include "IrSensor.h"

IrSensor::IrSensor(uint8_t readPin, uint8_t lightPin)
    : mReadPin(readPin)
    , mLightPin(lightPin)
    , mbLightOn(false)
{
    pinMode(readPin, INPUT_PULLUP);
}

/*
void IrSensor::SetReadPin(const uint8_t readPin)
{
    if (readPin < 0)
    {
        return;    
    }

    pinMode(readPin, INPUT_PULLUP);
    mReadPin = readPin;
}
*/

/*
void IrSensor::SetLightPin(uint8_t lightPin)
{
    if (lightPin < 0)
    {
        return;
    }

    pinMode(lightPin, OUTPUT);
    mLightPin = lightPin;
}
*/

bool IrSensor::GetLight() const
{
    return mbLightOn;
}

void IrSensor::SetLight(const bool state)
{
    if (mReadPin < 0) 
    {
        return;
    }

    if (mbLightOn == state) {
        return;
    }

    mbLightOn = state;
    digitalWrite(mLightPin, state);
}

bool IrSensor::GetState() const
{
    if (mReadPin < 0) 
    {
        return false;
    }

    return digitalRead(mReadPin);
}
