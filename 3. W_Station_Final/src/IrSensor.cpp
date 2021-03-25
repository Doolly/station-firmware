#include "IrSensor.h"

IrSensor::IrSensor(int8_t readPin, int8_t lightPin)
    : mReadPin(readPin)
    , mLightPin(lightPin)
{
    pinMode(readPin, INPUT_PULLUP);
    pinMode(lightPin, OUTPUT);
}

void IrSensor::SetLight(const bool state)
{
    if (mLightPin < 0) 
    {
        return;
    }

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
