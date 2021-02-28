
#include <Arduino.h>

#include "Motor.h"

Motor::Motor()
    : mRelaySwitchFirstPin(-1)
    , mRelaySwitchSecondPin(-1)
{
}

void Motor::SetPin(const int8_t relaySwitchFirstPin, const int8_t relaySwitchSecondPin)
{
    if (relaySwitchFirstPin < 0 || relaySwitchSecondPin < 0) 
    {
        return;
    }

    pinMode(relaySwitchFirstPin, OUTPUT);
    pinMode(relaySwitchSecondPin, OUTPUT);
    
    mRelaySwitchFirstPin = relaySwitchFirstPin;
    mRelaySwitchSecondPin = relaySwitchSecondPin;

    Stop();
}


void Motor::MoveClockWise() const
{
    if (mRelaySwitchFirstPin < 0 || mRelaySwitchSecondPin < 0) 
    {
        return;
    }

    digitalWrite(mRelaySwitchFirstPin, HIGH);
    digitalWrite(mRelaySwitchSecondPin, HIGH);
}

void Motor::MoveCounterClockWise() const
{
    if (mRelaySwitchFirstPin < 0 || mRelaySwitchSecondPin < 0) 
    {
        return;
    }

    digitalWrite(mRelaySwitchFirstPin, LOW);
    digitalWrite(mRelaySwitchSecondPin, LOW);
}

void Motor::Stop() const
{
    if (mRelaySwitchFirstPin < 0 || mRelaySwitchSecondPin < 0) 
    {
        return;
    }
    
    digitalWrite(mRelaySwitchFirstPin, HIGH);
    digitalWrite(mRelaySwitchSecondPin, LOW);
}