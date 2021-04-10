#include "LevelSwitch.h"

LevelSwitch::LevelSwitch(uint8_t readPin)
    : mReadPin(readPin)
{
    pinMode(readPin, INPUT);
}

/*
void LevelSwitch::SetReadPin(uint8_t readPin)
{
    if (readPin < 0) 
    {
        return;
    }

    pinMode(readPin, INPUT);
    mReadPin = readPin;
}
*/

bool LevelSwitch::GetState() const
{
    if (mReadPin < 0) 
    {
        return false;
    }

    return digitalRead(mReadPin);
}
