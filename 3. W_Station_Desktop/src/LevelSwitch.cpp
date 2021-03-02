#include "LevelSwitch.h"

LevelSwitch::LevelSwitch()
    : mReadPin(-1)
{

}

void LevelSwitch::SetReadPin(int8_t readPin)
{
    if (readPin < 0) 
    {
        return;
    }

    pinMode(readPin, INPUT_PULLUP);
    mReadPin = readPin;
}

bool LevelSwitch::GetState() const
{
    if (mReadPin < 0) 
    {
        return false;
    }

    return digitalRead(mReadPin);
}
