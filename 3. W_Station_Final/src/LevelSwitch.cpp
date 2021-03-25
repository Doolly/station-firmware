#include "LevelSwitch.h"

LevelSwitch::LevelSwitch(uint8_t readPin)
    : mReadPin(readPin)
{
}

bool LevelSwitch::GetState() const
{
    uint16_t value;

    if (mReadPin < 0) 
    {
        return false;
    }

    value = analogRead(mReadPin);

    // value : 0 ~ 1024, (0V ~ 5V) -> [675 =, 3.3V]
    return (value > 675) ? true : false;
}
