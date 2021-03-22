#ifndef LEVEL_SWITCH_H
#define LEVEL_SWITCH_H

#include <Arduino.h>

class LevelSwitch
{
public:
    LevelSwitch() = delete;
    LevelSwitch(uint8_t readPin);
    virtual ~LevelSwitch() = default;

    //void SetReadPin(uint8_t readPin);
    bool GetState() const;

private:
    uint8_t mReadPin;
};

#endif /* LEVEL_SWITCH_H */