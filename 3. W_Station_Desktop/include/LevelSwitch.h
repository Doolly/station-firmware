#ifndef LEVEL_SWITCH_H
#define LEVEL_SWITCH_H

#include <Arduino.h>

class LevelSwitch
{
public:
    LevelSwitch();
    virtual ~LevelSwitch() = default;

    void SetReadPin(int8_t readPin);
    bool GetState() const;

private:
    int8_t mReadPin;
};

#endif /* LEVEL_SWITCH_H */