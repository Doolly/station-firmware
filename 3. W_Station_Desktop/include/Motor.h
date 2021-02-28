#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

class Motor 
{
public:
    Motor();
    virtual ~Motor() = default;

    void SetPin(const int8_t relaySwitchFirstPin, const int8_t relaySwitchSecondPin);

    void MoveClockWise() const;
    void MoveCounterClockWise() const;
    void Stop() const;

private:
    int8_t mRelaySwitchFirstPin;
    int8_t mRelaySwitchSecondPin;
};

#endif /* MOTOR_H */