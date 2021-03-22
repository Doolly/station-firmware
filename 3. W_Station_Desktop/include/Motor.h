#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

enum class eMotorMode {
    NONE,
    MOTOR_PWM,
    MOTOR_RELAY
};

class Motor 
{
public:
    Motor() = delete;
    Motor(uint8_t firstPin, uint8_t secondPin, eMotorMode motorMode);
    Motor(eFloor floor);
    virtual ~Motor() = default;

    //void SetPwmPin(const int8_t pwmPin, const int8_t dirPin);
    //void SetRelayPin(const int8_t relaySwitchFirstPin, const int8_t relaySwitchSecondPin);

    void MoveClockWise() const;
    void MoveCounterClockWise() const;
    void Stop() const;

private:
    uint8_t mFirstPin;
    uint8_t mSecondPin;
    eMotorMode mMode;
};

#endif /* MOTOR_H */