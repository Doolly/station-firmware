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
    Motor(int8_t firstPin, int8_t secondPin);
    Motor(int8_t pwmPin, int8_t cwPin, int8_t ccwPin);
    virtual ~Motor() = default;

    //void SetPwmPin(const int8_t pwmPin, const int8_t dirPin);
    //void SetRelayPin(const int8_t relaySwitchFirstPin, const int8_t relaySwitchSecondPin);

    void MoveClockWise() const;
    void MoveCounterClockWise() const;
    void Stop() const;

private:
    int8_t mFirstPin;
    int8_t mSecondPin;
    int8_t mThirdPin;
    eMotorMode mMode;
};

#endif /* MOTOR_H */