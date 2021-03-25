
#include <Arduino.h>

#include "Configuration.h"
#include "Hardware.h"
#include "Motor.h"

Motor::Motor(int8_t firstPin, int8_t secondPin)
    : mFirstPin(firstPin)
    , mSecondPin(secondPin)
    , mThirdPin(-1)
    , mMode(eMotorMode::MOTOR_RELAY)
{
    pinMode(mFirstPin, OUTPUT);
    pinMode(mSecondPin, OUTPUT);
}

Motor::Motor(int8_t pwmPin, int8_t cwPin, int8_t ccwPin)
    : mFirstPin(pwmPin)
    , mSecondPin(cwPin)
    , mThirdPin(ccwPin)
    , mMode(eMotorMode::MOTOR_PWM)
{
    pinMode(pwmPin, OUTPUT);
    pinMode(cwPin, OUTPUT);
    pinMode(ccwPin, OUTPUT);
}

void Motor::MoveClockWise() const
{
    /*
    if (mFirstPin < 0 || mSecondPin < 0) 
    {
        return;
    }
    */

    if (mMode == eMotorMode::MOTOR_RELAY) 
    {
        digitalWrite(mFirstPin, HIGH);
        digitalWrite(mSecondPin, LOW);
    }
    else // eMotorMode::MOTOR_PWM
    {
        analogWrite(mFirstPin, 255);

        digitalWrite(mSecondPin, HIGH); 
        digitalWrite(mThirdPin, LOW);
    }
}

void Motor::MoveCounterClockWise() const
{
    /*
    if (mFirstPin < 0 || mSecondPin < 0) 
    {
        return;
    }
    */

    if (mMode == eMotorMode::MOTOR_RELAY) 
    {
        digitalWrite(mFirstPin, LOW);
        digitalWrite(mSecondPin, HIGH);
    }
    else // eMotorMode::MOTOR_PWM
    {
        analogWrite(mFirstPin, 255);

        digitalWrite(mSecondPin, LOW); 
        digitalWrite(mThirdPin, HIGH);
    }
}

void Motor::Stop() const
{
    /*
    if (mFirstPin < 0 || mSecondPin < 0) 
    {
        return;
    }
    */
    
    if (mMode == eMotorMode::MOTOR_RELAY)
    {
        digitalWrite(mFirstPin, LOW);
        digitalWrite(mSecondPin, LOW);
    }
    else // eMotorMode::MOTOR_PWM
    {
        analogWrite(mFirstPin, 0);
    }
}