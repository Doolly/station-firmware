
#include <Arduino.h>

#include "Configuration.h"
#include "Hardware.h"
#include "Motor.h"

Motor::Motor(uint8_t firstPin, uint8_t secondPin, eMotorMode motorMode)
    : mFirstPin(firstPin)
    , mSecondPin(secondPin)
    , mMode(motorMode)
{
    pinMode(mFirstPin, OUTPUT);
    pinMode(mSecondPin, OUTPUT);
}

Motor::Motor(eFloor floor)
    : mMode(eMotorMode::MOTOR_RELAY)
{
    switch (floor)
    {
    case eFloor::FirstFloor:
        mFirstPin = FLOOR1_MOTOR_RELAY_SWITCH1_PIN;
        mSecondPin = FLOOR1_MOTOR_RELAY_SWITCH2_PIN;
        break;

    case eFloor::SecondFloor:
        mFirstPin = FLOOR2_MOTOR_RELAY_SWITCH1_PIN;
        mSecondPin = FLOOR2_MOTOR_RELAY_SWITCH2_PIN;
        break;

    case eFloor::ThirdFloor:        
        mFirstPin = FLOOR3_MOTOR_RELAY_SWITCH1_PIN;
        mSecondPin = FLOOR3_MOTOR_RELAY_SWITCH2_PIN;
        break;

    default:
        break;
    }
    
    pinMode(mFirstPin, OUTPUT);
    pinMode(mSecondPin, OUTPUT);
}

/*
void Motor::SetPwmPin(const int8_t pwmPin, const int8_t dirPin) 
{
    if (pwmPin < 0 || dirPin < 0) 
    {
        return;
    }

    if (mMode == eMotorMode::MOTOR_RELAY) 
    {
        return;
    }

    mMode = eMotorMode::MOTOR_PWM;
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    mFirstPin = pwmPin;
    mSecondPin = dirPin;
}

void Motor::SetRelayPin(const int8_t relaySwitchFirstPin, const int8_t relaySwitchSecondPin)
{
    if (relaySwitchFirstPin < 0 || relaySwitchSecondPin < 0) 
    {
        return;
    }

    if (mMode == eMotorMode::MOTOR_PWM) 
    {
        return;
    }

    mMode = eMotorMode::MOTOR_RELAY;
    pinMode(relaySwitchFirstPin, OUTPUT);
    pinMode(relaySwitchSecondPin, OUTPUT);
    
    mFirstPin = relaySwitchFirstPin;
    mSecondPin = relaySwitchSecondPin;
}
*/

void Motor::MoveClockWise() const
{
    if (mFirstPin < 0 || mSecondPin < 0) 
    {
        return;
    }

    if (mMode == eMotorMode::MOTOR_RELAY) 
    {
        digitalWrite(mFirstPin, HIGH);
        digitalWrite(mSecondPin, LOW);
    }
    else 
    {
        analogWrite(mFirstPin, 255);
        //digitalWrite(mSecondPin, HIGH); //Set direction
    }
}

void Motor::MoveCounterClockWise() const
{
    if (mFirstPin < 0 || mSecondPin < 0) 
    {
        return;
    }

    if (mMode == eMotorMode::MOTOR_RELAY) 
    {
        digitalWrite(mFirstPin, LOW);
        digitalWrite(mSecondPin, HIGH);
    }
    else 
    {
        analogWrite(mFirstPin, 255);
        //digitalWrite(mSecondPin, LOW); //Set direction
    }
}

void Motor::Stop() const
{
    if (mFirstPin < 0 || mSecondPin < 0) 
    {
        return;
    }
    
    if (mMode == eMotorMode::MOTOR_RELAY)
    {
        digitalWrite(mFirstPin, HIGH);
        digitalWrite(mSecondPin, HIGH);
    }
    else 
    {
        analogWrite(mFirstPin, 0);
    }
}