#include <assert.h>

#include "Conveyor.h"
#include "Motor.h"

/* initialize static variable */
const String Conveyor::conveyorStatusList[MAX_CONVEYOR_STATUS_LIST] = {
    String("stop"),
    String("left"),
    String("right")
};

Conveyor::Conveyor(uint8_t motorFirstPin, uint8_t motorSecondPin)
    : mMotor(motorFirstPin, motorSecondPin, eMotorMode::MOTOR_RELAY)
    , mFloor()
    , mConveyorStatus(eConveyorStatus::STOP)
    , mbItemPassed(false)
{    
}

void Conveyor::MoveRight()
{
    mConveyorStatus = eConveyorStatus::RIGHT;
    mMotor.MoveClockWise();
}

void Conveyor::MoveLeft()
{
    mConveyorStatus = eConveyorStatus::LEFT;
    mMotor.MoveCounterClockWise();
}

void Conveyor::Stop()
{
    mConveyorStatus = eConveyorStatus::STOP;
    mMotor.Stop();
}

eConveyorStatus Conveyor::GetStatus() const
{
    return mConveyorStatus;
}

bool Conveyor::IsItemPassed() const
{
    return mbItemPassed;
}

void Conveyor::SetItemPassed(bool bPassed)
{
    mbItemPassed = bPassed;
}