#include "Configuration.h"
#include "Lift.h"

Lift lift;

Lift::Lift()
    : mCurrentFloor(eFloor::FirstFloor) 
    , mLiftStatus(LIFT_STATUS_WAITING)
    , mLiftItemStatus(false)
    , mIsJamesParked(false)
{
    mElevateMotor.SetPin(ELEVATE_MOTOR_RELAY_SWITCH1_PIN, ELEVATE_MOTOR_RELAY_SWITCH2_PIN);
    mLiftMotor.SetPin(LIFT_CONVEYOR_MOTOR_RELAY_SWITCH1_PIN, LIFT_CONVEYOR_MOTOR_RELAY_SWITCH2_PIN);

    mLiftIrSensor.SetReadPin(LIFT_IR_READ_PIN);
    mLiftIrSensor.SetLightPin(LIFT_IR_LED_PIN);

    mLevelSwitchList[0].SetReadPin(FLOOR1_LIMITED_SWITCH_READ_PIN);
    mLevelSwitchList[1].SetReadPin(FLOOR2_LIMITED_SWITCH_READ_PIN);
    mLevelSwitchList[2].SetReadPin(FLOOR3_LIMITED_SWITCH_READ_PIN);

    for (uint8_t i = 0; i < MAX_LEVEL_SWITCH_COUNT; ++i)
    {
        if (mLevelSwitchList[i].GetState() == true)
        {
            mCurrentFloor = static_cast<eFloor>(i + 1);
            break;
        }
    }
}

void Lift::MoveLiftMotorLeft() const
{
    mLiftMotor.MoveClockWise();
}

void Lift::MoveLiftMotorRight() const
{
    mLiftMotor.MoveCounterClockWise();
}

void Lift::StopLiftMotor() const
{
    mLiftMotor.Stop();
}

bool Lift::MoveToFloor(eFloor floor)
{
    mLiftStatus = LIFT_STATUS_MOVING;

    switch (floor)
    {
    case eFloor::FirstFloor:
        ElevateLift(eFloor::FirstFloor, FLOOR1_LIMITED_SWITCH_READ_PIN);
        break;
    
    case eFloor::SecondFloor:
        ElevateLift(eFloor::SecondFloor, FLOOR2_LIMITED_SWITCH_READ_PIN);
        break;
    
    case eFloor::ThirdFloor:
        ElevateLift(eFloor::ThirdFloor, FLOOR3_LIMITED_SWITCH_READ_PIN);
        break;
    
    default:
        return false;
    }

    mCurrentFloor = floor;
    return true;
}

void Lift::StopElevateMotor() const
{
    mElevateMotor.Stop();
}

eFloor Lift::GetCurrentFloor() const
{
    return mCurrentFloor;
}

String Lift::GetLiftStatus() const
{
    return mLiftStatus;
}

String Lift::GetLiftItemStatus() const
{
    return mLiftItemStatus;
}

bool Lift::IsItemPassed() const
{
    return mLiftIrSensor.GetState();
}

bool Lift::IsJamesParked() const
{
    return mIsJamesParked;
}

void Lift::MoveUp() const
{
    mElevateMotor.MoveClockWise();
}

void Lift::MoveDown() const
{
    mElevateMotor.MoveCounterClockWise();
}

void Lift::ElevateLift(eFloor targetFloor, uint8_t limitedSwitchPin)
{
    if (mCurrentFloor == targetFloor)
    {
        StopElevateMotor();
        return;
    }

    if (targetFloor < mCurrentFloor) 
    {
        MoveDown();
    }
    else
    {
        MoveUp();
    }    
}