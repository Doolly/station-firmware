#include "Configuration.h"
#include "Lift.h"

Lift lift;

Lift::Lift()
    : mCurrentFloor(eFloor::FirstFloor) 
    , mLiftStatus(LIFT_STATUS_WAIT)
    , mLiftItemStatus(LIFT_ITEM_STATUS_NONE)
    , mLiftMotorStatus(LIFT_MOTOR_STATUS_STOP)
    , mIsJamesParked(false)
{
    mElevateMotor.SetPin(ELEVATE_MOTOR_RELAY_SWITCH1_PIN, ELEVATE_MOTOR_RELAY_SWITCH2_PIN);
    mLiftMotor.SetPin(LIFT_CONVEYOR_MOTOR_RELAY_SWITCH1_PIN, LIFT_CONVEYOR_MOTOR_RELAY_SWITCH2_PIN);

    mLiftIrSensor.SetReadPin(LIFT_IR_READ_PIN);
    mLiftIrSensor.SetLightPin(LIFT_IR_LED_PIN);

    mLevelSwitchList[0].SetReadPin(FLOOR1_LIMITED_SWITCH_READ_PIN);
    mLevelSwitchList[1].SetReadPin(FLOOR2_LIMITED_SWITCH_READ_PIN);
    mLevelSwitchList[2].SetReadPin(FLOOR3_LIMITED_SWITCH_READ_PIN);

    UpdateCurrentFloor();
}

void Lift::Initialize()
{
    StopLiftMotor();
    mLiftStatus = LIFT_STATUS_WAIT;
    mLiftItemStatus = LIFT_ITEM_STATUS_NONE;
}

void Lift::MoveLiftMotorToJames()
{
    if (IsJamesParked() == false) 
    {
        return;
    }

    SetLiftMotorStatus(LIFT_MOTOR_STATUS_MOVE);
    mLiftMotor.MoveClockWise();
}

void Lift::MoveLiftMotorToTray()
{
    SetLiftMotorStatus(LIFT_MOTOR_STATUS_MOVE);
    mLiftMotor.MoveCounterClockWise();
}

void Lift::StopLiftMotor()
{
    SetLiftMotorStatus(LIFT_MOTOR_STATUS_STOP);
    mLiftMotor.Stop();
}

bool Lift::MoveToFloor(eFloor floor)
{
    switch (floor)
    {
    case eFloor::FirstFloor:
        ElevateLift(eFloor::FirstFloor);
        break;
    
    case eFloor::SecondFloor:
        ElevateLift(eFloor::SecondFloor);
        break;
    
    case eFloor::ThirdFloor:
        ElevateLift(eFloor::ThirdFloor);
        break;
    
    default:
        return false;
    }

    SetLiftStatus(LIFT_STATUS_MOVE);
    return true;
}

void Lift::StopElevateMotor()
{
    SetLiftStatus(LIFT_STATUS_ARRIVED);
    mElevateMotor.Stop();
}

void Lift::UpdateCurrentFloor()
{
    for (uint8_t i = 0; i < MAX_LEVEL_SWITCH_COUNT; ++i)
    {
        if (mLevelSwitchList[i].GetState() == true)
        {
            mCurrentFloor = static_cast<eFloor>(i + 1);
            break;
        }
    }
}

eFloor Lift::GetCurrentFloor() const
{
    return mCurrentFloor;
}

String Lift::GetLiftStatus() const
{
    return mLiftStatus;
}

void Lift::SetLiftStatus(const char* status)
{
    mLiftStatus = status;
}

String Lift::GetLiftItemStatus() const
{
    return mLiftItemStatus;
}

void Lift::SetLiftItemStatus(const char* status)
{
    mLiftItemStatus = status;
}

String Lift::GetLiftMotorStatus() const
{
    return mLiftMotorStatus;
}

bool Lift::IsItemPassed() const
{
    return mLiftIrSensor.GetState();
}

bool Lift::IsJamesParked() const
{
    return mIsJamesParked;
}

void Lift::UpdateIsJamesParked(bool isJamesParked)
{
    mIsJamesParked = isJamesParked;
}

/* private */
void Lift::MoveUp() const
{
    mElevateMotor.MoveClockWise();
}

void Lift::MoveDown() const
{
    mElevateMotor.MoveCounterClockWise();
}

void Lift::ElevateLift(eFloor targetFloor)
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

void Lift::SetLiftMotorStatus(const char* status)
{
    mLiftMotorStatus = status;
}