#include "Configuration.h"
#include "Lift.h"


/* initialize static variable */
const String Lift::liftStatusList[MAX_LIFT_STATUS_LIST] = {
    String("arrived"),
    String("up"),
    String("down")
};

Lift::Lift()
    : mElevateMotor(ELEVATE_MOTOR_PWM_PIN, ELEVATE_MOTOR_CW_PIN, ELEVATE_MOTOR_CCW_PIN)
    , mLiftConveyor(LIFT_CONVEYOR_MOTOR_RELAY_SWITCH1_PIN, LIFT_CONVEYOR_MOTOR_RELAY_SWITCH2_PIN)
    , mLevelSwitchList({ LevelSwitch(FLOOR1_LIMITED_SWITCH_READ_PIN), LevelSwitch(FLOOR2_LIMITED_SWITCH_READ_PIN), LevelSwitch(FLOOR3_LIMITED_SWITCH_READ_PIN), LevelSwitch(FLOOR3_CEILING_LIMITED_SWITCH_READ_PIN) })
    , mLiftIrSensor(LIFT_IR_READ_PIN, LIFT_IR_LED_PIN)
    , mCurrentFloor(eFloor::FirstFloor)
    , mLiftStatus(eLiftStatus::ARRIVED)
    , mbLiftItemStatus(false)
    , mbJamesParked(false)
{
    if (mLevelSwitchList[0].GetState() == true) 
    {
        mCurrentFloor = eFloor::FirstFloor;
    }
    else if (mLevelSwitchList[3].GetState() == true) 
    {
        mCurrentFloor = eFloor::ThirdFloor;
    }
    else 
    {
        if (mLevelSwitchList[1].GetState() == true && mLevelSwitchList[2].GetState() == true)
        {
            
        }
        else
        {

        }
        
        mCurrentFloor = eFloor::SecondFloor;
    }

   /* setting lift break */
   pinMode(ELEVATE_MOTOR_BREAK_PIN, OUTPUT);
   analogWrite(ELEVATE_MOTOR_BREAK_PIN, ANALOG_LOW);
}

void Lift::Reset()
{
    mLiftConveyor.Stop();
    StopElevateMotor();
    mLiftStatus = eLiftStatus::ARRIVED;
    mbLiftItemStatus = false;
}

bool Lift::MoveToFloor(eFloor targetFloor)
{
    /* Define move up & down */ 
    if (mCurrentFloor < targetFloor) 
    {
        mLiftStatus = eLiftStatus::UP;
    }
    else if (mCurrentFloor > targetFloor)
    {
        mLiftStatus = eLiftStatus::DOWN;
    }
    else 
    {
        mLiftStatus = eLiftStatus::ARRIVED;
    }

    switch (targetFloor)
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

    return true;
}

void Lift::StopElevateMotor()
{
    mLiftStatus = eLiftStatus::ARRIVED;
    mElevateMotor.Stop();

    /* break on */
    analogWrite(ELEVATE_MOTOR_BREAK_PIN, ANALOG_LOW);
}

void Lift::UpdateCurrentFloor()
{
    /* Check current floor! */
    uint8_t floor = static_cast<uint8_t>(mCurrentFloor);

    // LevelSwitch & floor starts at 0
    if (mLiftStatus == eLiftStatus::UP) 
    {
        if (mLevelSwitchList[floor + 2].GetState() == true) 
        {
            mCurrentFloor = static_cast<eFloor>(floor + 1);
        }
    }
    else if (mLiftStatus == eLiftStatus::DOWN)
    {
        if (mLevelSwitchList[floor - 1].GetState() == true) 
        {
            mCurrentFloor = static_cast<eFloor>(floor - 1);
        }
    }
}

Conveyor& Lift::GetConveyor()
{
    return mLiftConveyor;
}

eFloor Lift::GetCurrentFloor() const
{
    return mCurrentFloor;
}

eLiftStatus Lift::GetLiftStatus() const
{
    return mLiftStatus;
}

bool Lift::GetLiftItemStatus() const
{
    return mbLiftItemStatus;
}

void Lift::SetLiftItemStatus(bool status)
{
    mbLiftItemStatus = status;
}

bool Lift::GetIrStatus() const
{
    return mLiftIrSensor.GetState();
}

int8_t Lift::GetLevelSwitchStatus(uint8_t index) const
{
    return mLevelSwitchList[index].GetState();
}

bool Lift::IsJamesParked() const
{
    return mbJamesParked;
}

void Lift::UpdateJamesParked(bool bJamesParked)
{
    mbJamesParked = bJamesParked;
}

void Lift::EmergencyStop()
{
    
    mElevateMotor.Stop();
    mLiftConveyor.EmergencyStop();
}

/* private */
void Lift::MoveUp() const
{
    analogWrite(ELEVATE_MOTOR_BREAK_PIN, ANALOG_HIGH);
    mElevateMotor.MoveClockWise();
}

void Lift::MoveDown() const
{
    analogWrite(ELEVATE_MOTOR_BREAK_PIN, ANALOG_HIGH);
    mElevateMotor.MoveCounterClockWise();
}

void Lift::ElevateLift(eFloor targetFloor)
{
    if (targetFloor == mCurrentFloor)
    {
        StopElevateMotor();
    } 
    else if (targetFloor < mCurrentFloor) 
    {
        /* break off */
        MoveDown();
    }
    else
    {
        /* break off */
        MoveUp();
    }
}

