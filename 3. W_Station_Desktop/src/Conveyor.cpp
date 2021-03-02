#include <assert.h>

#include "Conveyor.h"
#include "Motor.h"

const char* Conveyor::CONVEYOR_STATUS_MOVE = "move";
const char* Conveyor::CONVEYOR_STATUS_STOP = "stop";

Conveyor::Conveyor(const eFloor floor) 
    : mFloor(floor)
    , mConveyorStatus(CONVEYOR_STATUS_STOP)
    , mIsItemPassed(false)
{
    switch (floor)
    {
    case eFloor::FirstFloor:
        mMotor.SetPin(FLOOR1_MOTOR_RELAY_SWITCH1_PIN, FLOOR1_MOTOR_RELAY_SWITCH2_PIN);

        mIrSensorList[0].SetReadPin(FLOOR1_FIRST_IR_READ_PIN);
        mIrSensorList[1].SetReadPin(FLOOR1_SECOND_IR_READ_PIN);
        mIrSensorList[2].SetReadPin(FLOOR1_THIRD_IR_READ_PIN);
        mIrSensorList[3].SetReadPin(FLOOR1_FOURTH_IR_READ_PIN);
        break;

    case eFloor::SecondFloor:
        mMotor.SetPin(FLOOR2_MOTOR_RELAY_SWITCH1_PIN, FLOOR2_MOTOR_RELAY_SWITCH2_PIN);

        mIrSensorList[0].SetReadPin(FLOOR2_FIRST_IR_READ_PIN);
        mIrSensorList[1].SetReadPin(FLOOR2_SECOND_IR_READ_PIN);
        mIrSensorList[2].SetReadPin(FLOOR2_THIRD_IR_READ_PIN);
        mIrSensorList[3].SetReadPin(FLOOR2_FOURTH_IR_READ_PIN);
        break;

    case eFloor::ThirdFloor:        
        mMotor.SetPin(FLOOR3_MOTOR_RELAY_SWITCH1_PIN, FLOOR3_MOTOR_RELAY_SWITCH2_PIN);

        mIrSensorList[0].SetReadPin(FLOOR3_FIRST_IR_READ_PIN);
        mIrSensorList[1].SetReadPin(FLOOR3_SECOND_IR_READ_PIN);
        mIrSensorList[2].SetReadPin(FLOOR3_THIRD_IR_READ_PIN);
        mIrSensorList[3].SetReadPin(FLOOR3_FOURTH_IR_READ_PIN);
        break;

    default:
        break;
    }
}

void Conveyor::MoveRight()
{
    SetState(CONVEYOR_STATUS_MOVE);
    mMotor.MoveClockWise();
}

void Conveyor::MoveLeft()
{
    SetState(CONVEYOR_STATUS_MOVE);
    mMotor.MoveCounterClockWise();
}

void Conveyor::Stop()
{
    SetState(CONVEYOR_STATUS_STOP);
    mMotor.Stop();
}


uint8_t Conveyor::GetIrSensorState(uint8_t index) const
{
    assert(index < MAX_IR_SENSOR_COUNT);

    return mIrSensorList[index].GetState();
}

String Conveyor::GetState() const
{
    return mConveyorStatus;
}

bool Conveyor::GetIsItemPassed() const
{
    return mIsItemPassed;
}

void Conveyor::SetIsItemPassed(bool isPassed)
{
    mIsItemPassed = isPassed;
}

/* private */
void Conveyor::SetState(const char* status)
{
    mConveyorStatus = status;
}