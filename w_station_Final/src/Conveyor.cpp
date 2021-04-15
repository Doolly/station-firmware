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
    : mMotor(motorFirstPin, motorSecondPin)
    , mFloor()
    , mConveyorStatus(eConveyorStatus::STOP)
    , mbItemPassed(false)
{
    // for lift conveyor    
}

Conveyor::Conveyor(uint8_t motorFirstPin, uint8_t motorSecondPin, eFloor floor)
    : mMotor(motorFirstPin, motorSecondPin)
    , mFloor(floor)
    , mConveyorStatus(eConveyorStatus::STOP)
    , mbItemPassed(false)
{
    // for floor conveyor
    switch (floor) 
    {
    case eFloor::FirstFloor:
        mIrSensors[0] = new IrSensor(FLOOR1_FIRST_IR_READ_PIN, -1);
        mIrSensors[1] = new IrSensor(FLOOR1_SECOND_IR_READ_PIN, -1);
        mIrSensors[2] = new IrSensor(FLOOR1_THIRD_IR_READ_PIN, -1);
        mIrSensors[3] = new IrSensor(FLOOR1_FOURTH_IR_READ_PIN, -1);
        mIrSensors[4] = new IrSensor(FLOOR1_FIFTH_IR_READ_PIN, -1);
        break;

    case eFloor::SecondFloor:
        mIrSensors[0] = new IrSensor(FLOOR2_FIRST_IR_READ_PIN, -1);
        mIrSensors[1] = new IrSensor(FLOOR2_SECOND_IR_READ_PIN, -1);
        mIrSensors[2] = new IrSensor(FLOOR2_THIRD_IR_READ_PIN, -1);
        mIrSensors[3] = new IrSensor(FLOOR2_FOURTH_IR_READ_PIN, -1);
        mIrSensors[4] = new IrSensor(FLOOR2_FIFTH_IR_READ_PIN, -1);
        break;

    case eFloor::ThirdFloor:
        mIrSensors[0] = new IrSensor(FLOOR3_FIRST_IR_READ_PIN, -1);
        mIrSensors[1] = new IrSensor(FLOOR3_SECOND_IR_READ_PIN, -1);
        mIrSensors[2] = new IrSensor(FLOOR3_THIRD_IR_READ_PIN, -1);
        mIrSensors[3] = new IrSensor(FLOOR3_FOURTH_IR_READ_PIN, -1);
        mIrSensors[4] = new IrSensor(FLOOR3_FIFTH_IR_READ_PIN, -1);
        break;

    default:
        break;
    }
}

Conveyor::~Conveyor()
{
    for (uint8_t index = 0; index < MAX_CONVEYOR_IR_SENSOR_COUNT; ++index)
    {
        delete mIrSensors[index];
    }
}

void Conveyor::MoveLeft()
{
    mConveyorStatus = eConveyorStatus::LEFT;
    mMotor.MoveClockWise();
}

void Conveyor::MoveRight()
{
    mConveyorStatus = eConveyorStatus::RIGHT;
    mMotor.MoveCounterClockWise();
}

void Conveyor::Stop()
{
    mConveyorStatus = eConveyorStatus::STOP;
    mMotor.Stop();
}

void Conveyor::EmergencyStop()
{
    mMotor.Stop();
}

eConveyorStatus Conveyor::GetStatus() const
{
    return mConveyorStatus;
}

bool Conveyor::GetIrSensorStatus(const uint8_t index) const
{
    return mIrSensors[index]->GetState();
}

bool Conveyor::IsItemPassed() const
{
    return mbItemPassed;
}

void Conveyor::SetItemPassed(bool bPassed)
{
    mbItemPassed = bPassed;
}