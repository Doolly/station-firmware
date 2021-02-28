#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <Arduino.h>
#include <std_msgs/Int8MultiArray.h>

#include "Configuration.h"
#include "IrSensor.h"
#include "Motor.h"
#include "Hardware.h"

#define CONVEYOR_STATUS_MOVE        ("move")
#define CONVEYOR_STATUS_STOP        ("stop")

class Conveyor
{
public:
    Conveyor() = delete;
    Conveyor(const eFloor floor);
    virtual ~Conveyor() = default;

    void MoveRight();
    void MoveLeft();
    void Stop();
    
    uint8_t GetIrSensorState(uint8_t index) const;
    String GetState() const;

    bool GetIsItemPassed() const;
    void SetIsItemPassed(bool isPassed);

private:
    void SetState(String status);

private:
    const eFloor mFloor;

    IrSensor mIrSensorList[MAX_IR_SENSOR_COUNT];
    Motor mMotor;

    String mConveyorStatus;

    bool mIsItemPassed;
};

extern Conveyor conveyorList[MAX_FLOOR_COUNT];

#endif /* CONVEYOR_H */