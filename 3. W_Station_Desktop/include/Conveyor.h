#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <Arduino.h>
#include <std_msgs/Int8MultiArray.h>

#include "Configuration.h"
#include "IrSensor.h"
#include "Motor.h"
#include "Hardware.h"

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

public:
    static const char* CONVEYOR_STATUS_MOVE;
    static const char* CONVEYOR_STATUS_STOP;

private:
    void SetState(const char* status);

private:
    const eFloor mFloor;

    IrSensor mIrSensorList[MAX_IR_SENSOR_COUNT];
    Motor mMotor;

    String mConveyorStatus;

    bool mIsItemPassed;
};

#endif /* CONVEYOR_H */