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

    void MoveRight() const;
    void MoveLeft() const;
    void Stop() const;
    
    uint8_t GetIrSensorState(uint8_t index) const;

private:
    const eFloor mFloor;

    IrSensor mIrSensorList[MAX_IR_SENSOR_COUNT];
    Motor mMotor;
};

extern Conveyor conveyorList[MAX_FLOOR_COUNT];

#endif /* CONVEYOR_H */