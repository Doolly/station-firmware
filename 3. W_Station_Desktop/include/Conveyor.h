#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <Arduino.h>

#include "Configuration.h"
#include "IrSensor.h"
#include "Motor.h"
#include "Hardware.h"

enum class eConveyorStatus {
    STOP,
    LEFT,
    RIGHT
};

class Conveyor
{
public:
    Conveyor() = delete;
    Conveyor(uint8_t motorFirstPin, uint8_t motorSecondPin);
    virtual ~Conveyor() = default;

    void MoveRight();
    void MoveLeft();
    void Stop();
    
    eConveyorStatus GetStatus() const;

    bool IsItemPassed() const;
    void SetItemPassed(bool bPassed);

public:
    static const String conveyorStatusList[MAX_CONVEYOR_STATUS_LIST];

private:
    Motor mMotor;
    
    const eFloor mFloor;
    eConveyorStatus mConveyorStatus;
    bool mbItemPassed;
};

#endif /* CONVEYOR_H */