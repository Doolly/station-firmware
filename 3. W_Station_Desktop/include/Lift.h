#ifndef LIFT_H
#define LIFT_H

#include "IrSensor.h"
#include "LevelSwitch.h"
#include "Hardware.h"
#include "Motor.h"

#define LIFT_STATUS_ARRIVED             ("arrived")
#define LIFT_STATUS_WAITING             ("waiting")
#define LIFT_STATUS_MOVING              ("moving")

class Lift 
{
public:
    Lift();
    virtual ~Lift() = default;
    
    void MoveLiftMotorLeft() const;
    void MoveLiftMotorRight() const;
    void StopLiftMotor() const;

    bool MoveToFloor(eFloor floor);
    void StopElevateMotor() const;

    eFloor GetCurrentFloor() const;
    String GetLiftStatus() const;
    String GetLiftItemStatus() const;

    bool IsItemPassed() const;

    bool IsJamesParked() const;

private:
    void MoveUp() const;
    void MoveDown() const;
    void ElevateLift(eFloor targetFloor, uint8_t limitedSwitchPin);

private:
    /* hardware module */
    Motor mElevateMotor;
    Motor mLiftMotor;
    IrSensor mLiftIrSensor;
    LevelSwitch mLevelSwitchList[MAX_LEVEL_SWITCH_COUNT];

    eFloor mCurrentFloor;

    String mLiftStatus;
    String mLiftItemStatus;
    bool mIsJamesParked;
};

extern Lift lift;

#endif /* LIFT_H */