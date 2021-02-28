#ifndef LIFT_H
#define LIFT_H

#include "IrSensor.h"
#include "LevelSwitch.h"
#include "Hardware.h"
#include "Motor.h"

#define LIFT_STATUS_ARRIVED             ("arrived")
#define LIFT_STATUS_WAIT                ("wait")
#define LIFT_STATUS_MOVE                ("move")
#define LIFT_ITEM_STATUS_NONE           ("none")
#define LIFT_ITEM_STATUS_EXIST          ("exist")
#define LIFT_MOTOR_STATUS_MOVE          ("move")
#define LIFT_MOTOR_STATUS_STOP          ("stop")

class Lift 
{
public:
    Lift();
    virtual ~Lift() = default;

    void Initialize();
    
    void MoveLiftMotorToJames();
    void MoveLiftMotorToTray();
    void StopLiftMotor();

    bool MoveToFloor(eFloor floor);
    void StopElevateMotor() const;

    void UpdateCurrentFloor();
    eFloor GetCurrentFloor() const;

    String GetLiftStatus() const;
    void SetLiftStatus(const String status);

    String GetLiftItemStatus() const;
    void SetLiftItemStatus(const String status);

    String GetLiftMotorStatus() const;

    bool IsItemPassed() const;

    bool IsJamesParked() const;
    void UpdateIsJamesParked(bool flag);

private:
    void MoveUp() const;
    void MoveDown() const;
    void ElevateLift(eFloor targetFloor);
    void SetLiftMotorStatus(const String status);

private:
    /* hardware module */
    Motor mElevateMotor;
    Motor mLiftMotor;
    IrSensor mLiftIrSensor;
    LevelSwitch mLevelSwitchList[MAX_LEVEL_SWITCH_COUNT];

    eFloor mCurrentFloor;

    String mLiftStatus;
    String mLiftItemStatus;
    String mLiftMotorStatus;
    bool mIsJamesParked;
};

extern Lift lift;

#endif /* LIFT_H */