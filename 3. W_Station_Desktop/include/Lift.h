#ifndef LIFT_H
#define LIFT_H

#include "IrSensor.h"
#include "LevelSwitch.h"
#include "Hardware.h"
#include "Motor.h"

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
    void StopElevateMotor();

    void UpdateCurrentFloor();
    eFloor GetCurrentFloor() const;

    String GetLiftStatus() const;
    

    String GetLiftItemStatus() const;
    void SetLiftItemStatus(const char* status);

    String GetLiftMotorStatus() const;

    bool IsItemPassed() const;

    bool IsJamesParked() const;
    void UpdateIsJamesParked(bool flag);
    

private:
    void MoveUp() const;
    void MoveDown() const;
    void ElevateLift(eFloor targetFloor);

    void SetLiftStatus(const char* status);
    void SetLiftMotorStatus(const char* status);

public:
    static const char* LIFT_STATUS_ARRIVED;
    static const char* LIFT_STATUS_WAIT;
    static const char* LIFT_STATUS_MOVE;

    static const char* LIFT_ITEM_STATUS_NONE;
    static const char* LIFT_ITEM_STATUS_EXIST;

    static const char* LIFT_MOTOR_STATUS_MOVE;
    static const char* LIFT_MOTOR_STATUS_STOP;

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

#endif /* LIFT_H */