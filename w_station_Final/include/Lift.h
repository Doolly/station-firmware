#ifndef LIFT_H
#define LIFT_H

#include "Conveyor.h"
#include "IrSensor.h"
#include "LevelSwitch.h"
#include "Hardware.h"

enum class eLiftStatus 
{
    ARRIVED,
    UP,
    DOWN
};

class Lift 
{
public:
    Lift();
    virtual ~Lift() = default;

    Conveyor& GetConveyor();
    eFloor GetCurrentFloor() const;
    eLiftStatus GetLiftStatus() const;
    bool GetLiftItemStatus() const;
    void SetLiftItemStatus(bool status);
    bool GetIrStatus() const;
    int8_t GetLevelSwitchStatus(uint8_t index) const;

    void Reset();
    bool MoveToFloor(eFloor targetFloor);
    bool MoveToFloorManual(eFloor targetFloor);
    void StopElevateMotor();
    void UpdateCurrentFloor();

    bool IsJamesParked() const;
    void UpdateJamesParked(bool bJamesParked);

    void EmergencyStop();

//private:
    void MoveUp() const;
    void MoveDown() const;
    void ElevateLift(eFloor targetFloor);

public:
    static const String liftStatusList[MAX_LIFT_STATUS_LIST];

private:
    /* hardware module */
    Motor mElevateMotor;
    Conveyor mLiftConveyor;
    LevelSwitch mLevelSwitchList[MAX_LEVEL_SWITCH_COUNT]; 
    IrSensor mLiftIrSensor;

    /*  state variable */
    eFloor mCurrentFloor;
    eLiftStatus mLiftStatus;

    bool mbLiftItemStatus;
    bool mbJamesParked;
};

#endif /* LIFT_H */