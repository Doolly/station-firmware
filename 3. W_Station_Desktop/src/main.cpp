#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Int8.h>

#include "Command.h"
#include "Conveyor.h"
#include "Configuration.h"
#include "Lift.h"
#include "TimerInterrupt.h"
#include "Hardware.h"

void IntializeLevelSwitchStatuses();

void PublishLiftCurrentFloor();
void PublishLiftStatus();
void PublishLiftItemStatus();
void PublishLiftIrStatus();
void PublishLiftConveyorStatus();
void PublishConveyorStatuses();
void PublishLevelSwitchStatuses();

void SubscribeLiftDestinationFloor(const std_msgs::Int8& floorOrNone);
void SubscribePushItemToLift(const std_msgs::String& flag);
void SubscribeSendToDestination(const std_msgs::String& dest);

void CheckLiftArrivedAtTargetFloor();
void CheckItemIsPushedToLift();
void CheckItemIsPushedToDestination();

void DebugLed1Toggle();
void DebugLed2Toggle();
void DebugLed3Toggle();
void DebugLed4Toggle();

/* ros - NodeHandle */
ros::NodeHandle nodeHandle;

/* ros - publisher */
std_msgs::Int8 liftCurrentFloor;
ros::Publisher publishLiftCurrentFloor("wstation/lift_current_floor", &liftCurrentFloor);

std_msgs::String liftStatus;
ros::Publisher publishLiftStatus("wstation/lift_status", &liftStatus);

std_msgs::Bool liftItemStatus;
ros::Publisher publishLiftItemStatus("wstation/lift_item_status", &liftItemStatus);

// not used currently in Ros core
std_msgs::Bool liftIrStatus; // "0", "1"
ros::Publisher publishLiftIrStatus("wstation/lift_ir_status", &liftIrStatus);

std_msgs::String liftConveyorStatus; // "left", "right", "stop"
ros::Publisher publishLiftConveyorStatus("wstation/lift_conveyor_status", &liftConveyorStatus);

std_msgs::String conveyorStatuses;
ros::Publisher publishConveyorStatuses("wstation/conveyor_status", &conveyorStatuses);

std_msgs::Int8MultiArray levelSwitchStatuses;
ros::Publisher publishLevelSwitchStatuses("wstation/limited_switch_status", &levelSwitchStatuses);

/* ros - subscriber */
ros::Subscriber<std_msgs::Int8> subscribeLiftDestinationFloor("wstation/lift_destination_floor", SubscribeLiftDestinationFloor); // "1", "2", "3"
ros::Subscriber<std_msgs::String> subscribePushItemToLift("wstation/push_item_to_lift", SubscribePushItemToLift); // "push", "none"
ros::Subscriber<std_msgs::String> subscribeSendToDestination("wstation/send_to_destination", SubscribeSendToDestination); // "james", "tray", "none"

/* global publisher status instance */
volatile bool gIsSubscribeLiftDestinationFloor = false;
volatile bool gIsSubscribePushItemToLift = false;
volatile bool gIsSubscribeSendToDestination = false;

/* global hardware instance */
Lift gLift;

Conveyor gConveyorList[MAX_FLOOR_COUNT] = {
    Conveyor(FLOOR1_MOTOR_RELAY_SWITCH1_PIN, FLOOR1_MOTOR_RELAY_SWITCH2_PIN), 
    Conveyor(FLOOR2_MOTOR_RELAY_SWITCH1_PIN, FLOOR2_MOTOR_RELAY_SWITCH2_PIN), 
    Conveyor(FLOOR3_MOTOR_RELAY_SWITCH1_PIN, FLOOR3_MOTOR_RELAY_SWITCH2_PIN)
};
//Conveyor* gCurrentConveyor = nullptr; //why?

eFloor gTargetFloor;
String gDestination;

bool gLed1Toggle = false;
bool gLed2Toggle = false;
bool gLed3Toggle = false;
bool gLed4Toggle = false;

void setup() 
{
    pinMode(DEBUG_LED1_PIN, OUTPUT);
    pinMode(DEBUG_LED2_PIN, OUTPUT);
    pinMode(DEBUG_LED3_PIN, OUTPUT);
    pinMode(DEBUG_LED4_PIN, OUTPUT);
    
    IntializeLevelSwitchStatuses();

    nodeHandle.getHardware()->setBaud(115200);
    nodeHandle.initNode();

    nodeHandle.advertise(publishLiftCurrentFloor);
    nodeHandle.advertise(publishLiftStatus);
    nodeHandle.advertise(publishLiftItemStatus);
    
    // Added
    nodeHandle.advertise(publishLiftIrStatus);
    nodeHandle.advertise(publishLiftConveyorStatus);    
    nodeHandle.advertise(publishConveyorStatuses);
    nodeHandle.advertise(publishLevelSwitchStatuses);

    nodeHandle.subscribe(subscribeLiftDestinationFloor);
    nodeHandle.subscribe(subscribePushItemToLift);
    nodeHandle.subscribe(subscribeSendToDestination);

    Timer1.initialize(PUBLISH_PERIOD_US);                
    Timer1.attachInterrupt(PublishISR);
}

void loop() 
{
    /*
    if (gbEmergency == false)
    {
        
    }
    */
    gLift.UpdateCurrentFloor();
    
    /* Publish current states */
    if (isPublishValidate == true) 
    {
        isPublishValidate = false;

        PublishLiftCurrentFloor();
        PublishLiftStatus();
        PublishLiftItemStatus();

        // Added
        PublishLiftIrStatus();
        PublishLiftConveyorStatus();
        PublishConveyorStatuses();
        PublishLevelSwitchStatuses();

        DebugLed1Toggle();
    }

    /* Subscribe */
    nodeHandle.spinOnce();

    /* subscribe -> lift_destination_floor */
    if (gIsSubscribeLiftDestinationFloor == true) 
    {
        gIsSubscribeLiftDestinationFloor = false;
        
        if (gLift.GetLiftStatus() == eLiftStatus::ARRIVED) 
        {
            DebugLed2Toggle();
            gLift.MoveToFloor(gTargetFloor);
        }
    }

    CheckLiftArrivedAtTargetFloor(); // If arrived, stop the motor

    /* subscribe -> push_item_to_lift */
    if (gIsSubscribePushItemToLift == true)
    {
        gIsSubscribePushItemToLift = false;

        if (gLift.GetLiftItemStatus() == true) 
        {
            gConveyorList[static_cast<uint8_t>(gLift.GetCurrentFloor())].Stop();
            gLift.GetConveyor().Stop();
        }
        else 
        {
            gConveyorList[static_cast<uint8_t>(gLift.GetCurrentFloor())].MoveLeft();
            gLift.GetConveyor().MoveLeft();
        }

        DebugLed3Toggle();
    }

    CheckItemIsPushedToLift();

    /* subscribe -> send_to_destination */
    if (gIsSubscribeSendToDestination == true)
    {
        gIsSubscribeSendToDestination = false;
        DebugLed4Toggle();

        if (gLift.GetLiftStatus() == eLiftStatus::ARRIVED && gLift.GetLiftItemStatus() == true)
        {
            if (gDestination == COMMAND_SEND_TO_JAMES)
            {
                if (gLift.IsJamesParked() == true)
                {
                    gLift.GetConveyor().MoveLeft();
                }
            }
            else // recieved "tray"
            {
                if (true)// TODO:  
                {
                    // alarm();
                }
                else 
                {
                    gLift.GetConveyor().MoveRight();
                    // TODO: 1층에 물건이 꽉 찾을 경우, 어떻게 할 것인지?
                    gConveyorList[0].MoveRight();
                }
                
            }
        }
    }

    CheckItemIsPushedToDestination();
}

void IntializeLevelSwitchStatuses()
{
    levelSwitchStatuses.data = (int8_t*)malloc(sizeof(int8_t) * MAX_LEVEL_SWITCH_COUNT);
    levelSwitchStatuses.data_length = MAX_LEVEL_SWITCH_COUNT;
}

/* Publisher */
void PublishLiftCurrentFloor()
{
    liftCurrentFloor.data = static_cast<int8_t>(gLift.GetCurrentFloor()) + 1;
    publishLiftCurrentFloor.publish(&liftCurrentFloor);
}

void PublishLiftStatus()
{
    char buf[MAX_BUFFER_SIZE];
    String currentLiftStatus = Lift::liftStatusList[static_cast<uint8_t>(gLift.GetLiftStatus())];
    uint8_t liftStatusLength = currentLiftStatus.length();

    for (uint8_t index = 0; index < liftStatusLength; ++index)
    {
        buf[index] = currentLiftStatus[index];
    }
    buf[liftStatusLength] = '\0';

    liftStatus.data = buf;
    publishLiftStatus.publish(&liftStatus);
}

void PublishLiftItemStatus()
{   
    liftItemStatus.data = gLift.GetLiftItemStatus();
    publishLiftItemStatus.publish(&liftItemStatus);
}

void PublishLiftIrStatus()
{
    liftIrStatus.data = gLift.GetIrStatus();
    publishLiftIrStatus.publish(&liftIrStatus);
}

void PublishLiftConveyorStatus()
{
    char buf[MAX_BUFFER_SIZE];
    String currentStatus = Conveyor::conveyorStatusList[static_cast<uint8_t>(gLift.GetConveyor().GetStatus())];
    uint8_t length = currentStatus.length();

    for (uint8_t index = 0; index < length; ++index)
    {
        buf[index] = currentStatus[index];
    }
    buf[length] = '\0';

    liftConveyorStatus.data = buf;
    publishLiftConveyorStatus.publish(&liftConveyorStatus);
}

void PublishConveyorStatuses()
{
    String statuses;
    char buf[MAX_BUFFER_SIZE];

    for (uint8_t index = 0; index < MAX_FLOOR_COUNT; ++index)
    {
        statuses += Conveyor::conveyorStatusList[static_cast<uint8_t>(gConveyorList[index].GetStatus())];
        statuses += ",";
    }

    strcpy(buf, statuses.c_str());
    buf[statuses.length() - 1] = '\0';

    conveyorStatuses.data = buf;
    publishConveyorStatuses.publish(&conveyorStatuses);
}

void PublishLevelSwitchStatuses()
{
    for (uint8_t index = 0; index < MAX_LEVEL_SWITCH_COUNT; ++index)
    {
        levelSwitchStatuses.data[index] = gLift.GetLevelSwitchStatus(index);
    }
    
    publishLevelSwitchStatuses.publish(&levelSwitchStatuses);
}

/* Subscriber Callback Function */
void SubscribeLiftDestinationFloor(const std_msgs::Int8& floorOrNone)
{
    if (floorOrNone.data <= 0)
    {
        return;
    }

    gTargetFloor = static_cast<eFloor>(floorOrNone.data - 1);
    gIsSubscribeLiftDestinationFloor = true;
}

void SubscribePushItemToLift(const std_msgs::String& flag)
{
    if (strcmp(flag.data, COMMAND_NONE) == 0)
    {
        return;
    }
    
    gIsSubscribePushItemToLift = true;
}

void SubscribeSendToDestination(const std_msgs::String& dest)
{
    if (strcmp(dest.data, COMMAND_NONE) == 0)
    {
        return;
    }

    gDestination = dest.data;
    gIsSubscribeSendToDestination = true;
}

void DebugLed1Toggle()
{
    gLed1Toggle = !gLed1Toggle;
    digitalWrite(DEBUG_LED1_PIN, gLed1Toggle);
}

void DebugLed2Toggle()
{
    gLed2Toggle = !gLed2Toggle;
    digitalWrite(DEBUG_LED2_PIN, gLed2Toggle);
}

void DebugLed3Toggle()
{
    gLed3Toggle = !gLed3Toggle;
    digitalWrite(DEBUG_LED3_PIN, gLed3Toggle);
}

void DebugLed4Toggle()
{
    gLed4Toggle = !gLed4Toggle;
    digitalWrite(DEBUG_LED4_PIN, gLed4Toggle);
}

void CheckLiftArrivedAtTargetFloor()
{
    /* lift will go to target floor => checking validation */
    if (gLift.GetLiftStatus() == eLiftStatus::UP || gLift.GetLiftStatus() == eLiftStatus::DOWN)
    {
        if (gLift.GetCurrentFloor() == gTargetFloor) 
        {
            gLift.StopElevateMotor();
        }
    }
}

void CheckItemIsPushedToLift()
{
    Conveyor* currentConveyor = &gConveyorList[static_cast<uint8_t>(gLift.GetCurrentFloor()) - 1];

    if (currentConveyor->GetStatus() == eConveyorStatus::RIGHT || currentConveyor->GetStatus() == eConveyorStatus::LEFT)
    {
        if (gLift.GetIrStatus() == true) 
        {
            currentConveyor->SetItemPassed(true);
            delay(50);                                  // IrSensor Chatterring
        }

        /* item completely arrived at lift */
        if (gLift.GetIrStatus() == false && currentConveyor->IsItemPassed() == true)
        {
            gLift.GetConveyor().Stop();
            currentConveyor->Stop();
            
            currentConveyor->SetItemPassed(false);
            //currentConveyor = nullptr;

            gLift.SetLiftItemStatus(true);
        }
    }
}

void CheckItemIsPushedToDestination()
{
    /* send to "james" or "tray" => checking validation */
    if (gLift.GetConveyor().GetStatus() == eConveyorStatus::LEFT || gLift.GetConveyor().GetStatus() == eConveyorStatus::RIGHT)
    {
        if (gDestination == COMMAND_SEND_TO_JAMES)
        {
            // TODO: PC에서 카메라 영상 인식을 통해 물건이 전달됐음을 토픽으로 받았을 경우 리프트 모터 멈추기
            // "wstation/camera_item_state" => "none" => lift.Reset()
            gLift.Reset();
        }
        else // recved "tray" message
        {
            if (gLift.GetIrStatus() == true)
            {
                gConveyorList[static_cast<uint8_t>(eFloor::FirstFloor)].SetItemPassed(true);
                delay(50);              // IrSensor Chatterring
            }

            // item completely arrived at tray conveyor 
            if (gLift.GetIrStatus() == false && gConveyorList[0].IsItemPassed() == true)
            {
                gConveyorList[static_cast<uint8_t>(eFloor::FirstFloor)].SetItemPassed(false);
                gLift.Reset();

                /* 1층 컨베이어를 어느정도 움직여 줘야 함! */
                // 1층에 물건이 꽉 찾을 경우?
                gConveyorList[0].Stop();
            }
        }
    }
}