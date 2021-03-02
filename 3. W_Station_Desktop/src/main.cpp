#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>

#include "Command.h"
#include "Conveyor.h"
#include "Configuration.h"
#include "Lift.h"
#include "TimerInterrupt.h"
#include "Hardware.h"

eFloor gTargetFloor = eFloor::None;
String gDestination;
volatile bool gIsSubscribeLiftDestinationFloor = false;
volatile bool gIsSubscribePushItemToLift = false;
volatile bool gIsSubscribeSendToDestination = false;

void InitializeStationStatus();
void PublishStationStatus();
void PublishLiftCurrentFloor();
void PublishLiftStatus();
void PublishLiftItemStatus();
void CheckLiftArrivedAtTargetFloor();
void CheckItemIsPushedToLift();
void CheckItemIsPushedToDestination();
void SubscribeLiftDestinationFloor(const std_msgs::Int8& floorOrNone);
void SubscribePushItemToLift(const std_msgs::String& flag);
void SubscribeSendToDestination(const std_msgs::String& dest);

/* ros - NodeHandle */
ros::NodeHandle nodeHandle;

/* ros - publisher */
std_msgs::Int8MultiArray stationStatus;
ros::Publisher publishItemStatus("wstation/item_status", &stationStatus);

std_msgs::Int8 liftCurrentFloor;
ros::Publisher publishLiftCurrentFloor("wstation/lift_current_floor", &liftCurrentFloor);

std_msgs::String liftStatus;
ros::Publisher publishLiftStatus("wstation/lift_status", &liftStatus);

std_msgs::String liftItemStatus;
ros::Publisher publishLiftItemState("wstation/lift_item_status", &liftItemStatus);

/* ros - subscriber */
ros::Subscriber<std_msgs::Int8> subscribeLiftDestinationFloor("wstation/lift_destination_floor", SubscribeLiftDestinationFloor);
ros::Subscriber<std_msgs::String> subscribePushItemToLift("wstation/push_item_to_lift", SubscribePushItemToLift);
ros::Subscriber<std_msgs::String> subscribeSendToDestination("wstation/send_to_destination", SubscribeSendToDestination);

void setup() 
{
    pinMode(DEBUG_LED1_PIN, OUTPUT);
    pinMode(DEBUG_LED2_PIN, OUTPUT);
    pinMode(DEBUG_LED3_PIN, OUTPUT);
    pinMode(DEBUG_LED4_PIN, OUTPUT);

    nodeHandle.getHardware()->setBaud(115200);
    nodeHandle.initNode();

    InitializeStationStatus();

    nodeHandle.advertise(publishItemStatus);
    nodeHandle.advertise(publishLiftCurrentFloor);
    nodeHandle.advertise(publishLiftStatus);
    nodeHandle.advertise(publishLiftItemState);

    nodeHandle.subscribe(subscribeLiftDestinationFloor);
    nodeHandle.subscribe(subscribePushItemToLift);
    nodeHandle.subscribe(subscribeSendToDestination);

    Timer1.initialize(PUBLISH_PERIOD_US);                
    Timer1.attachInterrupt(PublishISR);

    Timer3.initialize(SUBSCRIBE_PERIOD_US);              
    Timer3.attachInterrupt(SubscribeISR);
}

/* global hardware instance */
Lift lift;

Conveyor conveyorList[MAX_FLOOR_COUNT] = {
    Conveyor(eFloor::FirstFloor), 
    Conveyor(eFloor::SecondFloor), 
    Conveyor(eFloor::ThirdFloor)
};
Conveyor* currentConveyor = nullptr;

bool led1Toggle = false;
bool led2Toggle = false;
bool led3Toggle = false;
bool led4Toggle = false;

void loop() 
{
    lift.UpdateCurrentFloor();

    /* publish */
    if (isPublishValidate == true) 
    {
        isPublishValidate = false;

        PublishStationStatus();
        PublishLiftCurrentFloor();
        PublishLiftStatus();
        PublishLiftItemStatus();

        led1Toggle = !led1Toggle;
        digitalWrite(DEBUG_LED1_PIN, led1Toggle);
    }

    /* subscribe */
    if (isSubscribeValidate == true) 
    {
        isSubscribeValidate = false;
        nodeHandle.spinOnce();

        /* subscribe -> lift_destination_floor */
        if (gIsSubscribeLiftDestinationFloor == true) 
        {
            gIsSubscribeLiftDestinationFloor = false;
            
            String status = lift.GetLiftStatus();
            lift.MoveToFloor(gTargetFloor);
        }

        /* subscribe -> push_item_to_lift */
        if (gIsSubscribePushItemToLift == true)
        {
            gIsSubscribePushItemToLift = false;

            currentConveyor = &(conveyorList[static_cast<uint8_t>(lift.GetCurrentFloor()) - 1]);
            currentConveyor->MoveLeft();
        }

        if (gIsSubscribeSendToDestination == true)
        {
            gIsSubscribeSendToDestination = false;

            if (lift.GetLiftStatus() == Lift::LIFT_STATUS_ARRIVED && lift.GetLiftItemStatus() == Lift::LIFT_ITEM_STATUS_EXIST)
            {
                if (gDestination == COMMAND_SEND_TO_JAMES)
                {
                    if (lift.IsJamesParked() == true)
                    {
                        lift.MoveLiftMotorToJames();
                    }
                }
                else // recved "tray"
                {
                    lift.MoveLiftMotorToTray();
                }
            }
        }
    }

    CheckLiftArrivedAtTargetFloor();
    CheckItemIsPushedToLift();
    CheckItemIsPushedToDestination();
}

void InitializeStationStatus() 
{
    stationStatus.layout.dim = (std_msgs::MultiArrayDimension*)malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    stationStatus.layout.dim[0].label = "floors";
    stationStatus.layout.dim[0].size = 3;
    stationStatus.layout.dim[0].stride = 12;

    stationStatus.layout.dim[1].label = "states";
    stationStatus.layout.dim[1].size = 4;
    stationStatus.layout.dim[1].stride = 4;
    stationStatus.layout.data_offset = 0;    

    stationStatus.data = (int8_t*)malloc(sizeof(int8_t) * 12);      /* o */
    stationStatus.data_length = 12;                                 /* o */
}

void PublishStationStatus()
{
    for (uint8_t i = 0; i < MAX_IR_SENSOR_COUNT; ++i)
    {
        stationStatus.data[0 + i] = conveyorList[0].GetIrSensorState(i);
        stationStatus.data[4 + i] = conveyorList[1].GetIrSensorState(i);
        stationStatus.data[8 + i] = conveyorList[2].GetIrSensorState(i);
    }

    publishItemStatus.publish(&stationStatus);
}

void PublishLiftCurrentFloor()
{
    liftCurrentFloor.data = static_cast<int8_t>(lift.GetCurrentFloor());
    publishLiftCurrentFloor.publish(&liftCurrentFloor);
}

void PublishLiftStatus()
{
    char buf[10];
    uint8_t liftStatusLength = lift.GetLiftStatus().length();

    for (uint8_t i = 0; i < liftStatusLength; ++i)
    {
        buf[i] = lift.GetLiftStatus()[i];
    }
    buf[liftStatusLength] = 0;

    liftStatus.data = buf;
    publishLiftStatus.publish(&liftStatus);
}

void PublishLiftItemStatus()
{
    char buf[10];
    uint8_t liftItemStatusLength = lift.GetLiftItemStatus().length();

    for (uint8_t i = 0; i < liftItemStatusLength; ++i)
    {
        buf[i] = lift.GetLiftItemStatus()[i];
    }
    buf[liftItemStatusLength] = 0;
    
    liftItemStatus.data = buf;
    publishLiftItemState.publish(&liftItemStatus);
}

void CheckLiftArrivedAtTargetFloor()
{
    /* lift will go to target floor => checking validation */
    if (lift.GetLiftStatus() == Lift::LIFT_STATUS_MOVE)
    {
        lift.UpdateCurrentFloor();
        if (lift.GetCurrentFloor() == gTargetFloor) 
        {
            lift.StopElevateMotor();
        }
    }
}

void CheckItemIsPushedToLift()
{
    if (currentConveyor != nullptr) 
    {
        if (currentConveyor->GetState() == CONVEYOR_STATUS_MOVE)
        {
            if (lift.IsItemPassed() == true) 
            {
                currentConveyor->SetIsItemPassed(true);
                delay(50);                                  // IrSensor Chatterring
            }

            /* item completely arrived at lift */
            if (lift.IsItemPassed() == false && currentConveyor->GetIsItemPassed() == true)
            {
                currentConveyor->Stop();
                currentConveyor->SetIsItemPassed(false);
                currentConveyor = nullptr;

                lift.SetLiftItemStatus(Lift::LIFT_ITEM_STATUS_NONE);
            }
        }
    }
}

void CheckItemIsPushedToDestination()
{
    /* send to "james" or "tray" => checking validation */
    if (lift.GetLiftMotorStatus() == Lift::LIFT_MOTOR_STATUS_MOVE)
    {
        if (gDestination == COMMAND_SEND_TO_JAMES)
        {
            // james 에게 잘 들어 갔는 지에 대한 판단 필요 
            // TODO: 
            lift.Initialize();
        }
        else // recved "tray" 
        {
            if (lift.IsItemPassed() == true)
            {
                conveyorList[static_cast<uint8_t>(eFloor::FirstFloor)].SetIsItemPassed(true);
                delay(50);              // IrSensor Chatterring
            }

            // item completely arrived at tray conveyor 
            if (lift.IsItemPassed() == false && conveyorList[0].GetIsItemPassed() == true)
            {
                conveyorList[static_cast<uint8_t>(eFloor::FirstFloor)].SetIsItemPassed(false);
                lift.Initialize();
            }
        }
    }
}

/* Subscriber Callback Function */
void SubscribeLiftDestinationFloor(const std_msgs::Int8& floorOrNone)
{
    if (floorOrNone.data <= 0)
    {
        return;
    }

    gTargetFloor = static_cast<eFloor>(floorOrNone.data);
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
    gDestination = dest.data;
    gIsSubscribeSendToDestination = true;
}