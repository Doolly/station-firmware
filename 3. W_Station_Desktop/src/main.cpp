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

/* ros - NodeHandle */
ros::NodeHandle nodeHandle;

/* ros - publisher */
std_msgs::Int8MultiArray stationStatus;
ros::Publisher publishItemStatus("wstation/item_status", &stationStatus);
void InitializeStationStatus();
void GetAllStationStatus();

std_msgs::Int8 liftCurrentFloor;
ros::Publisher publishLiftCurrentFloor("wstation/lift_current_floor", &liftCurrentFloor);

std_msgs::String liftStatus;
ros::Publisher publishLiftStatus("wstation/lift_status", &liftStatus);

std_msgs::String liftItemStatus;
ros::Publisher publishLiftItemState("wstation/lift_item_status", &liftItemStatus);

/* ros - subscriber */
volatile bool isSubscribeLiftDestinationFloor = false;
eFloor targetFloor = eFloor::None;
int8_t liftDestinationFloor;
void SubscribeLiftDestinationFloor(const std_msgs::Int8& floor);
ros::Subscriber<std_msgs::Int8> subscribeLiftDestinationFloor("wstation/lift_destination_floor", SubscribeLiftDestinationFloor);

volatile bool isSubscribePushItemToLift = false;
void SubscribePushItemToLift(const std_msgs::String& flag);
ros::Subscriber<std_msgs::String> subscribePushItemToLift("wstation/push_item_to_lift", SubscribePushItemToLift);

volatile bool isSubscribeSendToDestination = false;
String destination;
void SubscribeSendToDestination(const std_msgs::String& dest);
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
Conveyor* currentConveyor = nullptr;

bool led1Toggle = false;
bool led2Toggle = false;
bool led3Toggle = false;
bool led4Toggle = false;

char cArrLiftStatus[10] = {0,};

void loop() 
{
    lift.UpdateCurrentFloor();

    /* publish */
    if (isPublishValidate == true) 
    {
        isPublishValidate = false;
        
        GetAllStationStatus();
        //publishItemStatus.publish(&stationStatus);

        liftCurrentFloor.data = static_cast<int8_t>(lift.GetCurrentFloor());
        publishLiftCurrentFloor.publish(&liftCurrentFloor);

        uint8_t liftStatusLength = lift.GetLiftStatus().length();
        for (uint8_t i = 0; i < liftStatusLength; ++i)
        {
            cArrLiftStatus[i] = lift.GetLiftStatus()[i];
        }
        cArrLiftStatus[liftStatusLength] = 0;
        liftStatus.data = cArrLiftStatus;
        publishLiftStatus.publish(&liftStatus);

        uint8_t liftItemStatusLength = lift.GetLiftItemStatus().length();
        for (uint8_t i = 0; i < liftItemStatusLength; ++i)
        {
            cArrLiftStatus[i] = lift.GetLiftItemStatus()[i];
        }
        cArrLiftStatus[liftItemStatusLength] = 0;
        liftItemStatus.data = cArrLiftStatus;
        publishLiftItemState.publish(&liftItemStatus);

        led1Toggle = !led1Toggle;
        digitalWrite(DEBUG_LED1_PIN, led1Toggle);
    }

    /* subscribe */
    if (isSubscribeValidate == true) 
    {
        isSubscribeValidate = false;
        nodeHandle.spinOnce();

        /* subscribe -> lift_destination_floor */
        if (isSubscribeLiftDestinationFloor == true) 
        {
            isSubscribeLiftDestinationFloor = false;
            
            String status = lift.GetLiftStatus();
            lift.MoveToFloor(targetFloor);

            digitalWrite(DEBUG_LED2_PIN, HIGH);
            digitalWrite(DEBUG_LED3_PIN, LOW);
            digitalWrite(DEBUG_LED4_PIN, LOW);
        }

        /* subscribe -> push_item_to_lift */
        if (isSubscribePushItemToLift == true)
        {
            isSubscribePushItemToLift = false;

            currentConveyor = &(conveyorList[static_cast<uint8_t>(lift.GetCurrentFloor()) - 1]);
            currentConveyor->MoveLeft();

            digitalWrite(DEBUG_LED2_PIN, LOW);
            digitalWrite(DEBUG_LED3_PIN, HIGH);
            digitalWrite(DEBUG_LED4_PIN, LOW);
        }

        if (isSubscribeSendToDestination == true)
        {
            isSubscribeSendToDestination = false;

            if (lift.GetLiftStatus() == LIFT_STATUS_ARRIVED && lift.GetLiftItemStatus() == LIFT_ITEM_STATUS_EXIST)
            {
                if (lift.GetLiftMotorStatus() == LIFT_MOTOR_STATUS_STOP) 
                {
                    if (destination == COMMAND_SEND_TO_JAMES)
                    {
                        if (lift.IsJamesParked() == true)
                        {
                            lift.MoveLiftMotorToJames();
                        }
                    }
                    else // recved "tray"
                    {
                        lift.MoveLiftMotorToTray();

                        digitalWrite(DEBUG_LED2_PIN, LOW);
                        digitalWrite(DEBUG_LED3_PIN, LOW);
                        digitalWrite(DEBUG_LED4_PIN, HIGH);
                    }
                }
            }
        }
    }

    /* lift will go to target floor => checking validation */
    if (lift.GetLiftStatus() == LIFT_STATUS_MOVE)
    {
        lift.UpdateCurrentFloor();
        if (lift.GetCurrentFloor() == targetFloor) 
        {
            lift.StopElevateMotor();
        }
    }

    if (currentConveyor != nullptr) 
    {
        if (currentConveyor->GetState() == CONVEYOR_STATUS_MOVE)
        {
            if (lift.IsItemPassed() == true) 
            {
                currentConveyor->SetIsItemPassed(true);
                delay(50);          // IrSensor Chatterring
            }

            /* item completely arrived at lift */
            if (lift.IsItemPassed() == false && currentConveyor->GetIsItemPassed() == true)
            {
                currentConveyor->Stop();
                currentConveyor->SetIsItemPassed(false);
                currentConveyor = nullptr;
                lift.SetLiftItemStatus(LIFT_ITEM_STATUS_EXIST);
            }
        }
    }

    /* send to "james" or "tray" => checking validation */
    if (lift.GetLiftMotorStatus() == LIFT_MOTOR_STATUS_MOVE)
    {
        if (destination == COMMAND_SEND_TO_JAMES)
        {
            // james 에게 잘 들어 갔는 지에 대한 판단 필요 
            // TODO: 
            lift.Initialize();

            digitalWrite(DEBUG_LED2_PIN, LOW);
            digitalWrite(DEBUG_LED3_PIN, LOW);
            digitalWrite(DEBUG_LED4_PIN, LOW);
        }
        else // recved "tray" 
        {
            if (lift.IsItemPassed() == true)
            {
                conveyorList[0].SetIsItemPassed(true);
                delay(50);      // IrSensor Chatterring
            }

            // item completely arrived at tray conveyor 
            if (lift.IsItemPassed() == false && conveyorList[0].GetIsItemPassed() == true)
            {
                conveyorList[0].SetIsItemPassed(false);
                lift.Initialize();

                digitalWrite(DEBUG_LED2_PIN, LOW);
                digitalWrite(DEBUG_LED3_PIN, LOW);
                digitalWrite(DEBUG_LED4_PIN, LOW);
            }
        }
    }

    if (lift.IsItemPassed() == true)
    {
        digitalWrite(DEBUG_LED4_PIN, HIGH);
    }
    else
    {
        digitalWrite(DEBUG_LED4_PIN, LOW);
    }
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

void GetAllStationStatus()
{
    for (uint8_t i = 0; i < MAX_IR_SENSOR_COUNT; ++i)
    {
        stationStatus.data[0 + i] = conveyorList[0].GetIrSensorState(i);
        stationStatus.data[4 + i] = conveyorList[1].GetIrSensorState(i);
        stationStatus.data[8 + i] = conveyorList[2].GetIrSensorState(i);
    }
}

/* Subscriber Callback Function */
void SubscribeLiftDestinationFloor(const std_msgs::Int8& floor)
{
    if (floor.data <= 0)
    {
        return;
    }

    targetFloor = static_cast<eFloor>(floor.data);
    isSubscribeLiftDestinationFloor = true;
}

void SubscribePushItemToLift(const std_msgs::String& flag)
{
    if (strcmp(flag.data, COMMAND_NONE) == 0)
    {
        return;
    }
    
    isSubscribePushItemToLift = true;
}

void SubscribeSendToDestination(const std_msgs::String& dest)
{
    destination = dest.data;
    isSubscribeSendToDestination = true;
}