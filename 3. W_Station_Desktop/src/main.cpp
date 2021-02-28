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
void InitializeStationStatus(std_msgs::Int8MultiArray& status);
void GetAllStationStatus(std_msgs::Int8MultiArray& status);

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
String pushItemToLiftFlag;
void SubscribePushItemToLift(const std_msgs::String& flag);
ros::Subscriber<std_msgs::String> subscribePushItemToLift("wstation/push_item_to_lift", SubscribePushItemToLift);

volatile bool isSubscribeSendToDestination = false;
String destination;
void SubscribeSendToDestination(const std_msgs::String& dest);
ros::Subscriber<std_msgs::String> subscribeSendToDestination("wstation/send_to_destination", SubscribeSendToDestination);

void setup() 
{
    /* Intialize Ros instances */
    pinMode(DEBUG_LED1_PIN, OUTPUT);
    pinMode(DEBUG_LED2_PIN, OUTPUT);
    pinMode(DEBUG_LED3_PIN, OUTPUT);
    pinMode(DEBUG_LED4_PIN, OUTPUT);

    nodeHandle.getHardware()->setBaud(115200);
    nodeHandle.initNode();
    
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
        
        GetAllStationStatus(stationStatus);
        //publishItemStatus.publish(&stationStatus);
        
        liftCurrentFloor.data = static_cast<int8_t>(lift.GetCurrentFloor());
        publishLiftCurrentFloor.publish(&liftCurrentFloor);

        for (uint8_t i = 0; i < lift.GetLiftStatus().length(); ++i)
        {
            cArrLiftStatus[i] = lift.GetLiftStatus()[i];
        }
        cArrLiftStatus[lift.GetLiftStatus().length()] = 0;
        liftStatus.data = cArrLiftStatus;
        publishLiftStatus.publish(&liftStatus);

        for (uint8_t i = 0; i < lift.GetLiftItemStatus().length(); ++i)
        {
            cArrLiftStatus[i] = lift.GetLiftItemStatus()[i];
        }
        cArrLiftStatus[lift.GetLiftItemStatus().length()] = 0;
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

        if (isSubscribeLiftDestinationFloor == true) 
        {
            isSubscribeSendToDestination = false;
            
            String status = lift.GetLiftStatus();
            
            if (status == LIFT_STATUS_WAIT || status == LIFT_STATUS_ARRIVED)
            {
                lift.MoveToFloor(targetFloor);
            }

            led2Toggle = !led2Toggle;
            digitalWrite(DEBUG_LED2_PIN, led2Toggle);
        }

        if (isSubscribePushItemToLift == true)
        {
            isSubscribePushItemToLift = false;

            currentConveyor = &(conveyorList[static_cast<uint8_t>(lift.GetCurrentFloor()) - 1]);

            if (pushItemToLiftFlag == COMMAND_PUSH)
            {
                /* lift 가 목적지에 도착해서 arrived 상태로 대기하고 있을 경우에만 conveyor moving 명령을 먹음 */
                if (lift.GetLiftStatus() == LIFT_STATUS_ARRIVED)
                {
                    currentConveyor->MoveLeft();
                }
            }

            led3Toggle = !led3Toggle;
            digitalWrite(DEBUG_LED3_PIN, led3Toggle);
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
                    else /* recved "tray" */
                    {
                        lift.MoveLiftMotorToTray();
                    }
                }
            }

            led4Toggle = !led4Toggle;
            digitalWrite(DEBUG_LED2_PIN, led4Toggle);
        }
    }

    if (lift.GetLiftStatus() == LIFT_STATUS_MOVE)
    {
        lift.UpdateCurrentFloor();
        if (lift.GetCurrentFloor() == targetFloor) 
        {
            lift.StopElevateMotor();
            lift.SetLiftStatus(LIFT_STATUS_ARRIVED);
        }
    }

    if (currentConveyor != nullptr) 
    {
        if (currentConveyor->GetState() == CONVEYOR_STATUS_MOVE)
        {
            if (lift.IsItemPassed() == true) 
            {
                currentConveyor->SetIsItemPassed(true);
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

    if (lift.GetLiftMotorStatus() == LIFT_MOTOR_STATUS_MOVE)
    {
        if (destination == COMMAND_SEND_TO_JAMES)
        {
            lift.Initialize();
        }
        else /* recved "tray" */
        {
            if (lift.IsItemPassed() == true)
            {
                conveyorList[0].SetIsItemPassed(true);
            }

            /* item completely arrived at tray conveyor */
            if (lift.IsItemPassed() == false && conveyorList[0].GetIsItemPassed() == true)
            {
                conveyorList[0].SetIsItemPassed(false);
                lift.Initialize();
            }
        }
    }
}

void InitializeStationStatus(std_msgs::Int8MultiArray& status) 
{
    status.layout.dim = (std_msgs::MultiArrayDimension*)malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    status.layout.dim_length = 2;

    status.layout.dim[0].label = "floors";
    status.layout.dim[0].size = MAX_FLOOR_COUNT;
    status.layout.dim[0].stride = 12;
    
    status.layout.dim[1].label = "states";
    status.layout.dim[1].size = MAX_IR_SENSOR_COUNT;
    status.layout.dim[1].stride = 3;

    status.data = (int8_t*)malloc(sizeof(int8_t) * 12);
    memset(status.data, 0, 12 * sizeof(int8_t));

    status.data_length = 12;
    status.layout.data_offset = 0;
}

void GetAllStationStatus(std_msgs::Int8MultiArray& status)
{
    for (uint8_t i = 0; i < MAX_IR_SENSOR_COUNT; ++i)
    {
        status.data[0 + i] = conveyorList[0].GetIrSensorState(i);
        status.data[4 + i] = conveyorList[1].GetIrSensorState(i);
        status.data[8 + i] = conveyorList[2].GetIrSensorState(i);
    }
}

/* Subscriber Callback Function */
void SubscribeLiftDestinationFloor(const std_msgs::Int8& floor)
{
    if (floor.data == -1) 
    {
        return;
    }

    targetFloor = static_cast<eFloor>(floor.data);
    isSubscribeSendToDestination = true;
}

void SubscribePushItemToLift(const std_msgs::String& flag)
{
    pushItemToLiftFlag = flag.data;
    isSubscribePushItemToLift = true;
}

void SubscribeSendToDestination(const std_msgs::String& dest)
{
    destination = dest.data;
    isSubscribeSendToDestination = true;
}