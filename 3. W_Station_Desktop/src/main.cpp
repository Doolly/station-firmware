#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>

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

std_msgs::String liftStatus;
ros::Publisher publishLiftStatus("wstation/lift_status", &liftStatus);

std_msgs::Int8 liftCurrentFloor;
ros::Publisher publishLiftCurrentFloor("wstation/lift_current_floor", &liftCurrentFloor);

std_msgs::String liftItemStatus;
ros::Publisher publishLiftItemState("wstation/lift_item_status", &liftItemStatus);

/* ros - subscriber */
volatile bool isSubscribeLiftDestinationFloor = false;
eFloor targetFloor = eFloor::None;
int8_t liftDestinationFloor;
void SubscribeLiftDestinationFloor(const std_msgs::Int8& floor);
ros::Subscriber<std_msgs::Int8> subscribeLiftDestinationFloor("wstation/lift_destination_floor", SubscribeLiftDestinationFloor);

#define COMMAND_NONE            ("none")
#define COMMAND_PUSH            ("push")
volatile bool isSubscribePushItemToLift = false;
String pushItemToLiftFlag;
void SubscribePushItemToLift(const std_msgs::String& flag);
ros::Subscriber<std_msgs::String> subscribePushItemToLift("wstation/push_item_to_lift", SubscribePushItemToLift);

#define COMMAND_TRAY            ("tray")
#define COMMAND_JAMES           ("james")
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
    nodeHandle.advertise(publishLiftItemState);

    nodeHandle.subscribe(subscribeLiftDestinationFloor);
    nodeHandle.subscribe(subscribePushItemToLift);
    nodeHandle.subscribe(subscribeSendToDestination);

    Timer1.initialize(PUBLISH_PERIOD_US);                
    Timer1.attachInterrupt(PublishISR);
    Timer3.initialize(SUBSCRIBE_PERIOD_US);              
    Timer3.attachInterrupt(SubscribeISR);
}

void loop() 
{
    /* publish */
    if (isPublishValidate == true) 
    {
        isPublishValidate = false;
        
        GetAllStationStatus(stationStatus);
        liftCurrentFloor.data = static_cast<int8_t>(lift.GetCurrentFloor());
        liftStatus.data = lift.GetLiftStatus().c_str();
        liftItemStatus.data = lift.GetLiftItemStatus().c_str();

        publishItemStatus.publish(&stationStatus);
        publishLiftCurrentFloor.publish(&liftCurrentFloor);
        publishLiftStatus.publish(&liftStatus);
        publishLiftItemState.publish(&liftItemStatus);
    }

    /* subscribe */
    if (isSubscribeValidate == true) 
    {
        isSubscribeValidate = false;
        nodeHandle.spinOnce();

        if (isSubscribeLiftDestinationFloor == true) 
        {
            isSubscribeSendToDestination = false;
            
            if (lift.GetLiftStatus() == LIFT_STATUS_WAITING) 
            {
                lift.MoveToFloor(targetFloor);
            }
        }

        if (isSubscribePushItemToLift == true)
        {
            isSubscribePushItemToLift = false;
            Conveyor* currentConveyor = &(conveyorList[static_cast<uint8_t>(lift.GetCurrentFloor()) - 1]);

            if (pushItemToLiftFlag == COMMAND_PUSH)
            {
                if (lift.GetLiftStatus() == LIFT_STATUS_ARRIVED)
                {
                    currentConveyor->MoveLeft();
                    currentConveyor->Stop();
                }
            }
            else 
            {
                currentConveyor->Stop();
            }
        }

        if (isSubscribeSendToDestination == true)
        {
            isSubscribeSendToDestination = false;

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

void SubscribeLiftDestinationFloor(const std_msgs::Int8& floor)
{
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

}