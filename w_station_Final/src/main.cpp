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

void InitializeLiftFloor();
void InitializeItemStatuses();
void IntializeLevelSwitchStatuses();

void PublishLiftCurrentFloor();
void PublishLiftStatus();
void PublishLiftItemStatus();
void PublishLiftIrStatus();
void PublishLiftConveyorStatus();
void PublishConveyorStatuses();
void PublishItemStatuses();
void PublishLevelSwitchStatuses();

void SubscribeLiftDestinationFloor(const std_msgs::Int8& floorOrNone);
void SubscribePushItem(const std_msgs::Bool& flag);
void SubscribeSendToDestination(const std_msgs::String& dest);
void SubscribeManual(const std_msgs::Bool& bManual);
void SubscribeEmergency(const std_msgs::Bool& bEmergency);

void CheckLiftArrivedAtTargetFloor();
void CheckItemIsPushedItem();
void CheckItemIsSendToDestination();

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

std_msgs::Int8MultiArray itemStatuses;
ros::Publisher publishItemStatuses("wstation/item_statuses", &itemStatuses);

std_msgs::Int8MultiArray levelSwitchStatuses;
ros::Publisher publishLevelSwitchStatuses("wstation/limited_switch_status", &levelSwitchStatuses);

/* ros - subscriber */
ros::Subscriber<std_msgs::Int8> subscribeLiftDestinationFloor("wstation/lift_destination_floor", SubscribeLiftDestinationFloor); // "1", "2", "3"
ros::Subscriber<std_msgs::Bool> subscribePushItemToLift("wstation/push_item", SubscribePushItem); 
ros::Subscriber<std_msgs::String> subscribeSendToDestination("wstation/send_to_destination", SubscribeSendToDestination); // "james", "tray", "none"

ros::Subscriber<std_msgs::Bool> subscribeManual("wstation/manual", SubscribeManual);
ros::Subscriber<std_msgs::Bool> subscribeEmergency("wstation/emergency", SubscribeEmergency);

/* global publisher status instance */
volatile bool gIsSubscribeLiftDestinationFloor = false;
volatile bool gbPushItem = false;
volatile bool gIsSubscribeSendToDestination = false;
volatile bool gbManual = false;
volatile bool gbEmergency = false;

/* global hardware instance */
Lift gLift;

Conveyor gConveyorList[MAX_FLOOR_COUNT] = {
    Conveyor(-1, FLOOR1_MOTOR_RELAY_SWITCH1_PIN, eFloor::FirstFloor), 
    Conveyor(FLOOR2_MOTOR_RELAY_SWITCH1_PIN, -1, eFloor::SecondFloor), 
    Conveyor(FLOOR3_MOTOR_RELAY_SWITCH1_PIN, -1, eFloor::ThirdFloor)
};

eFloor gTargetFloor;
String gDestination;

/* debug led toggle flag */
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
    
    /* System kill pin initialization */
    pinMode(SYSTEM_KILL_PIN, OUTPUT);
    digitalWrite(SYSTEM_KILL_PIN, SYSTEM_KILL_OFF);

    InitializeItemStatuses();
    IntializeLevelSwitchStatuses();

    // external analog input reference voltage setting to 5V [A0, A1, A2, A3] */
    analogReference(EXTERNAL);

    nodeHandle.getHardware()->setBaud(115200);
    nodeHandle.initNode();

    nodeHandle.advertise(publishLiftCurrentFloor);
    nodeHandle.advertise(publishLiftStatus);
    nodeHandle.advertise(publishLiftItemStatus);
    nodeHandle.advertise(publishLiftIrStatus);
    nodeHandle.advertise(publishLiftConveyorStatus);    
    nodeHandle.advertise(publishConveyorStatuses);
    nodeHandle.advertise(publishLevelSwitchStatuses);
    nodeHandle.advertise(publishItemStatuses);

    nodeHandle.subscribe(subscribeLiftDestinationFloor);
    nodeHandle.subscribe(subscribePushItemToLift);
    nodeHandle.subscribe(subscribeSendToDestination);
    nodeHandle.subscribe(subscribeManual);
    nodeHandle.subscribe(subscribeEmergency);

    Timer1.initialize(PUBLISH_PERIOD_US);                
    Timer1.attachInterrupt(PublishISR);

    InitializeLiftFloor();
}

void loop() 
{
    gLift.UpdateCurrentFloor();
    
    /* Publish current states */
    if (isPublishValidate == true) 
    {
        isPublishValidate = false;

        PublishLiftCurrentFloor();
        PublishLiftStatus();
        PublishLiftItemStatus();
        PublishLiftIrStatus();
        PublishLiftConveyorStatus();
        PublishConveyorStatuses();
        PublishItemStatuses();
        PublishLevelSwitchStatuses();

        DebugLed1Toggle();
    }

    nodeHandle.spinOnce();

    /* manual & emergency mode */
    if ((gbEmergency == false) || (gbManual == true))
    {
        digitalWrite(SYSTEM_KILL_PIN, SYSTEM_KILL_OFF); 

        if (gLift.GetLiftStatus() != eLiftStatus::ARRIVED) 
        {
            gLift.MoveToFloor(gTargetFloor);
        }  
    }
    else if ((gbEmergency == true) && (gbManual == false))
    {
        digitalWrite(SYSTEM_KILL_PIN, SYSTEM_KILL_ON);

        gLift.EmergencyStop();

        for (uint8_t index = 0; index < MAX_FLOOR_COUNT; ++index) 
        {
            gConveyorList[index].EmergencyStop();
        }
    }
    else 
    {
        digitalWrite(SYSTEM_KILL_PIN, SYSTEM_KILL_OFF);

        if (gLift.GetLiftStatus() != eLiftStatus::ARRIVED) 
        {
            gLift.MoveToFloor(gTargetFloor);
        }  
    }

    /* sub process part */

    if (gIsSubscribeLiftDestinationFloor == true) 
    {
        gIsSubscribeLiftDestinationFloor = false;
        
        if ((gLift.GetLiftStatus() == eLiftStatus::ARRIVED)) 
        {
            gLift.MoveToFloor(gTargetFloor);
        }

        DebugLed2Toggle();
    }

    CheckLiftArrivedAtTargetFloor();

    if (gbPushItem == true)
    {
        gbPushItem = false;

        if (gLift.GetLiftItemStatus() == false)
        {
            gConveyorList[static_cast<uint8_t>(gLift.GetCurrentFloor())].MoveLeft();
            gLift.GetConveyor().MoveLeft();
        }

        DebugLed3Toggle();
    }

    CheckItemIsPushedItem();

    if (gIsSubscribeSendToDestination == true)
    {
        gIsSubscribeSendToDestination = false;
        DebugLed4Toggle();

        if (gLift.GetLiftStatus() == eLiftStatus::ARRIVED && gLift.GetLiftItemStatus() == true && gLift.GetCurrentFloor() == eFloor::FirstFloor)
        {
            if (gDestination == COMMAND_SEND_TO_JAMES)
            {
                gLift.GetConveyor().MoveLeft();
            }
            else if (gDestination == COMMAND_SEND_TO_TRAY)
            {
                // TODO: tray가 꽉 차있을 경우 처리해야 함
                
                gLift.GetConveyor().MoveRight();
                gConveyorList[0].MoveRight();
            }
        }
    }

    CheckItemIsSendToDestination();
}

void InitializeLiftFloor()
{
    /* Go to first floor at starting system */
    while (gLift.GetLevelSwitchStatus(0) != true)
    {
        gLift.MoveDown();
    }

    gLift.StopElevateMotor();
}

void InitializeItemStatuses()
{
    itemStatuses.data = (int8_t*)malloc(sizeof(int8_t) * MAX_IR_SENSOR_COUNT);
    itemStatuses.data_length = MAX_IR_SENSOR_COUNT;
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

void PublishItemStatuses()
{
    for (uint8_t index = 0; index < MAX_CONVEYOR_IR_SENSOR_COUNT; ++index) 
    {
        itemStatuses.data[0 + index] = gConveyorList[0].GetIrSensorStatus(index);
        itemStatuses.data[4 + index] = gConveyorList[1].GetIrSensorStatus(index);
        itemStatuses.data[8 + index] = gConveyorList[2].GetIrSensorStatus(index);
    }

    publishItemStatuses.publish(&itemStatuses);
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
    if (floorOrNone.data <= 0 || floorOrNone.data > 3)
    {
        return;
    }

    if (gLift.GetLiftStatus() == eLiftStatus::UP || gLift.GetLiftStatus() == eLiftStatus::DOWN)
    {
        return;
    }

    gTargetFloor = static_cast<eFloor>(floorOrNone.data - 1);
    gIsSubscribeLiftDestinationFloor = true;
}

void SubscribePushItem(const std_msgs::Bool& flag)
{
    gbPushItem = flag.data;
}

void SubscribeSendToDestination(const std_msgs::String& dest)
{
    gDestination = dest.data;
    gIsSubscribeSendToDestination = true;
}

void SubscribeManual(const std_msgs::Bool& bManual)
{
    gbManual = bManual.data;
}

void SubscribeEmergency(const std_msgs::Bool& bEmergency)
{
    gbEmergency = bEmergency.data;
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
    if (gLift.GetLiftStatus() != eLiftStatus::ARRIVED)
    {
        if (gLift.GetCurrentFloor() == gTargetFloor) 
        {
            gLift.StopElevateMotor();
        }
    }
}

void CheckItemIsPushedItem()
{
    Conveyor* currentConveyor = &gConveyorList[static_cast<uint8_t>(gLift.GetCurrentFloor())];

    if (gLift.GetLiftItemStatus() == false)
    {
        if (gLift.GetIrStatus() == true) 
        {
            currentConveyor->SetItemPassed(true);
        }

        /* item completely arrived at lift */
        if (gLift.GetIrStatus() == false && currentConveyor->IsItemPassed() == true)
        {
            currentConveyor->Stop();
            currentConveyor->SetItemPassed(false);
            digitalWrite(LIFT_MAIN_LED_PIN, HIGH);

            delay(600);
            gLift.GetConveyor().Stop();
            gLift.SetLiftItemStatus(true);
        }
    }
}

void CheckItemIsSendToDestination()
{
    /* send to "james" or "tray" => checking validation */
    if (gLift.GetLiftItemStatus() == true)
    {
        if (gDestination == COMMAND_SEND_TO_JAMES)
        {
            // TODO: PC에서 카메라 영상 인식을 통해 물건이 전달됐음을 토픽으로 받았을 경우 리프트 모터 멈추기
            // "wstation/camera_item_state" => "none" => lift.Reset()
            // 일단 지금 제임스에게 잘 전달됐다는 토픽이 정의되지 않아 5초 동안 lift conveyor 돌린 후 stop() 되도록 시뮬
   
            delay(3000);        
            gLift.Reset();

            digitalWrite(LIFT_MAIN_LED_PIN, LOW);
        }
        else if (gDestination == COMMAND_SEND_TO_TRAY)// recved "tray" message
        {
            if (gLift.GetIrStatus() == true)
            {
                gConveyorList[static_cast<uint8_t>(eFloor::FirstFloor)].SetItemPassed(true);
            }

            // item completely arrived at tray conveyor 
            if (gLift.GetIrStatus() == false && gConveyorList[0].IsItemPassed() == true)
            {
                gLift.Reset();

                // 1층에 물건이 꽉 찾을 경우?

                delay(2000);
                gConveyorList[0].Stop();
                gConveyorList[static_cast<uint8_t>(eFloor::FirstFloor)].SetItemPassed(false);
                
                digitalWrite(LIFT_MAIN_LED_PIN, LOW);
            }
        }
    }
}