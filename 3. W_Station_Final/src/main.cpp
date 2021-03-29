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
void SubscribePushItemToLift(const std_msgs::String& flag);
void SubscribeSendToDestination(const std_msgs::String& dest);
void SubscribeManual(const std_msgs::Bool& bManual);
void SubscribeEmergency(const std_msgs::Bool& bEmergency);

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

std_msgs::Int8MultiArray itemStatuses;
ros::Publisher publishItemStatuses("wstation/item_statuses", &itemStatuses);

std_msgs::Int8MultiArray levelSwitchStatuses;
ros::Publisher publishLevelSwitchStatuses("wstation/limited_switch_status", &levelSwitchStatuses);

/* ros - subscriber */
ros::Subscriber<std_msgs::Int8> subscribeLiftDestinationFloor("wstation/lift_destination_floor", SubscribeLiftDestinationFloor); // "1", "2", "3"
ros::Subscriber<std_msgs::String> subscribePushItemToLift("wstation/push_item_to_lift", SubscribePushItemToLift); // "push", "none"
ros::Subscriber<std_msgs::String> subscribeSendToDestination("wstation/send_to_destination", SubscribeSendToDestination); // "james", "tray", "none"

ros::Subscriber<std_msgs::Bool> subscribeManual("wstation/manual", SubscribeManual);
ros::Subscriber<std_msgs::Bool> subscribeEmergency("wstation/emergency", SubscribeEmergency);


/* global publisher status instance */
volatile bool gIsSubscribeLiftDestinationFloor = false;
volatile bool gIsSubscribePushItemToLift = false;
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
    
    InitializeItemStatuses();
    IntializeLevelSwitchStatuses();

    // 아날로그 입력 기준전압을 VREF 핀에 연결되어 있는 5V로 설정 [A0, A1, A2, A3] for LevelSwitch
    analogReference(EXTERNAL);

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
    nodeHandle.advertise(publishItemStatuses);

    nodeHandle.subscribe(subscribeLiftDestinationFloor);
    nodeHandle.subscribe(subscribePushItemToLift);
    nodeHandle.subscribe(subscribeSendToDestination);

    // add
    nodeHandle.subscribe(subscribeManual);
    nodeHandle.subscribe(subscribeEmergency);

    Timer1.initialize(PUBLISH_PERIOD_US);                
    Timer1.attachInterrupt(PublishISR);

    /* lift의 생성자에서 while문을 실행하면 도중에 blocking 됨 - 리프트는 무조건 1층에서 시작 */
    // if (gLift.GetCurrentFloor() != eFloor::FirstFloor) 
    // {
    //     while (gLift.GetCurrentFloor() != eFloor::FirstFloor)
    //     {
    //         gLift.MoveToFloor(eFloor::FirstFloor);
    //         gLift.UpdateCurrentFloor();
    //     }
    //     gLift.StopElevateMotor();
    // }
}

LevelSwitch lvSwitch0(FLOOR1_LIMITED_SWITCH_READ_PIN);
LevelSwitch lvSwitch1(FLOOR2_LIMITED_SWITCH_READ_PIN);
LevelSwitch lvSwitch2(FLOOR3_LIMITED_SWITCH_READ_PIN);
LevelSwitch lvSwitch3(FLOOR3_CEILING_LIMITED_SWITCH_READ_PIN);

void loop() 
{
    /* test */
    /*
    while (true) 
    {
        gLift.GetConveyor().MoveLeft();
        delay(1000);
        gLift.GetConveyor().MoveRight();
        delay(1000);
        gLift.GetConveyor().Stop();
        delay(1000);
    
    */    

   //analogWrite(ELEVATE_MOTOR_BREAK_PIN, ANALOG_HIGH);
   //digitalWrite(ELEVATE_MOTOR_CW_PIN, HIGH);
   
   while (true)
   {   
       
        gLift.MoveUp();
        delay(1000);

        //gLift.MoveDown();
        //delay(2000);

        gLift.StopElevateMotor();
        while(true);
        
        /* 1. Level Switch, Lift IR, Conveyor IR Test */
        /*
        if (lvSwitch0.GetState() == true) 
        {
            digitalWrite(DEBUG_LED1_PIN, HIGH);
        }
        else 
        {
            digitalWrite(DEBUG_LED1_PIN, LOW);
        }

        if (lvSwitch1.GetState() == true)
        {
            digitalWrite(DEBUG_LED2_PIN, HIGH);
        }
        else 
        {
            digitalWrite(DEBUG_LED2_PIN, LOW);
        }

        if (lvSwitch2.GetState() == true)
        {
            digitalWrite(DEBUG_LED3_PIN, HIGH);
        }
        else 
        {
            digitalWrite(DEBUG_LED3_PIN, LOW);
        }

        if (lvSwitch3.GetState() == true)
        {
            digitalWrite(DEBUG_LED4_PIN, HIGH);
        }
        else 
        {
            digitalWrite(DEBUG_LED4_PIN, LOW);
        }
        */
        

        /* main LED */
        //digitalWrite(LIFT_IR_LED_PIN, HIGH);
        // delay(5000);
        // digitalWrite(LIFT_IR_LED_PIN, LOW);
        // delay(5000);

        /*
        if (gLift.GetIrStatus() == true)
        {
            digitalWrite(DEBUG_LED1_PIN, HIGH);
        }
        else
        {
            digitalWrite(DEBUG_LED1_PIN, LOW);
        }
        */
        

        /*
        for (int index = 0; index < MAX_FLOOR_COUNT; ++index)
        {
            for (int irIndex = 0; irIndex < 5; ++irIndex)
            {
                if (gConveyorList[index].GetIrSensorStatus(irIndex) == true)
                {
                    digitalWrite(DEBUG_LED2_PIN, HIGH);
                }
                else
                {
                    digitalWrite(DEBUG_LED2_PIN, LOW);
                }
            }
        }
        */
       
        /* 2. Conveyor Test */
        // gConveyorList[0].MoveRight();
        // delay(5000);
        // gConveyorList[0].Stop();
        // delay(5000);

        // INFO : second floor not working 
        // gConveyorList[1].MoveLeft();
        // delay(5000);
        // gConveyorList[1].Stop();
        // delay(5000);

        // gConveyorList[2].MoveLeft();
        // delay(5000);
        // gConveyorList[2].Stop();
        // delay(5000);
        

        /* 3. Lift Test */
        
        // Up Down
        /*
        gLift.MoveToFloor(eFloor::FirstFloor);
        while (gLift.GetCurrentFloor() != eFloor::FirstFloor)
        {
            gLift.UpdateCurrentFloor();
        }
        gLift.StopElevateMotor();
        
        gLift.MoveToFloor(eFloor::SecondFloor);
        while (gLift.GetCurrentFloor() != eFloor::SecondFloor)
        {
            gLift.UpdateCurrentFloor();
        }
        gLift.StopElevateMotor();

        gLift.MoveToFloor(eFloor::ThirdFloor);
        while (gLift.GetCurrentFloor() != eFloor::ThirdFloor)
        {
            gLift.UpdateCurrentFloor();
        }
        gLift.StopElevateMotor();

        gLift.MoveToFloor(eFloor::FirstFloor);
        while (gLift.GetCurrentFloor() != eFloor::FirstFloor)
        {
            gLift.UpdateCurrentFloor();
        }
        gLift.StopElevateMotor();
        */

        // Lift Conveyor Left Right
        // gLift.GetConveyor().MoveLeft();
        // delay(5000);
        // gLift.GetConveyor().MoveRight();
        // delay(5000);
        // gLift.GetConveyor().Stop();
        // delay(5000);
   }

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
        PublishItemStatuses();
        PublishLevelSwitchStatuses();

        DebugLed1Toggle();
    }

    /* Subscribe */
    nodeHandle.spinOnce();

    //TODO: 토픽 받으면 subscribe 콜백 함수에서 움직이는 하드웨어들 전부 멈추게끔 만든다. 그후 gbManual을 true로!
    /*
        3 -> 1 으로 lift가 움직이고 있는 도중 2층에서 manual mode shooking 이 들어왔을 경우?
    */
    /*
    if (gbManual == true)
    {
        
    }

    if (gbManual == false)
    {
    }
    */

    if (gbEmergency == true) 
    {
        gLift.EmergencyStop();
        for (uint8_t index = 0; index < MAX_FLOOR_COUNT; ++index) 
        {
            gConveyorList[index].EmergencyStop();
        }

        gbEmergency = false;
    }

    /* subscribe -> [wstation/lift_destination_floor] */
    if (gIsSubscribeLiftDestinationFloor == true) 
    {
        gIsSubscribeLiftDestinationFloor = false;
        
        if (gLift.GetLiftStatus() == eLiftStatus::ARRIVED) 
        {
            gLift.MoveToFloor(gTargetFloor);
        }

        DebugLed2Toggle();
    }

    CheckLiftArrivedAtTargetFloor(); // If arrived, stop the motor

    /* subscribe -> [wstation/push_item_to_lift] */
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

    /* subscribe -> [wstation/send_to_destination] */
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
                // TODO: tray가 꽉 차있을 경우 처리해야 함
                if (false)  
                {
                    // alarm();
                }
                else 
                {
                    gLift.GetConveyor().MoveRight();
                    gConveyorList[0].MoveRight();
                }
                
            }
        }
    }

    CheckItemIsPushedToDestination();
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
            // IrSensor Chatterring
            delay(50);                     
        }

        /* item completely arrived at lift */
        if (gLift.GetIrStatus() == false && currentConveyor->IsItemPassed() == true)
        {
            gLift.GetConveyor().Stop();
            gLift.SetLiftItemStatus(true);

            currentConveyor->Stop();
            currentConveyor->SetItemPassed(false);
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
            // 일단 지금 제임스에게 잘 전달됐다는 토픽이 정의되지 않아 5초 동안 lift conveyor 돌린 후 stop() 되도록 시뮬
            delay(5000);        
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
                gLift.Reset();

                /* 1층 컨베이어를 어느정도 움직여 줘야 함! */
                // 1층에 물건이 꽉 찾을 경우?
                gConveyorList[0].Stop();
                gConveyorList[static_cast<uint8_t>(eFloor::FirstFloor)].SetItemPassed(false);
            }
        }
    }
}