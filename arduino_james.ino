/*------ Headers to Include ------*/
#include "barometer.h"
#include "infra_red.h"
#include "ultra_sound.h"
#include "james_motor.h"
#include <ros.h>
#include <ros/time.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Range.h>
#include <james_msgs/Barometer.h>
#include <james_msgs/MotorControl.h>

//#define IR_READ_PIN 2
//#define IR_LIGHT_PIN 3
//#define US0_ANALOG_PIN A0
//#define US1_ANALOG_PIN A1
#define LINEAR_ACTUATOR_L1 4
#define LINEAR_ACTUATOR_L2 7
#define LINEAR_ACTUATOR_R1 8
#define LINEAR_ACTUATOR_R2 9
#define CONVEYOR_PIN1 10
#define CONVEYOR_PIN2 11

/*------ Global Variables ------*/
ros::NodeHandle nh;
/*----baromter----*/
james_msgs::Barometer baro_msg;
Barometer baro;
ros::Publisher pub_baro("/barometer", &baro_msg);

/*----Motors----*/
MotorController mc;
void motorControlCB(const james_msgs::MotorControl &msg)
{
  mc.control(msg.motor, msg.direction, msg.pwm);
}
ros::Subscriber<james_msgs::MotorControl> sub("motor_control", &motorControlCB);

/*----IR----*/
//InfraRed ir(IR_READ_PIN, IR_LIGHT_PIN);
//bool lightControl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response &res) {
//  ir.lightControl(req.data);
//  res.success = true;
//  if (req.data)
//    res.message = "Light on";
//  else
//    res.message = "Light off";
//  return res.success;
//}
//bool isOccupied(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response &res) {
//  res.success = ir.isOccupied();
//  if (res.success)
//    res.message = "There is a package";
//  else
//    res.message = "There is not a package";
//  return res.success;
//}
//ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> light_server("/ir_light", &lightControl);
//ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> ir_server("/ir", &isOccupied);

/*----Sonar----*/
//sensor_msgs::Range range_msg;
//UltraSound us0(US0_ANALOG_PIN), us1(US1_ANALOG_PIN);
//UltraSound us0(US0_ANALOG_PIN);
//ros::Publisher pub_range0( "/sonar0", &range_msg);
//ros::Publisher pub_range1( "/sonar1", &range_msg);

void setup()
{
  nh.initNode();
  nh.advertise(pub_baro);
  baro.setReference();

  nh.subscribe(sub);
  mc.addMotor(new PairedBaseMotor(LINEAR_ACTUATOR_L1, LINEAR_ACTUATOR_L2, LINEAR_ACTUATOR_R1, LINEAR_ACTUATOR_R2));
  mc.addMotor(new BaseMotor(CONVEYOR_PIN1, CONVEYOR_PIN2));
  mc.pinSetup();

  //  nh.advertiseService(light_server);
  //  nh.advertiseService(ir_server);
  //  ir.pinSetup();

  //  nh.advertise(pub_range0);
  //  nh.advertise(pub_range1);
  //  us0.pinSetup();
  //  us1.pinSetup();
  //  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  //  range_msg.field_of_view = 0.1;  // fake
  //  range_msg.min_range = 0.15;
  //  range_msg.max_range = 5.0;
}

long range_time;

void loop()
{
  if (millis() >= range_time)
  {
    //    range_msg.range = us0.getDistance();
    //    range_msg.header.frame_id = "sonar0";
    //    range_msg.header.stamp = nh.now();
    //    pub_range0.publish(&range_msg);

    //    range_msg.range = us1.getDistance();
    //    range_msg.header.frame_id = "sonar1";
    //    range_msg.header.stamp = nh.now();
    //    pub_range1.publish(&range_msg);

    baro_msg.header.frame_id = "/barometer";
    baro_msg.header.stamp = nh.now();
    baro_msg.temperature = baro.getTemperature();
    baro_msg.absolute_alt = baro.getAbsoluteAltitude();
    baro_msg.relative_alt = baro.getRelativeAltitude();
    pub_baro.publish(&baro_msg);

    range_time = millis() + 20;
  }
  nh.spinOnce();
}
