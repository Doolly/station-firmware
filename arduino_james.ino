  /*------ Headers to Include ------*/
#include "infra_red.h"
#include "motors.h"
#include <ros.h>
#include <ros/time.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <james_msgs/MotorControl.h>
#include <std_msgs/String.h>

#define IR_READ_PIN A2
#define IR_LIGHT_PIN A3
#define LINEAR_ACTUATOR_L1 4
#define LINEAR_ACTUATOR_L2 7
#define LINEAR_ACTUATOR_R1 8
#define LINEAR_ACTUATOR_R2 9
#define CONVEYOR_PIN1 10
#define CONVEYOR_PIN2 11

#define ROLLSHUTTER_PIN1 5
#define ROLLSHUTTER_PIN2 6

/*------ Global Variables ------*/
ros::NodeHandle nh;
/*------ test string------*/
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

/*----Motors----*/
MotorController mc;
void motorControlCB(const james_msgs::MotorControl &msg)
{
  mc.control(msg.motor, static_cast<Direction>(msg.direction), msg.pwm);
}
ros::Subscriber<james_msgs::MotorControl> sub("motor_control", &motorControlCB);

/*----IR----*/
InfraRed ir(IR_READ_PIN, IR_LIGHT_PIN);
void lightControl(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response &res) {
  ir.lightControl(req.data);
  res.success = true;
  if (req.data)
    res.message = "Light on";
  else
    res.message = "Light off";
}

void isOccupied(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response &res) {
  res.success = ir.isOccupied();
  if (res.success)
    res.message = "There is a package";
  else
    res.message = "There is not a package";
}

ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> light_server("/ir_light", &lightControl);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> ir_server("/ir", &isOccupied); 



void setup()
{
  nh.initNode();

  nh.subscribe(sub);
  mc.addMotor(new PairedBaseMotor(LINEAR_ACTUATOR_L1, LINEAR_ACTUATOR_L2, LINEAR_ACTUATOR_R1, LINEAR_ACTUATOR_R2));
  mc.addMotor(new BaseMotor(CONVEYOR_PIN1, CONVEYOR_PIN2));
  mc.addMotor(new BaseMotor(ROLLSHUTTER_PIN1, ROLLSHUTTER_PIN2));

  nh.advertise(chatter);
  nh.advertiseService(light_server);
  nh.advertiseService(ir_server);


}

long range_time;

void loop()
{
  if (millis() >= range_time)
  {
    str_msg.data = hello;
    chatter.publish( &str_msg );
   
    range_time = millis() + 20; // 50Hz
  }
  
  nh.spinOnce();
}
