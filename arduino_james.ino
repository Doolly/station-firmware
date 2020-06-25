/*------ Headers to Include ------*/
#include "barometer.h"
#include "infra_red.h"
#include "ultra_sound.h"
#include <ros.h>
#include <ros/time.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#define IR_READ_PIN 2
#define IR_LIGHT_PIN 3
#define US0_ANALOG_PIN A0
#define US1_ANALOG_PIN A1

/*------ Global Variables ------*/
ros::NodeHandle nh;
std_msgs::Float64 temperature, relative_alt, absolute_alt;
sensor_msgs::Range range_msg;
Barometer baro;
InfraRed ir(IR_READ_PIN, IR_LIGHT_PIN);
UltraSound us0(US0_ANALOG_PIN), us1(US1_ANALOG_PIN);

bool lightControl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response &res) {
  ir.lightControl(req.data);
  res.success = true;
  if (req.data)
    res.message = "Light on";
  else
    res.message = "Light off";
  return res.success;
}

bool isOccupied(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response &res) {
  res.success = ir.isOccupied();
  if (res.success)
    res.message = "There is a package";
  else
    res.message = "There is not a package";
  return res.success;
}

ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> light_server("/ir_light", &lightControl);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> ir_server("/ir", &isOccupied);
ros::Publisher pub_range0( "/sonar0", &range_msg);
//ros::Publisher pub_range1( "/sonar1", &range_msg);
ros::Publisher pub_temp( "/temperature", &temperature);
ros::Publisher pub_absolute_alt( "/absolute_alt", &absolute_alt);
ros::Publisher pub_relative_alt( "/relative_alt", &relative_alt);

void setup()
{
  nh.initNode();
  nh.advertiseService(light_server);
  nh.advertiseService(ir_server);
  nh.advertise(pub_range0);
//  nh.advertise(pub_range1);
  nh.advertise(pub_temp);
  nh.advertise(pub_absolute_alt);
  nh.advertise(pub_relative_alt);

  baro.setReference();
  ir.pinSetup();
  us0.pinSetup();
//  us1.pinSetup();

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.15;
  range_msg.max_range = 5.0;
}

long range_time;

void loop()
{
  if ( millis() >= range_time ) {
    range_msg.range = us0.getDistance();
    range_msg.header.frame_id = "/sonar0";
    range_msg.header.stamp = nh.now();
    pub_range0.publish(&range_msg);

//    range_msg.range = us1.getDistance();
//    range_msg.header.frame_id = "/sonar1";
//    range_msg.header.stamp = nh.now();
//    pub_range1.publish(&range_msg);

    temperature.data = baro.getTemperature();
    pub_temp.publish(&temperature);

    absolute_alt.data = baro.getAbsoluteAltitude();
    pub_absolute_alt.publish(&absolute_alt);

    relative_alt.data = baro.getRelativeAltitude();
    pub_relative_alt.publish(&relative_alt);

    range_time =  millis() + 20;
  }
  nh.spinOnce();
}
