////////////////////////////////////////////
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <math.h>
#include <stdio.h> //Scheduler
#include <string.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>

#include "MPU9250.h"
#include "barometer.h"
#include "MadgwickAHRS.h"
#include "infra_red.h"
#include "ultra_sound.h"
#include "motors.h"

#define IR_READ_PIN A2
#define IR_LIGHT_PIN A3
#define US0_ANALOG_PIN A0
#define US1_ANALOG_PIN A1
#define LINEAR_ACTUATOR_L1 4
#define LINEAR_ACTUATOR_L2 7
#define LINEAR_ACTUATOR_R1 8
#define LINEAR_ACTUATOR_R2 9
#define CONVEYOR_PIN1 10
#define CONVEYOR_PIN2 11
#define ROLLSHUTTER_PIN1 5
#define ROLLSHUTTER_PIN2 6

/*-----msgs-----*/
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <james_msgs/Barometer.h>
#include <james_msgs/MotorControl.h>

#define FILTER_NUM    3
///////////////////////////////////////////////////////
ros::NodeHandle nh;

/*-------TF-------*/
//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;

/*-------IMU-------*/
sensor_msgs::Imu imu;
ros::Publisher pub("imumsgs_mpu9250", &imu);
MPU9250 IMU(SPI, 12);


//MS5611 MS5611(12);
float acc[3], gyro[3], mag[3];
float D1, D2;
uint16_t fc[6];
int64_t OFF2, SENS2;
int32_t TEMP2;

int16_t gyroData[3];
int16_t gyroRaw[3];
int16_t accData[3];
int16_t accRaw[3];
int16_t magData[3];
int16_t magRaw[3];
float gx, gy, gz;

float gRes;

unsigned long IMU_COUNT = 0;
unsigned long BARO_COUNT = 1;
unsigned long SONAR_1_COUNT = 1;
unsigned long extra_COUNT = 0;

/*----baromter----*/
james_msgs::Barometer baro_msg;
ros::Publisher pub_baro("/barometer", &baro_msg);
Barometer baro;

/*----Ultrasound----*/
sensor_msgs::Range range_msg;
UltraSound us0(US0_ANALOG_PIN), us1(US1_ANALOG_PIN);
ros::Publisher pub_range0( "/sonar0", &range_msg);
ros::Publisher pub_range1( "/sonar1", &range_msg);

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
    res.message = "There is not a pacdpdkage";
}

ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> light_server("/ir_light", &lightControl);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> ir_server("/ir", &isOccupied);

/////////////////////////////////////////////////////////////////////

#define FILTER_NUM    3

void computeIMU_gyro() {


  uint32_t i;
  static float gyroADC[3][FILTER_NUM] = {0,};
  float gyroAdcSum;

  uint32_t axis;


  for (axis = 0; axis < 3; axis++)
  {
    gyroADC[axis][0] = gyro[axis];

    gyroAdcSum = 0;
    for (i = 0; i < FILTER_NUM; i++)
    {
      gyroAdcSum += gyroADC[axis][i];
    }
    gyro[axis] = gyroAdcSum / FILTER_NUM;
    for (i = FILTER_NUM - 1; i > 0; i--)
    {
      gyroADC[axis][i] = gyroADC[axis][i - 1];
    }

    if (abs(gyro[axis]) <= 0.03)
    {
      gyro[axis] = 0;
    }
  }

}
void computeIMU_acc() {


  uint32_t i;
  static float accADC[3][FILTER_NUM] = {0,};
  float accAdcSum;

  uint32_t axis;


  for (axis = 0; axis < 3; axis++)
  {
    accADC[axis][0] = acc[axis];

    accAdcSum = 0;
    for (i = 0; i < FILTER_NUM; i++)
    {
      accAdcSum += accADC[axis][i];
    }
    acc[axis] = accAdcSum / FILTER_NUM;
    for (i = FILTER_NUM - 1; i > 0; i--)
    {
      accADC[axis][i] = accADC[axis][i - 1];
    }

    if (abs(acc[0]) <= 0.1)
    {
      acc[0] = 0;
    }

    if (abs(acc[1]) <= 0.1)
    {
      acc[1] = 0;
    }

    //    if (abs(acc[axis]) <= 0.03)
    //    {
    //      acc[axis] = 0;
    //    }

  }


}


void pub_IMU(const unsigned long IMU_FREQUENCY) {

  unsigned long IMU_CYCLE_T = 1000 / IMU_FREQUENCY;


  if (millis() >= IMU_CYCLE_T * IMU_COUNT) {
    IMU.readSensor();
    acc[0] = IMU.getAccelY_mss() - 1.24; // + 0.19;
    acc[1] = IMU.getAccelX_mss() - 0.47; // - 0.06;
    acc[2] = -IMU.getAccelZ_mss() + 1.2;

    gyro[0] = IMU.getGyroY_rads();
    gyro[1] = IMU.getGyroX_rads();
    gyro[2] = -IMU.getGyroZ_rads();
    //gyro[0] = IMU.gyroADC[0];
    //gyro[1] = IMU.gyroADC[1];
    //gyro[2] = -IMU.gyroADC[2];

    mag[0] = IMU.getMagY_uT();
    mag[1] = IMU.getMagX_uT();
    mag[2] = IMU.getMagZ_uT();

    //    gyro_common();

    computeIMU_gyro();
    computeIMU_acc();



    MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);



    imu.header.frame_id = "imu_link_9250";
    imu.header.stamp = nh.now();
    imu.orientation.w = q0;
    imu.orientation.x = q1;
    imu.orientation.y = q2;
    imu.orientation.z = q3;

    imu.linear_acceleration.x = acc[0];
    imu.linear_acceleration.y = acc[1];
    imu.linear_acceleration.z = acc[2];

    imu.angular_velocity.x = gyro[0];
    imu.angular_velocity.y = gyro[1];
    imu.angular_velocity.z = gyro[2];

    pub.publish(&imu);
    IMU_COUNT++;

  }
}
void pub_Baro(unsigned long BARO_FREQUENCY) {

  unsigned long Baro_CYCLE_T = 1000 / BARO_FREQUENCY;

  if (millis() >= Baro_CYCLE_T * BARO_COUNT) {
    baro_msg.header.frame_id = "/barometer";
    baro_msg.header.stamp = nh.now();

    baro_msg.temperature = baro.getTemperature();
    baro_msg.absolute_alt = baro.getAbsoluteAltitude();
    baro_msg.relative_alt = baro.getRelativeAltitude();
    pub_baro.publish(&baro_msg);

    BARO_COUNT++;
  }
}
void pub_Sonar(unsigned long SONAR_1_FREQUENCY) {
  
  unsigned long SONAR_1_CYCLE_T = 1000 / SONAR_1_FREQUENCY;

  if (millis() >= SONAR_1_CYCLE_T * SONAR_1_COUNT) {

    range_msg.range = us0.getDistance();
    if (range_msg.range < range_msg.min_range || range_msg.range > range_msg.max_range ) range_msg.range = range_msg.max_range;
    range_msg.header.frame_id = "sonar0";
    range_msg.header.stamp = nh.now();
    pub_range0.publish(&range_msg);

    //    range_msg.range = us1.getDistance();
    //    range_msg.header.frame_id = "sonar1";
    //    range_msg.header.stamp = nh.now();
    //    pub_range1.publish(&range_msg);

    SONAR_1_COUNT++;
  }
}


