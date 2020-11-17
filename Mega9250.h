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

//#include <Wire.h>
//#include <SPI.h>
#include "MS5611.h"
#include "MPU9250.h"
#include "MadgwickAHRS.h"
#include "infra_red.h"
#include "motors.h"

#define IR_READ_PIN A2
#define IR_LIGHT_PIN A3
#define LINEAR_ACTUATOR_L1 4
#define LINEAR_ACTUATOR_L2 7
#define LINEAR_ACTUATOR_R1 8
#define LINEAR_ACTUATOR_R2 9
#define CONVEYOR_PIN1 10
#define CONVEYOR_PIN2 11
#define ROLLSHUTTER_PIN1 10
#define ROLLSHUTTER_PIN2 11


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
MS5611 MS5611(0x77);

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



/*----baromter----*/
james_msgs::Barometer baro_msg;
ros::Publisher pub_baro("/barometer", &baro_msg);
float referencePressure;

/*----Ultrasound----*/
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);


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


const int adc_pin = 0;

//char frameid[] = "/ultrasound";

double getRange_Ultrasound(int pin_num) {
  int val = 0;
  for (int i = 0; i < 4; i++) val += analogRead(pin_num);
  double range =  val;
  return range / 322.519685;  // (0.0124023437 /4) ; //cvt to meters
}

/*----Scheduler----*/

const unsigned long extra_FREQUENCY = 0;

unsigned long IMU_RUNTIME = 11;
unsigned long BARO_RUNTIME = 22;
unsigned long SONAR_1_RUNTIME = 0;
unsigned long extra_RUNTIME = 0;

unsigned long IMU_COUNT = 0;
unsigned long BARO_COUNT = 1;
unsigned long SONAR_1_COUNT = 1;
unsigned long extra_COUNT = 0;
unsigned long IMU_CYCLE_T = 1000 / IMU_FREQUENCY;
unsigned long Baro_CYCLE_T = 1000 / BARO_FREQUENCY;
unsigned long SONAR_1_CYCLE_T = 1000 / SONAR_1_FREQUENCY;

unsigned long extra_CYCLE_T = 0;
/////////////////////////////////////////////////////////////////////


//#define MPU_CALI_COUNT  70


#define FILTER_NUM    3

void computeIMU() {


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

    if (abs(gyro[axis]) <= 0.003)
    {
      gyro[axis] = 0;
    }
  }

}
//
//
//void computeIMU_1() {
//
//  //  static uint32_t prev_process_time = micros();
//  //  static uint32_t cur_process_time = 0;
//  //  static uint32_t process_time = 0;
//  uint32_t i;
//  static int32_t gyroADC[3][FILTER_NUM] = {0,};
//  int32_t gyroAdcSum;
//
//  gRes = 2000.0 / 32768.0; // 2000dps
//
//  uint32_t axis;
//
//  //  SEN.acc_get_adc();
//  //IMU.gyro_get_adc();
//  //  SEN.mag_get_adc();
//
//  for (axis = 0; axis < 3; axis++)
//  {
//    gyroADC[axis][0] = IMU.gyroADC[axis];
//
//
//    gyroAdcSum = 0;
//    for (i = 0; i < FILTER_NUM; i++)
//    {
//      gyroAdcSum += gyroADC[axis][i];
//    }
//    IMU.gyroADC[axis] = gyroAdcSum / FILTER_NUM;
//    for (i = FILTER_NUM - 1; i > 0; i--)
//    {
//      gyroADC[axis][i] = gyroADC[axis][i - 1];
//    }
//
//    if (abs(IMU.gyroADC[axis]) <= 3)
//    {
//      IMU.gyroADC[axis] = 0;
//    }
//  }
//
//
//  for ( i = 0; i < 3; i++ )
//  {
//    gyroRaw[i]  = IMU.gyroRAW[i];
//    gyroData[i] = IMU.gyroADC[i];
//  }
//  
////  gx = (float)IMU.gyroADC[0]* 0.0010642;
////  gy = (float)IMU.gyroADC[1]* 0.0010642;
////  gz = (float)IMU.gyroADC[2]* 0.0010642;
//
//
// // gx = (float)IMU.gyroADC[0] * gRes;

// // gy = (float)IMU.gyroADC[1] * gRes;
// // gz = (float)IMU.gyroADC[2] * gRes;
//
//
//
//}


void pub_IMU() {
  if (millis() >= IMU_CYCLE_T * IMU_COUNT) {
        IMU.readSensor();
    acc[0] = IMU.getAccelY_mss();// + 0.19;
    acc[1] = IMU.getAccelX_mss();// - 0.06;
    acc[2] = -IMU.getAccelZ_mss() +0.67;

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

      computeIMU();



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


void pub_Baro() {
  if (millis() >= Baro_CYCLE_T * BARO_COUNT) {
    baro_msg.header.frame_id = "/barometer";
    baro_msg.header.stamp = nh.now();

    int result_1 = MS5611.read(12);
    float absolutealt = MS5611.getAltitude(MS5611.getPressure() * 0.01); //cm
    float relativealt = MS5611.getAltitude(MS5611.getPressure() * 0.01, referencePressure);
    float temperature = MS5611.getTemperature() * 0.01; //c

    baro_msg.temperature = temperature;
    baro_msg.absolute_alt = absolutealt;
    baro_msg.relative_alt = relativealt;
    pub_baro.publish(&baro_msg);
    BARO_COUNT++;
  }
}

void pub_Sonar() {
  if (millis() >= SONAR_1_CYCLE_T * SONAR_1_COUNT) {

    range_msg.range = getRange_Ultrasound(1);
    if (range_msg.range < range_msg.min_range || range_msg.range > range_msg.max_range ) range_msg.range = range_msg.max_range;
    range_msg.header.frame_id = "sonar";
    range_msg.header.stamp = nh.now();


    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.field_of_view = 0.6;  // fake
    range_msg.min_range = 0.15;
    range_msg.max_range = 1.5;

    pub_range.publish(&range_msg);


    SONAR_1_COUNT++;
  }
}
