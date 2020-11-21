const unsigned long IMU_FREQUENCY = 50;
const unsigned long SONAR_1_FREQUENCY = 20;
const unsigned long BARO_FREQUENCY = 20;

#include "Mega9250.h"

void setup() {
  SPI.begin();
  IMU.begin();
  
  int IMU_gyroStatus = IMU.calibrateGyro();
  int IMU_setAccelRange = IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);

  int IMU_accelStatus = IMU.calibrateAccel();
  int IMU_magStatus = IMU.calibrateMag();


  Wire.begin();
  MS5611.begin();

  ///// Reference_pressure setting :) //////
  int result = MS5611.read();
  referencePressure = MS5611.getPressure() * 0.01 ;

  nh.initNode();
  nh.subscribe(sub);

  //  broadcaster.init(nh);
  nh.advertise(pub);
  nh.advertise(pub_baro);
  nh.advertise(pub_range);

  mc.addMotor(new PairedBaseMotor(LINEAR_ACTUATOR_L1, LINEAR_ACTUATOR_L2, LINEAR_ACTUATOR_R1, LINEAR_ACTUATOR_R2));
  mc.addMotor(new BaseMotor(CONVEYOR_PIN1, CONVEYOR_PIN2));
  mc.addMotor(new BaseMotor(ROLLSHUTTER_PIN1, ROLLSHUTTER_PIN2));

  nh.advertiseService(light_server);
  nh.advertiseService(ir_server);

  nh.advertise(pub_range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.6;  // fake
  range_msg.min_range = 0.15;
  range_msg.max_range = 1.5;


}

int count_IMU;
long range_time_imu, range_time_baro;

void loop()
{
  pub_IMU();
  pub_Baro();
  pub_Sonar();
  nh.spinOnce();

}
