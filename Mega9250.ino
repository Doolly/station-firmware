#include "Mega9250.h"

void setup(){
  SPI.begin();
  IMU.begin();

  int IMU_gyroStatus = IMU.calibrateGyro();
  int IMU_setAccelRange = IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  int IMU_accelStatus = IMU.calibrateAccel();
  int IMU_magStatus = IMU.calibrateMag();


  nh.initNode();
  nh.subscribe(sub);

  //  broadcaster.init(nh);
  nh.advertise(pub);
  nh.advertise(pub_baro);
  baro.setReference();

  nh.advertise(pub_range0);
  nh.advertise(pub_range1);
  
  mc.addMotor(new PairedBaseMotor(LINEAR_ACTUATOR_L1, LINEAR_ACTUATOR_L2, LINEAR_ACTUATOR_R1, LINEAR_ACTUATOR_R2));
  mc.addMotor(new BaseMotor(CONVEYOR_PIN1, CONVEYOR_PIN2));
  mc.addMotor(new BaseMotor(ROLLSHUTTER_PIN1, ROLLSHUTTER_PIN2));

  nh.advertiseService(light_server);
  nh.advertiseService(ir_server);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.6;  // fake
  range_msg.min_range = 0.15;
  range_msg.max_range = 1.5;
}

void loop(){
  //number inside the '()' is the rate iof each sensors.//
  pub_IMU(50);
  pub_Baro(14);
  pub_Sonar(15);
  nh.spinOnce();
}
