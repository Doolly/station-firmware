# Arduino_James
# MEGA ver.
james에 올라갈 arduino code 입니다.

## 적용된 센서
- [x] barometer : MS5611
- [x] ultresound : MB1010 LV-MaxSonar
- [x] IR : --- ( 내부가 비었는지 찼는지 확인용 )
- [x] Motor ( linear Actuator ) 
- [ ] Motor ( conveyer control ) **- 추가 예정**

## Topic & Service
### Topic (20 hz) - Publish
- /sonar0 (sensor_msgs/Range)
- /barometer (james_msgs/Barometer) : header, temperature, absolute_alt, relative_alt
- /imu_mpu9250 (james_msgs/imu_msgs) : acc, gyro ,quat

### Topic - Subscribe
- /motor_control (james_msgs/MotorControl) : motor, direction, pwm
#### motor control example
```
rostopic pub /motor_control james_msgs/MotorControl {0,0,255}
```
### Service
- /ir       ( std_srvs/Trigger)
- /ir_light ( std_srvs/SetBool )
