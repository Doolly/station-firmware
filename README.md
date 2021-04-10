# Arduino_W-station

W-station 올라갈 arduino code 입니다.

## 적용된 센서
- [x] IR : --- ( 내부가 비었는지 찼는지 확인용 )
- [x] Motor ( linear Actuator ) 
- [ ] Motor ( conveyer control ) 

## Topic & Service

### Topic - Subscribe
- /motor_control (james_msgs/MotorControl) : motor, direction, pwm
#### motor control example
```
rostopic pub /motor_control james_msgs/MotorControl {0,0,255}
```
### Service
- /ir       ( std_srvs/Trigger)
- /ir_light ( std_srvs/SetBool )

---

## ISSUE (20. 06. 26 기준)
 - PC 와 ROS로 연결 시 PC의 rosserial package 의 최신 버전에서 service response를 받을 수 없음. (18년 12월 발생한 문제지만 수정되지 않고 있음)

### 해결 방법

Install rosserial for Arduino by running

```bash
sudo apt-get install ros-melodic-rosserial-arduino ros-melodic-rosserial
```

Install ros_lib into the Arduino Environment
> 만약 arduino IDE가 설치되어있지 않다면 [다음 링크](https://emanual.robotis.com/docs/en/software/arduino_ide/) 참조

```bash
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

# If you are building the Arduino on Windows, you need to create the ros_lib folder in some convenient directory.
cd <some_empty_directory>
rosrun rosserial_arduino make_libraries.py .
```

terminal에서 여러 메시지들이 빌드되는 것이 보이는데, 그 중 James*가 보이면 성공한 것임

이제 문제가 되는 rosserial을 0.7.7 버전으로 사용하게 바꾼다.

```bash
sudo apt-get purge ros-melodic-rosserial
```

[rosserial github](https://github.com/ros-drivers/rosserial/releases/tag/0.7.7)에서 0.7.7 버전 다운 (`catkin_ws/src` 위치에 )
이후, catkin_make를 해주면 완성

> 절대 이 이후 catkin_install은 하지 말 것!!! 


