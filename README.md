# Arduino_James

james에 올라갈 arduino code 입니다.

## 적용된 센서
- [x] barometer : MS5611
- [x] ultresound : MB1010 LV-MaxSonar
- [x] IR : --- ( 내부가 비었는지 찼는지 확인용 )
- [ ] Motor ( conveyer control ) **- 추가 예정**
- [ ] Motor ( linear Actuator ) **- 추가 예정**

## Topic & Service
### Topic (20 hz)
- /sonar0 (ultrasound)
- /temperature (barometer)
- /absolute_alt (barometer)
- /relative_alt (barometer)
### Service
- /ir       ( std_srvs/Trigger)
- /ir_light ( std_srvs/SetBool )

---

## ISSUE (20. 06. 26 기준)
 - PC 와 ROS로 연결 시 PC의 rosserial package 의 최신 버전에서 service response를 받을 수 없음. (18년 12월 발생한 문제지만 수정되지 않고 있음)

해결 방법 : https://github.com/ros-drivers/rosserial 에서 0.7.7 버전 받아서 사용 
