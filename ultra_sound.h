#ifndef _ULTRASOUND_H_
#define _ULTRASOUND_H_

#include "Arduino.h"

class UltraSound
{
private:
  const int analog_pin_;

public:
  UltraSound() = delete;
  UltraSound(const int);
  double getDistance() const;
};

UltraSound::UltraSound(const int analog_pin) : analog_pin_(analog_pin)
{
  pinMode(analog_pin_, INPUT);
}

double UltraSound::getDistance() const
{
  // 센서 안정화 시간을 고려하여 20Hz 정도로만 콜 할 것
  int val = 0;
  for (int i = 0; i < 4; i++)
    val += analogRead(analog_pin_);
  double range = val;
  return range / 322.519685; // (0.0124023437 /4) ; //cvt to meters
}

#endif
