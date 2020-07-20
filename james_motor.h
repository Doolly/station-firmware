#ifndef _JAMES_MOTOR_H_
#define _JAMES_MOTOR_H_

#include "Arduino.h"

class BaseMotor
{
private:
  const int pin_1_;
  const int pin_2_;

public:
  BaseMotor() = delete;
  BaseMotor(const int, const int);
  virtual void pinSetup() const;
  virtual void driveForward(int val = 255) const;
  virtual void driveReverse(int val = 255) const;
  virtual void stop() const;
};

BaseMotor::BaseMotor(const int pin_1, const int pin_2) : pin_1_(pin_1), pin_2_(pin_2) {}

void BaseMotor::pinSetup() const
{
  pinMode(pin_1_, OUTPUT);
  pinMode(pin_2_, OUTPUT);
}

void BaseMotor::driveForward(int val) const
{
  digitalWrite(pin_1_, HIGH);
  digitalWrite(pin_2_, LOW);
}

void BaseMotor::driveReverse(int val) const
{
  digitalWrite(pin_1_, LOW);
  digitalWrite(pin_2_, HIGH);
}

void BaseMotor::stop() const
{
  digitalWrite(pin_1_, LOW);
  digitalWrite(pin_2_, LOW);
}

class PWMMotor : public BaseMotor
{
private:
  const int pwm_pin_;

public:
  PWMMotor() = delete;
  PWMMotor(const int, const int, const int);
  void pinSetup() const override;
  void driveForward(int) const override;
  void driveReverse(int) const override;
};

PWMMotor::PWMMotor(const int pin_1, const int pin_2, const int pwm_pin) : BaseMotor(pin_1, pin_2), pwm_pin_(pwm_pin) {}

void PWMMotor::pinSetup() const
{
  BaseMotor::pinSetup();
  pinMode(pwm_pin_, OUTPUT);
}

void PWMMotor::driveForward(int val) const
{
  BaseMotor::driveForward();
  if (val > 255)
    val = 255;
  if (val < 0)
    val = 0;
  analogWrite(pwm_pin_, val);
}

void PWMMotor::driveReverse(int val) const
{
  BaseMotor::driveReverse();
  if (val > 255)
    val = 255;
  if (val < 0)
    val = 0;
  analogWrite(pwm_pin_, val);
}

class PairedBaseMotor : public BaseMotor
{
private:
  BaseMotor motor2_;

public:
  PairedBaseMotor() = delete;
  PairedBaseMotor(const int, const int, const int, const int);
  void pinSetup() const override;
  void driveForward(int) const override;
  void driveReverse(int) const override;
  void stop() const override;
};

PairedBaseMotor::PairedBaseMotor(const int motor1_pin1, const int motor1_pin2, const int motor2_pin1, const int motor2_pin2)
    : BaseMotor(motor1_pin1, motor1_pin2), motor2_(motor2_pin1, motor2_pin2) {}

void PairedBaseMotor::pinSetup() const
{
  BaseMotor::pinSetup();
  motor2_.pinSetup();
}

void PairedBaseMotor::driveForward(int val) const
{
  BaseMotor::driveForward(val);
  motor2_.driveForward(val);
}

void PairedBaseMotor::driveReverse(int val) const
{
  BaseMotor::driveReverse(val);
  motor2_.driveReverse(val);
}

void PairedBaseMotor::stop() const
{
  BaseMotor::stop();
  motor2_.stop();
}

class PairedPWMMotor : public PWMMotor
{
private:
  PWMMotor motor2_;

public:
  PairedPWMMotor() = delete;
  PairedPWMMotor(const int, const int, const int, const int, const int, const int);
  void pinSetup() const override;
  void driveForward(int) const override;
  void driveReverse(int) const override;
  void stop() const override;
};

PairedPWMMotor::PairedPWMMotor(const int motor1_pin1, const int motor1_pin2, const int motor1_pwm, const int motor2_pin1, const int motor2_pin2, const int motor2_pwm)
    : PWMMotor(motor1_pin1, motor1_pin2, motor1_pwm), motor2_(motor2_pin1, motor2_pin2, motor2_pwm) {}

void PairedPWMMotor::pinSetup() const
{
  PWMMotor::pinSetup();
  motor2_.pinSetup();
}

void PairedPWMMotor::driveForward(int val) const
{
  PWMMotor::driveForward(val);
  motor2_.driveForward(val);
}

void PairedPWMMotor::driveReverse(int val) const
{
  PWMMotor::driveReverse(val);
  motor2_.driveReverse(val);
}

void PairedPWMMotor::stop() const
{
  PWMMotor::stop();
  motor2_.stop();
}

enum Direction
{
  REVERSE = -1,
  STOP,
  FORWARD,
};

class MotorController
{
private:
  int alloc_motor_;
  int current_motor_;
  BaseMotor **motors_;

public:
  MotorController(const int = 5);
  ~MotorController();
  MotorController &addMotor(BaseMotor *motor);
  MotorController &pinSetup();
  int currentMotorNum() const;
  MotorController &control(const int, Direction, const int pwm = 255);
};

MotorController::MotorController(const int alloc_motor) : alloc_motor_(alloc_motor), current_motor_(0)
{
  motors_ = new BaseMotor *[alloc_motor_];
}

MotorController::~MotorController()
{
  for (int i = 0; i < current_motor_; i++)
    delete motors_[i];
  delete[] motors_;
}

MotorController &MotorController::addMotor(BaseMotor *motor)
{
  motors_[current_motor_] = motor;
  current_motor_++;
  return *this;
}

MotorController &MotorController::pinSetup()
{
  for (int i = 0; i < current_motor_; i++)
    motors_[i]->pinSetup();
  return *this;
}

int MotorController::currentMotorNum() const
{
  return current_motor_;
}

MotorController &MotorController::control(const int motor_address, Direction direction, const int pwm)
{
  if (motor_address >= current_motor_ || motor_address < 0)
    return *this;
  if (direction == FORWARD)
    motors_[motor_address]->driveForward(pwm);
  else if (direction == REVERSE)
    motors_[motor_address]->driveReverse(pwm);
  else
    motors_[motor_address]->stop();
  return *this;
}

#endif
