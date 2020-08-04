#ifndef MOTORS_H_
#define MOTORS_H_

#include "Arduino.h"

template <typename T>
T limit(T val, T min, T max) {
    if (val > max) {
        val = max;
    } else if (val < min) {
        val = min;
    }
    return val;
}

enum Motors {
    BASE_MOTOR,
    PWM_MOTOR,
    PAIRED_BASE_MOTOR,
    PAIRED_PWM_MOTOR,
    ENCODER_MOTOR
};

class BaseMotor {
   private:
    const uint8_t pin_1_;
    const uint8_t pin_2_;

   protected:
    Motors motor_type_;

   public:
    BaseMotor() = delete;
    BaseMotor(const uint8_t, const uint8_t);
    virtual Motors motorType() const;
    virtual void driveForward(const int val = 255) const;
    virtual void driveReverse(const int val = 255) const;
    virtual void stop() const;
};

class PWMMotor : public BaseMotor {
   private:
    const uint8_t pwm_pin_;
    int max_pwm_;

   public:
    PWMMotor() = delete;
    PWMMotor(const uint8_t, const uint8_t, const uint8_t);
    void driveForward(int) const override;
    void driveReverse(int) const override;
    PWMMotor &setMaxPWM(const int);
};

class PairedBaseMotor : public BaseMotor {
   private:
    BaseMotor motor2_;

   public:
    PairedBaseMotor() = delete;
    PairedBaseMotor(const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    void driveForward(const int) const override;
    void driveReverse(const int) const override;
    void stop() const override;
};

class PairedPWMMotor : public PWMMotor {
   private:
    PWMMotor motor2_;

   public:
    PairedPWMMotor() = delete;
    PairedPWMMotor(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    void driveForward(int) const override;
    void driveReverse(int) const override;
    void stop() const override;
};

class Encoder {
   protected:
    const uint8_t interrupt_pin_;
    const uint8_t digital_pin_;
    long cnt_;
    bool state_;
    bool direction_;
    int resolution_;

   public:
    Encoder() = delete;
    Encoder(const uint8_t, const uint8_t);
    Encoder &counter();
    Encoder &setResolution(const int);
    long getCnt() const;
};

class EncoderMotor : public PWMMotor, public Encoder {
   private:
    double angular_velocity_;
    double target_velocity_;
    int pwm_;
    long prev_cnt_;
    uint8_t control_period_;  // ms 제어주기
    double kp_;

   public:
    EncoderMotor() = delete;
    EncoderMotor(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    EncoderMotor &setControlPeriod(const uint8_t);
    EncoderMotor &setKp(const double);
    EncoderMotor &setTarget(const double);
    EncoderMotor &control();
    double getAngularVelocity() const;
    uint8_t getPWM() const;
};

enum Direction {
    REVERSE = -1,
    STOP,
    FORWARD,
};

class MotorController {
   protected:
    uint8_t alloc_motor_;
    uint8_t current_motor_;
    BaseMotor **motors_;

   public:
    MotorController(const uint8_t = 5);
    virtual ~MotorController();
    MotorController &addMotor(BaseMotor *motor);
    int currentMotorNum() const;
    MotorController &control(const int, Direction, int);
};

class Vehicle : public MotorController {
   private:
    double wheel_radius_;
    double width_between_wheels_;
    double linear_x_;
    double angular_z_;
    double pose_x_;
    double pose_y_;
    double theta_;

   public:
    Vehicle(const uint8_t = 2);
    ~Vehicle();
    Vehicle &setWheelRadius(const double);
    Vehicle &setWidth(const double);
    Vehicle &control();
    Vehicle &setVelocity(double, double);
};

#endif