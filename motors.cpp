#include "motors.h"

BaseMotor::BaseMotor(const uint8_t pin_1, const uint8_t pin_2) : pin1_(pin_1), pin2_(pin_2) {
    pinMode(pin1_, OUTPUT);
    pinMode(pin2_, OUTPUT);
    motor_type_ = BASE_MOTOR;
}

Motors BaseMotor::motorType() const {
    return motor_type_;
}

void BaseMotor::control(const Direction direction) const {
    if (direction == FORWARD) {
        digitalWrite(pin1_, HIGH);
        digitalWrite(pin2_, LOW);

    } else if (direction == REVERSE) {
        digitalWrite(pin1_, LOW);
        digitalWrite(pin2_, HIGH);
    } else {
        digitalWrite(pin1_, LOW);
        digitalWrite(pin2_, LOW);
    }
}

PWMMotor::PWMMotor(const uint8_t pin_1, const uint8_t pin_2, const uint8_t pwm_pin) : BaseMotor(pin_1, pin_2), pwm_pin_(pwm_pin), max_pwm_(255) {
    pinMode(pwm_pin_, OUTPUT);
    motor_type_ = PWM_MOTOR;
}

void PWMMotor::control(const Direction direction, int pwm) const {
    BaseMotor::control(direction);
    pwm = limit(pwm, 0, max_pwm_);
    analogWrite(pwm_pin_, pwm);
}

PWMMotor &PWMMotor::setMaxPWM(const int max_pwm) {
    max_pwm_ = max_pwm;
    return *this;
}

PairedBaseMotor::PairedBaseMotor(const uint8_t motor1_pin1, const uint8_t motor1_pin2, const uint8_t motor2_pin1, const uint8_t motor2_pin2)
    : BaseMotor(motor1_pin1, motor1_pin2), motor2_(motor2_pin1, motor2_pin2) {
    motor_type_ = PAIRED_BASE_MOTOR;
}

void PairedBaseMotor::control(const Direction direction) const {
    BaseMotor::control(direction);
    motor2_.control(direction);
}

PairedPWMMotor::PairedPWMMotor(const uint8_t motor1_pin1, const uint8_t motor1_pin2, const uint8_t motor1_pwm, const uint8_t motor2_pin1, const uint8_t motor2_pin2, const uint8_t motor2_pwm)
    : PWMMotor(motor1_pin1, motor1_pin2, motor1_pwm), motor2_(motor2_pin1, motor2_pin2, motor2_pwm) {
    motor_type_ = PAIRED_PWM_MOTOR;
}

void PairedPWMMotor::control(const Direction direction, int pwm) const {
    PWMMotor::control(direction, pwm);
    motor2_.control(direction, pwm);
}

Encoder::Encoder(const uint8_t intterupt, const uint8_t digital) : interrupt_pin_(intterupt), digital_pin_(digital), cnt_(0), state_(false), direction_(true), resolution_(1920) {
    pinMode(digital_pin_, INPUT);
}

Encoder &Encoder::counter() {
    bool state_now = digitalRead(interrupt_pin_);
    if ((state_ == LOW) && state_now == HIGH) {
        bool val = digitalRead(digital_pin_);
        if (val != direction_) {
            direction_ = val;
        }
    }
    state_ = state_now;

    if (!direction_)
        cnt_++;
    else
        cnt_--;

    if (cnt_ < 0)
        cnt_ += 100000000;
    else if (cnt_ >= 100000000)
        cnt_ = 0;
    return *this;
}

Encoder &Encoder::setResolution(const int resolution) {
    resolution_ = resolution;
    return *this;
}

long Encoder::getCnt() const {
    return cnt_;
}

EncoderMotor::EncoderMotor(const uint8_t motor_pin1, const uint8_t motor_pin2, const uint8_t motor_pwm, const uint8_t encoder_int, const uint8_t encoder_digi)
    : PWMMotor(motor_pin1, motor_pin2, motor_pwm), Encoder(encoder_int, encoder_digi), angular_velocity_(0), target_velocity_(0), pwm_(0), prev_cnt_(0), control_period_(8), kp_(0.8) {
    motor_type_ = ENCODER_MOTOR;
}

EncoderMotor &EncoderMotor::setControlPeriod(const uint8_t control_period) {
    control_period_ = control_period;
    return *this;
}

EncoderMotor &EncoderMotor::setKp(const double kp) {
    kp_ = kp;
    return *this;
}

EncoderMotor &EncoderMotor::setTarget(const double target_angular_velocity) {
    target_velocity_ = target_angular_velocity;
    return *this;
}

EncoderMotor &EncoderMotor::control() {
    int diff_cnt = Encoder::getCnt() - prev_cnt_;
    prev_cnt_ = Encoder::getCnt();

    if (abs(diff_cnt) < 10000)
        angular_velocity_ = 2 * PI * (diff_cnt)*1000 / (resolution_ * control_period_);

    double err = (fabs(target_velocity_) - fabs(angular_velocity_)) * 5;
    pwm_ += static_cast<int>(err * kp_);
    pwm_ = limit(pwm_, 20, 255);

    if (target_velocity_ > 0) {
        PWMMotor::control(Direction::FORWARD, pwm_);
    } else if (target_velocity_ < 0) {
        PWMMotor::control(Direction::REVERSE, pwm_);
    } else {
        BaseMotor::control(Direction::STOP);
    }

    return *this;
}

double EncoderMotor::getAngularVelocity() const {
    return angular_velocity_;
}

uint8_t EncoderMotor::getPWM() const {
    return pwm_;
}

PulseControlledMotor::PulseControlledMotor(const uint8_t enable, const uint8_t cw, const uint8_t ccw)
    : BaseMotor(cw, ccw), enable_(enable), resolution_(4000), wheel_radius_(0.012), position_(0), target_position_(0) {
    pinMode(enable_, OUTPUT);
    motor_type_ = PULSE_CONTROLLED_MOTOR;
}

void PulseControlledMotor::control(const Direction direction) {
    if (direction == FORWARD) {
        digitalWrite(pin1_, HIGH);
        delayMicroseconds(1);
        digitalWrite(pin1_, LOW);
        delayMicroseconds(1);
        position_ += tick_dist_;
    }
    if (direction == REVERSE) {
        digitalWrite(pin2_, HIGH);
        delayMicroseconds(1);
        digitalWrite(pin2_, LOW);
        delayMicroseconds(1);
        position_ -= tick_dist_;
    }
}

PulseControlledMotor &PulseControlledMotor::setResolution(const int resolution) {
    resolution_ = resolution;
    tick_dist_ = wheel_radius_ * 2 * PI / resolution_;
    return *this;
}

PulseControlledMotor &PulseControlledMotor::setWheelRadius(const double wheel_radius) {
    wheel_radius_ = wheel_radius;
    resolution_ == 0 ? tick_dist_ = 1 : (tick_dist_ = wheel_radius_ * 2 * PI / resolution_);
    return *this;
}

PulseControlledMotor &PulseControlledMotor::setTarget(const double target) {
    target_position_ = target;
    return *this;
}

PulseControlledMotor &PulseControlledMotor::control() {
    digitalWrite(enable_, LOW);
    double err = (target_position_ - position_) / tick_dist_;
    if (err > 0.001) {
        control(Direction::FORWARD);
    } else if (err < -0.001) {
        control(Direction::REVERSE);
    } else {
        control(Direction::STOP);
    }
    return *this;
}

MotorController::MotorController(const uint8_t alloc_motor) : alloc_motor_(alloc_motor), current_motor_(0) {
    motors_ = new BaseMotor *[alloc_motor_];
}

MotorController::~MotorController() {
    for (int i = 0; i < current_motor_; i++)
        delete motors_[i];
    delete[] motors_;
}

MotorController &MotorController::addMotor(BaseMotor *motor) {
    motors_[current_motor_] = motor;
    current_motor_++;
    return *this;
}

int MotorController::currentMotorNum() const {
    return current_motor_;
}

MotorController &MotorController::control(const int motor_address, Direction direction, int pwm) {
    if (motor_address >= current_motor_ || motor_address < 0)
        return *this;
    Motors motor = motors_[motor_address]->motorType();
    if (motor == BASE_MOTOR || motor == PAIRED_BASE_MOTOR) {
        motors_[motor_address]->control(direction);
    } else if (motor == PWM_MOTOR || motor == PAIRED_PWM_MOTOR) {
        PWMMotor *motor = (PWMMotor *)motors_[motor_address];
        motor->control(direction, pwm);
    } else if (motor == ENCODER_MOTOR) {
        EncoderMotor *motor = (EncoderMotor *)motors_[motor_address];
        motor->control();
    } else if (motor == PULSE_CONTROLLED_MOTOR) {
        PulseControlledMotor *motor = (PulseControlledMotor *)motors_[motor_address];
        motor->control();
    }
    return *this;
}

Vehicle::Vehicle(const uint8_t alloc_motor) : MotorController(alloc_motor), wheel_radius_(0.026), width_between_wheels_(0.1), linear_x_(0), angular_z_(0), pose_x_(0), pose_y_(0), theta_(0) {}

Vehicle::~Vehicle() {}

Vehicle &Vehicle::setWheelRadius(const double radius) {
    wheel_radius_ = radius;
    return *this;
}

Vehicle &Vehicle::setWidth(const double width_between_wheels) {
    width_between_wheels_ = width_between_wheels;
    return *this;
}

Vehicle &Vehicle::control() {
    for (int i = 0; i < current_motor_; i++) {
        if (motors_[i]->motorType() == ENCODER_MOTOR) {
            EncoderMotor *motor = (EncoderMotor *)motors_[i];
            motor->control();
        }
    }
    return *this;
}

Vehicle &Vehicle::setVelocity(double linear_x, double angular_z) {
    double target[2] = {0, 0};
    target[0] = linear_x + (angular_z * width_between_wheels_ / 2);
    target[1] = -(linear_x - (angular_z * width_between_wheels_ / 2));
    for (int i = 0; i < current_motor_; i++) {
        if (motors_[i]->motorType() == ENCODER_MOTOR) {
            EncoderMotor *motor = (EncoderMotor *)motors_[i];
            motor->setTarget(target[i]);
        }
    }
    return *this;
}