#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

/* Pin Configuration */
#define DEBUG_LED1_PIN                                  (39U)
#define DEBUG_LED2_PIN                                  (40U)
#define DEBUG_LED3_PIN                                  (41U)
#define DEBUG_LED4_PIN                                  (42U)

/* 
    Motor - Relay 
    Relay 쪽 COM 단자에 모두 GND 물려줘야 합니다.
*/
#define FLOOR1_MOTOR_RELAY_SWITCH1_PIN                  (62U)
#define FLOOR1_MOTOR_RELAY_SWITCH2_PIN                  (63U)
#define FLOOR2_MOTOR_RELAY_SWITCH1_PIN                  (64U)
#define FLOOR2_MOTOR_RELAY_SWITCH2_PIN                  (65U)
#define FLOOR3_MOTOR_RELAY_SWITCH1_PIN                  (66U)
#define FLOOR3_MOTOR_RELAY_SWITCH2_PIN                  (67U)
#define LIFT_CONVEYOR_MOTOR_RELAY_SWITCH1_PIN           (68U)
#define LIFT_CONVEYOR_MOTOR_RELAY_SWITCH2_PIN           (69U)

/* Motor - PWM */
#define ELEVATE_MOTOR_PWM_PIN                           (5U)
#define ELEVATE_MOTOR_DIR_PIN                           (22U)

/* 
    READ PIN 단자와 물린 COM 단자에는 GND 단자 물려줘야 합니다.
*/
#define LIFT_IR_READ_PIN                                (23U)
#define LIFT_IR_LED_PIN                                 (46U)                     


/* Limited Switch */
/*
    이쪽 COM 단자에는 5V 전압 물려줘야 합니다.
*/
#define FLOOR1_LIMITED_SWITCH_READ_PIN                  (37U)
#define FLOOR2_LIMITED_SWITCH_READ_PIN                  (36U)
#define FLOOR3_LIMITED_SWITCH_READ_PIN                  (35U)
#define FLOOR3_CEILING_LIMITED_SWITCH_READ_PIN          (34U)

#endif /* HARDWARE_H */