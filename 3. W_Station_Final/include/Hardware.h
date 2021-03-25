#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

/* Pin Configuration */
#define DEBUG_LED1_PIN                                  (39U)
#define DEBUG_LED2_PIN                                  (40U)
#define DEBUG_LED3_PIN                                  (41U)
#define DEBUG_LED4_PIN                                  (42U)

/* Roll shutter */
// 정 68, 역 69

/* Conveyor Motor - Relay */
#define FLOOR1_MOTOR_RELAY_SWITCH1_PIN                  (5U)  
#define FLOOR2_MOTOR_RELAY_SWITCH1_PIN                  (66U) 
#define FLOOR3_MOTOR_RELAY_SWITCH1_PIN                  (67U) 

#define LIFT_CONVEYOR_MOTOR_RELAY_SWITCH1_PIN           (62U) //62 정
#define LIFT_CONVEYOR_MOTOR_RELAY_SWITCH2_PIN           (63U) //63 역

/* Elevate Motor - PWM */
#define ELEVATE_MOTOR_PWM_PIN                           (2U)  // P2
#define ELEVATE_MOTOR_BREAK_PIN                         (A6) 
#define ELEVATE_MOTOR_CW_PIN                            (11U) // P11
#define ELEVATE_MOTOR_CCW_PIN                           (12U) // P12

/* IR */
#define LIFT_IR_READ_PIN                                (37U) 
#define LIFT_IR_LED_PIN                                 (64U) 

//#define MAIN_IR_LED_PIN                                 (2U) //2 오디세이로
/* 원격 킬 output pin*/
// 오디세이로 3, 4

/* IR Sensor -> COM = GND */
#define FLOOR1_FIRST_IR_READ_PIN                        (22U)
#define FLOOR1_SECOND_IR_READ_PIN                       (23U)
#define FLOOR1_THIRD_IR_READ_PIN                        (24U)
#define FLOOR1_FOURTH_IR_READ_PIN                       (25U)
#define FLOOR1_FIFTH_IR_READ_PIN                        (26U)
#define FLOOR2_FIRST_IR_READ_PIN                        (27U)
#define FLOOR2_SECOND_IR_READ_PIN                       (28U)
#define FLOOR2_THIRD_IR_READ_PIN                        (29U)
#define FLOOR2_FOURTH_IR_READ_PIN                       (30U)
#define FLOOR2_FIFTH_IR_READ_PIN                        (31U)
#define FLOOR3_FIRST_IR_READ_PIN                        (32U)
#define FLOOR3_SECOND_IR_READ_PIN                       (33U)
#define FLOOR3_THIRD_IR_READ_PIN                        (34U)
#define FLOOR3_FOURTH_IR_READ_PIN                       (35U)
#define FLOOR3_FIFTH_IR_READ_PIN                        (36U)

/* Limited Switch */
#define FLOOR1_LIMITED_SWITCH_READ_PIN                  (A0) 
#define FLOOR2_LIMITED_SWITCH_READ_PIN                  (A1)
#define FLOOR3_LIMITED_SWITCH_READ_PIN                  (A2)
#define FLOOR3_CEILING_LIMITED_SWITCH_READ_PIN          (A3)

#endif /* HARDWARE_H */