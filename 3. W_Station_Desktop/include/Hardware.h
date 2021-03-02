#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

/* Pin Configuration */
#define DEBUG_LED1_PIN                                  (39U)
#define DEBUG_LED2_PIN                                  (40U)
#define DEBUG_LED3_PIN                                  (41U)
#define DEBUG_LED4_PIN                                  (42U)

/* Motor */
#define FLOOR1_MOTOR_RELAY_SWITCH1_PIN                  (62U)
#define FLOOR1_MOTOR_RELAY_SWITCH2_PIN                  (63U)
#define FLOOR2_MOTOR_RELAY_SWITCH1_PIN                  (64U)
#define FLOOR2_MOTOR_RELAY_SWITCH2_PIN                  (65U)
#define FLOOR3_MOTOR_RELAY_SWITCH1_PIN                  (66U)
#define FLOOR3_MOTOR_RELAY_SWITCH2_PIN                  (67U)

#define ELEVATE_MOTOR_RELAY_SWITCH1_PIN                 (68U)    
#define ELEVATE_MOTOR_RELAY_SWITCH2_PIN                 (69U)

#define LIFT_CONVEYOR_MOTOR_RELAY_SWITCH1_PIN           (62U)
#define LIFT_CONVEYOR_MOTOR_RELAY_SWITCH2_PIN           (63U)

/* IR Sensor */
/*
    Digital INPUT 쪽에 COM0, COM1 에 5V 전압 물려줘야 합니다.
*/
#define FLOOR1_FIRST_IR_READ_PIN                        (22U)
#define FLOOR1_SECOND_IR_READ_PIN                       (23U)
#define FLOOR1_THIRD_IR_READ_PIN                        (24U)
#define FLOOR1_FOURTH_IR_READ_PIN                       (25U)

#define FLOOR2_FIRST_IR_READ_PIN                        (26U)
#define FLOOR2_SECOND_IR_READ_PIN                       (27U)
#define FLOOR2_THIRD_IR_READ_PIN                        (28U)
#define FLOOR2_FOURTH_IR_READ_PIN                       (29U)

#define FLOOR3_FIRST_IR_READ_PIN                        (30U)
#define FLOOR3_SECOND_IR_READ_PIN                       (31U)
#define FLOOR3_THIRD_IR_READ_PIN                        (32U)
#define FLOOR3_FOURTH_IR_READ_PIN                       (33U)

#define LIFT_IR_READ_PIN                                (34U)
#define LIFT_IR_LED_PIN                                 (43U)

/* Limited Switch */
/*
    Digital INPUT 쪽에 COM0, COM1 에 5V 전압 물려줘야 합니다.
*/
#define FLOOR1_LIMITED_SWITCH_READ_PIN                  (35U)
#define FLOOR2_LIMITED_SWITCH_READ_PIN                  (36U)
#define FLOOR3_LIMITED_SWITCH_READ_PIN                  (37U)

#endif /* HARDWARE_H */