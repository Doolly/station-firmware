#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

/* Pin Configuration */
#define DEBUG_LED1_PIN                       (13U)
#define DEBUG_LED2_PIN                       (12U)
#define DEBUG_LED3_PIN                       (11U)
#define DEBUG_LED4_PIN                       (10U)

/* Motor */
#define FLOOR1_MOTOR_RELAY_SWITCH1_PIN       (53)
#define FLOOR1_MOTOR_RELAY_SWITCH2_PIN       (52)
#define FLOOR2_MOTOR_RELAY_SWITCH1_PIN       (51)
#define FLOOR2_MOTOR_RELAY_SWITCH2_PIN       (50)
#define FLOOR3_MOTOR_RELAY_SWITCH1_PIN       (49)
#define FLOOR3_MOTOR_RELAY_SWITCH2_PIN       (48)

#define ELEVATE_MOTOR_RELAY_SWITCH1_PIN      (47)    
#define ELEVATE_MOTOR_RELAY_SWITCH2_PIN      (46)
#define LIFT_CONVEYOR_MOTOR_RELAY_SWITCH1_PIN         (45)
#define LIFT_CONVEYOR_MOTOR_RELAY_SWITCH2_PIN         (44)

/* IR Sensor */
#define FLOOR1_FIRST_IR_READ_PIN             (43U)
#define FLOOR1_SECOND_IR_READ_PIN            (42U)
#define FLOOR1_THIRD_IR_READ_PIN             (41U)
#define FLOOR1_FOURTH_IR_READ_PIN            (40U)

#define FLOOR2_FIRST_IR_READ_PIN             (39U)
#define FLOOR2_SECOND_IR_READ_PIN            (38U)
#define FLOOR2_THIRD_IR_READ_PIN             (37U)
#define FLOOR2_FOURTH_IR_READ_PIN            (36U)

#define FLOOR3_FIRST_IR_READ_PIN             (35U)
#define FLOOR3_SECOND_IR_READ_PIN            (34U)
#define FLOOR3_THIRD_IR_READ_PIN             (33U)
#define FLOOR3_FOURTH_IR_READ_PIN            (32U)

#define LIFT_IR_READ_PIN                     (31U)
#define LIFT_IR_LED_PIN                      (30U)

/* Limited Switch */
#define FLOOR1_LIMITED_SWITCH_READ_PIN       (29U)
#define FLOOR2_LIMITED_SWITCH_READ_PIN       (28U)
#define FLOOR3_LIMITED_SWITCH_READ_PIN       (27U)

#endif /* HARDWARE_H */