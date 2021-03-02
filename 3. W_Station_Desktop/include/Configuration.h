
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

/* Timer Task */
#define PUBLISH_PERIOD_US           (100000U)       /* 10Hz - 100ms */

/* Conveyor */
#define MAX_FLOOR_COUNT             (3U)
#define MAX_IR_SENSOR_COUNT         (4U)

/* Lift */
#define MAX_LEVEL_SWITCH_COUNT      (3U)

/* Floor */
enum class eFloor : uint8_t
{
    None = 0,
    FirstFloor = 1,
    SecondFloor = 2,
    ThirdFloor = 3
};

#endif /* CONFIGURATION_H */