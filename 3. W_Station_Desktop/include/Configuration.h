
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

/* Timer Task */
#define PUBLISH_PERIOD_US           (100000U)       /* 10Hz - 100ms */

#define MAX_FLOOR_COUNT             (3U)
#define MAX_LEVEL_SWITCH_COUNT      (4U)

#define MAX_BUFFER_SIZE             (20U)

#define MAX_CONVEYOR_STATUS_LIST    (3U)
#define MAX_LIFT_STATUS_LIST        (3U)

/* Floor */
enum class eFloor : uint8_t
{
    FirstFloor,
    SecondFloor,
    ThirdFloor
};

#endif /* CONFIGURATION_H */