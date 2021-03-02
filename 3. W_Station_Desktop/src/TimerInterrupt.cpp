#include "TimerInterrupt.h"

volatile bool isPublishValidate = false;

void PublishISR()
{
    isPublishValidate = true;
}