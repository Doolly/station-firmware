#include "TimerInterrupt.h"

volatile bool isPublishValidate = false;
volatile bool isSubscribeValidate = false;

void PublishISR()
{
    isPublishValidate = true;
}

void SubscribeISR()
{
    isSubscribeValidate = true;
}