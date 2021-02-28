#ifndef TIMERINTERRUPT_H
#define TIMERINTERRUPT_H

extern volatile bool isPublishValidate;
extern volatile bool isSubscribeValidate;

void PublishISR();
void SubscribeISR();

#endif /* TIMERINTERRUPT_H */