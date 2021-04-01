#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>

class IrSensor
{
public:
    IrSensor() = delete;
    IrSensor(int8_t readPin, int8_t lightPin);
    virtual ~IrSensor() = default;

    void SetLight(const bool state);

    bool GetState() const;

private:
    int8_t mReadPin;
    int8_t mLightPin;
};

#endif /* IRSENSOR_H */