#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>

class IrSensor
{
public:
    IrSensor();
    virtual ~IrSensor() = default;

    void SetReadPin(const int8_t readPin);

    bool GetLight() const;
    void SetLightPin(int8_t lightPin);
    void SetLight(const bool state);

    bool GetState() const;

private:
    int8_t mReadPin;
    int8_t mLightPin;
    bool mIsLightOn;
};

#endif /* IRSENSOR_H */