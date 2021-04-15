#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>

class IrSensor
{
public:
    IrSensor() = delete;
    IrSensor(uint8_t readPin, uint8_t lightPin);
    virtual ~IrSensor() = default;

    //void SetReadPin(const uint8_t readPin);
    //void SetLightPin(uint8_t lightPin);

    bool GetLight() const;
    void SetLight(const bool state);

    bool GetState() const;

private:
    uint8_t mReadPin;
    uint8_t mLightPin;
    bool mbLightOn;
};

#endif /* IRSENSOR_H */