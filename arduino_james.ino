/*------ Headers to Include ------*/
#include "barometer.h"

/*------ Global Variables ------*/
Barometer baro;

void setup()
{
  Serial.begin(9600);

  baro.setReference();
  Serial.println("MS5611 Sensor Initialized");

}

void loop()
{
  Serial.print("AbsoluteAltitude : ");
  Serial.println(baro.getAbsoluteAltitude());
  Serial.print("RelativeAltitude : ");
  Serial.println(baro.getRelativeAltitude());
  Serial.print("Temperature : ");
  Serial.println(baro.getTemperature());
  delay(400);
}
