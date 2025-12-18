#include "config.h"

#include <EEPROM.h>
#include <gravity_tds.h>
#include "config.h"

GravityTDS gravityTds;

float temperature = TDS_TEMPERATURE;
float tdsValue = 0;

void setup()
{
    Serial.begin(115200);
    gravityTds.setPin(TDS_PIN);
    gravityTds.setAref(TDS_AREF);
    gravityTds.setAdcRange(ADC_RANGE);
    gravityTds.begin();
}

void loop()
{
    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate 
    tdsValue = gravityTds.getTdsValue();  // then get the value
    Serial.print(tdsValue,0);
    Serial.println("ppm");
    delay(1000);
}
