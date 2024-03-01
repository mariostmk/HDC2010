#include <Arduino.h>
#include <Wire.h>
#include <HDC2010.h>

HDC2010 hdc2010Sensor = HDC2010(0x40);

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  if(hdc2010Sensor.begin())
  {
    Serial.println("HDC2010 Sensor found!");
    hdc2010Sensor.resetSensor();
    hdc2010Sensor.configureSensor(HDC2010_MeasurementRate::Manual, HDC2010_TemperatureResolution::Temp_14_Bit, HDC2010_HumidityResolution::Humid_14_Bit);
  }
  else
  {
    Serial.println("HDC2010 Sensor not found!");
    while(1)
    {
      delay(2000);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  float temperature = hdc2010Sensor.readTemperature(5);

  float humidity = hdc2010Sensor.readHumidity(5);

  Serial.print("Temperature:");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print("Humidity:");
  Serial.println(humidity);

  delay(1000);
}
