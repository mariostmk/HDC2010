/*
This file is part of the HDC2010 library.
Copyright(c) 2024 SecondOfFive Electronics AB.All rights reserved.
This library is free software; you can redistribute it and /or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU
Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110 - 1301  USA
*/

#include <Arduino.h>
#include <Wire.h>

#define DEVICE_ID_LOW_REG 0xFE
#define DEVICE_ID_HIGH_REG 0xFF
#define EXPECTED_DEVICE_ID_LOW 0xD0
#define EXPECTED_DEVICE_ID_HIGH 0x07

enum HDC2010_MeasurementRate {
    Manual,
    TwoMinutes,
    OneMinute,
    TenSeconds,
    FiveSeconds,
    OneSecond,
    HalfSecond,
    TwoHundredMilliseconds
};

enum HDC2010_TemperatureResolution {
    Temp_14_Bit = 14,
    Temp_11_Bit = 11,
    Temp_9_Bit = 9
};

enum HDC2010_HumidityResolution {
    Humid_14_Bit = 14,
    Humid_11_Bit = 11,
    Humid_9_Bit = 9
};

class HDC2010 {
public:
    HDC2010(uint8_t address);
    bool begin();
    float readHumidity(uint8_t readCount);
    float readTemperature(uint8_t readCount);
    void resetSensor();
    void configureSensor(HDC2010_MeasurementRate measurementRate, HDC2010_TemperatureResolution resolutionTemperature, HDC2010_HumidityResolution resolutionHumidity);

private:
    uint8_t _address;
    uint8_t HDC2010_Config = 0x00;
    uint8_t HDC2010_MeasurementConfig = 0x00;
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void triggerMeasurement();
    float calculateAverage(float* measurements, uint8_t count);
};
