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

#include "HDC2010.h"


HDC2010::HDC2010(uint8_t address) : _address(address) {}

bool HDC2010::begin() {
    Wire.begin();
    
	uint8_t deviceIdLow = readRegister(DEVICE_ID_LOW_REG);
	uint8_t deviceIdHigh = readRegister(DEVICE_ID_HIGH_REG);

	if (deviceIdLow == EXPECTED_DEVICE_ID_LOW && deviceIdHigh == EXPECTED_DEVICE_ID_HIGH) {
		// Sensor is connected and responding
		return true;
	}
	
	// Sensor not found or error in communication
	return false;
}

void HDC2010::resetSensor() {
    uint8_t resetRead = readRegister(0x0E);
    resetRead |= 0b10000000; // Set Reset Bit
    writeRegister(0x0E, resetRead);
    delay(10); // Wait for the reset to complete
}

void HDC2010::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t HDC2010::readRegister(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)_address, 1);
    return Wire.read();
}

void HDC2010::configureSensor(HDC2010_MeasurementRate measurementRate, HDC2010_TemperatureResolution resolutionTemperature, HDC2010_HumidityResolution resolutionHumidity) {
	HDC2010_Config = readRegister(0x0E);
	HDC2010_MeasurementConfig = readRegister(0x0F);

	switch (measurementRate) {

	case Manual:
		HDC2010_Config = (HDC2010_Config & 0x8F);
		break;

	case TwoMinutes:
		HDC2010_Config = (HDC2010_Config & 0x9F);
		HDC2010_Config = (HDC2010_Config | 0x10);
		break;

	case OneMinute:
		HDC2010_Config = (HDC2010_Config & 0xAF);
		HDC2010_Config = (HDC2010_Config | 0x20);
		break;

	case TenSeconds:
		HDC2010_Config = (HDC2010_Config & 0xBF);
		HDC2010_Config = (HDC2010_Config | 0x30);
		break;

	case FiveSeconds:
		HDC2010_Config = (HDC2010_Config & 0xCF);
		HDC2010_Config = (HDC2010_Config | 0x40);
		break;

	case OneSecond:
		HDC2010_Config = (HDC2010_Config & 0xDF);
		HDC2010_Config = (HDC2010_Config | 0x50);
		break;

	case HalfSecond:
		HDC2010_Config = (HDC2010_Config & 0xEF);
		HDC2010_Config = (HDC2010_Config | 0x60);
		break;

	case TwoHundredMilliseconds:
		HDC2010_Config = (HDC2010_Config | 0x70);
		break;

	default:
		HDC2010_Config = (HDC2010_Config & 0x8F); // Manual Mode

	}

	// Set Temperature Resolution
	switch (resolutionTemperature) {

	case Temp_14_Bit:	// 14 Bit
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig & 0x3F);
		break;

	case Temp_11_Bit:	// 11 Bit
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig & 0x7F);
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig | 0x40);
		break;

	case Temp_9_Bit:		// 9 Bit
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig & 0xBF);
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig | 0x80);
		break;

	default:
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig & 0x3F); //Default 14 Bit

	}

	// Set Humidity Resolution
	switch (resolutionHumidity) {

	case Humid_14_Bit:	// 14 Bit
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig & 0xCF);
		break;

	case Humid_11_Bit:	// 11 Bit
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig & 0xDF);
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig | 0x10);
		break;

	case Humid_9_Bit:		// 9 Bit
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig & 0xEF);
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig | 0x20);
		break;

	default:
		HDC2010_MeasurementConfig = (HDC2010_MeasurementConfig & 0xCF); //Default 14 Bit

	}

	// Write Register
	writeRegister(0x0E, HDC2010_Config);

	// Write Register
	writeRegister(0x0F, HDC2010_MeasurementConfig);
}

float HDC2010::readHumidity(uint8_t readCount) {
    float measurements[readCount];
    for (uint8_t i = 0; i < readCount; ++i) {
		uint8_t HDC2010_Data[2];

		triggerMeasurement();
		delay(5);

		HDC2010_Data[0] = readRegister(0x02);
		HDC2010_Data[1] = readRegister(0x03);

		uint16_t Measurement_Raw = ((uint16_t)(HDC2010_Data[1]) << 8 | (uint16_t)HDC2010_Data[0]);

		measurements[i] = (float)Measurement_Raw / 65536 * 100;
    }
    return calculateAverage(measurements, readCount);
}

float HDC2010::readTemperature(uint8_t readCount) {
	float measurements[readCount];
	for (uint8_t i = 0; i < readCount; ++i) {
		uint8_t HDC2010_Data[2];
		triggerMeasurement();
		delay(5);

		HDC2010_Data[0] = readRegister(0x00);
		HDC2010_Data[1] = readRegister(0x01);

		uint16_t Measurement_Raw = ((uint16_t)(HDC2010_Data[1]) << 8 | (uint16_t)HDC2010_Data[0]);

		measurements[i] = (float)Measurement_Raw * 165 / 65536 - 40;
	}
	return calculateAverage(measurements, readCount);
}

float HDC2010::calculateAverage(float* measurements, uint8_t count) {
	if (count == 0) 
		return 0; // Return 0 if no measurements

	float sum = 0;
	for (uint8_t i = 0; i < count; ++i) {
		sum += measurements[i];
	}
	return sum / count;
}

void HDC2010::triggerMeasurement() {
	HDC2010_MeasurementConfig |= 0x1; // Set the bit to start measurement
	writeRegister(0x0F, HDC2010_MeasurementConfig);

}