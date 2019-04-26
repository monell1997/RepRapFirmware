/*
 * HDC1010Sensor.cpp
 *
 *  Created on: 26 abr. 2019
 *      Author: agarciamoreno
 */

#include "HDC1010Sensor.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "GCodes/GCodes.h"
#ifdef BCN3D_DEV

// Define the minimum interval between readings.
const uint32_t MinimumReadInterval = 2000;		// minimum interval between reads, in milliseconds

void HDC1010Sensor::Init()
{
	InitI2C();
	TemperatureError rslt;
	for (unsigned int i = 0; i < 3; ++i)		// try 3 times
	{
		rslt = TryInitI2C();
		if (rslt == TemperatureError::success)
		{
			break;
		}
		delay(MinimumReadInterval);
	}

	lastReadingTime = millis();
	lastResult = rslt;
	lastTemperature = 0.0;

	if (rslt != TemperatureError::success)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to initialise I2C: %s\n", TemperatureErrorString(rslt));
	}

}
// Configure this temperature sensor
GCodeResult HDC1010Sensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	if (mCode == 305)
	{
		bool seen = false;
		if(gb.Seen('A')){
			seen = true;
			addr = (uint16_t) gb.GetUIValue();
		}
		TryConfigureHeaterName(gb, seen);

		if (!gb.Seen('X'))
		{
			CopyBasicHeaterDetails(heater, reply);
			reply.catf(", Something wrong");
		}
	}
	return GCodeResult::ok;
}
// Try to initialise the I2C
TemperatureError HDC1010Sensor::TryInitI2C() const
{

	static const uint8_t command[1] = {0x50};			// Read Memory from dir 0x50 tem
	const uint16_t addr = 80;//default dir
	uint32_t rawVal;

	const TemperatureError sts = DoI2CTransaction(command, ARRAY_SIZE(command), 2, rawVal, addr);

	if(rawVal > 100 && 0 > rawVal){
		sts = TemperatureError::badResponse;
		return sts;
	}

	static const uint8_t command[1] = {0x52};			// Read Memory from dir 0x52 hum
	const TemperatureError sts = DoI2CTransaction(command, ARRAY_SIZE(command), 2, rawVal, addr);
	if(rawVal > 100 && 0 > rawVal){
		sts = TemperatureError::badResponse;
	}

	return sts;
}
TemperatureError HDC1010Sensor::TryGetTemperature(float& t)
{
	if (inInterrupt() || millis() - lastReadingTime < MinimumReadInterval)
	{
		t = lastTemperature;
	}
	else
	{
		static const uint8_t command[1] = {0x50};			// Read Memory from dir 0x50 tem
		const uint16_t addr = 80;//default dir
		uint32_t rawVal;

		const TemperatureError sts = DoI2CTransaction(command, ARRAY_SIZE(command), 2, rawVal, addr);

		if (sts != TemperatureError::success)
		{
			lastResult = sts;
		}
		else
		{
			lastReadingTime = millis();

			if(rawVal < 100 && 0 < rawVal){
				t = (float) rawVal;
			}else{
				lastResult = TemperatureError::hardwareError;
				delay(1);
				TryInitI2C();
			}

		}
	}
	return lastResult;
}
TemperatureError HDC1010Sensor::TryGetHumidity(float& t)
{
	if (inInterrupt() || millis() - lastReadingTime < MinimumReadInterval)
	{
		t = lastTemperature;
	}
	else
	{
		static const uint8_t command[1] = {0x52};			// Read Memory from dir 0x52 hum
		const uint16_t addr = 80;//default dir
		uint32_t rawVal;

		const TemperatureError sts = DoI2CTransaction(command, ARRAY_SIZE(command), 2, rawVal, addr);

		if (sts != TemperatureError::success)
		{
			lastResult = sts;
		}
		else
		{
			lastReadingTime = millis();

			if(rawVal < 100 && 0 < rawVal){
				t = (float) rawVal;
			}else{
				lastResult = TemperatureError::hardwareError;
				delay(1);
				TryInitI2C();
			}

		}
	}
	return lastResult;
}
#endif
