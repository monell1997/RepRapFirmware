/*
 * I2CTemHumSensor.cpp
 *
 *  Created on: 26 abr. 2019
 *      Author: agarciamoreno
 */

#include "I2CTemHumSensor.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Tasks.h"
#include "Wire.h"
#ifdef BCN3D_DEV
I2CTemHumSensor::I2CTemHumSensor(unsigned int channel, const char *name)
	: TemperatureSensor(channel, name)
{

	lastTemperature = 0.0;
	lastResult = TemperatureError::notInitialised;
}
void I2CTemHumSensor::InitI2C(){
	// Initialise the I2C interface, if not already done

	#if defined(I2C_IFACE)
		if (!i2cInitialised)
		{
			MutexLocker lock(Tasks::GetI2CMutex());
			if (!i2cInitialised)			// test it again, now that we own the mutex
			{
				I2C_IFACE.BeginMaster(I2cClockFreq);
				i2cInitialised = true;
			}
		}
	#endif

}
TemperatureError I2CTemHumSensor::DoI2CTransaction(const uint8_t command[], size_t numToSend, size_t numToReceive, uint32_t& rslt, uint16_t addr) const pre(numToSend <= 8){


	size_t bytesTransferred;
	uint8_t bValues[MaxI2cBytes];
	if (numToSend + numToReceive != 0)
	{
		for (size_t i = 0; i < numToSend; ++i)
		{
			bValues[i] = (uint8_t)command[i];
		}

		MutexLocker lock(Tasks::GetI2CMutex());
		if (!lock)
		{
			return TemperatureError::busBusy;
		}
		bytesTransferred = I2C_IFACE.Transfer(addr, bValues, numToSend, numToReceive);


		if (bytesTransferred < numToSend)
		{

		return TemperatureError::hardwareError;
		}
		else if (numToReceive != 0)
		{

			if (bytesTransferred == numToSend)
			{
				return TemperatureError::hardwareError;
			}
			else
			{
				rslt = bValues[numToSend];
				for (size_t i = numToSend + 1; i < bytesTransferred; ++i)
				{
					rslt <<= 8;
					rslt |= bValues[i];
				}
			}
		}
		return (bytesTransferred == numToSend + numToReceive) ? TemperatureError::success : TemperatureError::badResponse;
	}
	return TemperatureError::badResponse;

}
#endif
