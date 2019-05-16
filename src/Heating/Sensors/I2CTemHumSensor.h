/*
 * I2CTemHumSensor.h
 *
 *  Created on: 26 abr. 2019
 *      Author: agarciamoreno
 */

#ifndef SRC_HEATING_SENSORS_I2CTEMHUMSENSOR_H_
#define SRC_HEATING_SENSORS_I2CTEMHUMSENSOR_H_
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Wire.h"
#include "Tasks.h"
#include "TemperatureSensor.h"
#ifdef BCN3D_DEV
class I2CTemHumSensor : public TemperatureSensor
{
protected:
	I2CTemHumSensor(unsigned int channel, const char *name);
	void InitI2C();
	void RestartI2C();
	TemperatureError DoI2CTransaction(const uint8_t command[], size_t numToSend, size_t numToReceive, uint32_t& rslt, uint16_t addr) const
		pre(numToSend <= 8);

	//sspi_device device;
	uint32_t lastReadingTime;
	float lastTemperature, lastHumidity;
	TemperatureError lastResult;
private:
	bool i2cInitialised;
};
#endif
#endif /* SRC_HEATING_SENSORS_I2CTEMHUMSENSOR_H_ */
