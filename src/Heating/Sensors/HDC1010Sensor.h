/*
 * HDC1010Sensor.h
 *
 *  Created on: 26 abr. 2019
 *      Author: agarciamoreno
 */

#ifndef SRC_HEATING_SENSORS_HDC1010SENSOR_H_
#define SRC_HEATING_SENSORS_HDC1010SENSOR_H_

#include "I2CTemHumSensor.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"

#ifdef BCN3D_DEV
class HDC1010Sensor : public I2CTemHumSensor
{
public:
	HDC1010Sensor(unsigned int channel);
	GCodeResult Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) override;
	void Init() override;

protected:
	TemperatureError TryGetTemperature(float& t) override;
	TemperatureError TryGetHumidity(float& t);

private:
	TemperatureError TryInitI2C() const;
	uint16_t addr;				// i2c address
};
#endif
#endif /* SRC_HEATING_SENSORS_HDC1010SENSOR_H_ */
