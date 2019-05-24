/*
 * HDC1011Sensor.h
 *
 *  Created on: 16 may. 2019
 *      Author: agarciamoreno
 */

#ifndef SRC_HEATING_SENSORS_HDCSENSOR_H_
#define SRC_HEATING_SENSORS_HDCSENSOR_H_


#include "RepRapFirmware.h"
//#include "I2CTemHumSensor.h"
#ifdef BCN3D_DEV

#ifndef RTOS
# error HDC sensors are only supported in RTOS builds
#endif

# include "TemperatureSensor.h"
# include "RTOSIface/RTOSIface.h"

enum class HdcSensorType
{
	none,
	Hdc1010
};

// This class represents a HDC sensor attached to a particular SPI CS pin
class HdcSensorHardwareInterface
{
public:

	static GCodeResult Configure(TemperatureSensor *ts, unsigned int relativeChannel, unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply);
	void Interrupt();

	static HdcSensorHardwareInterface *Create(unsigned int relativeChannel);
	static TemperatureError GetTemperatureOrHumidity(unsigned int relativeChannel, float& t, bool wantHumidity);
	static void InitStatic();
	static void SensorTask();

private:
	HdcSensorHardwareInterface(uint8_t addr);

	GCodeResult Configure(TemperatureSensor *ts, unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply);
	TemperatureError GetTemperatureOrHumidity(float& t, bool wantHumidity) const;
	void DoI2CTransaction(const uint8_t command[], size_t numToSend, size_t numToReceive, uint32_t& rslt, uint16_t addr) const
			pre(numToSend <= 8);
	void TakeReading();
	TemperatureError ProcessReadings();

	static constexpr uint32_t HdcTaskStackWords = 100;		// task stack size in dwords. 80 was not enough. Use 300 if debugging is enabled.
	static Mutex hdcMutex;
	static Task<HdcTaskStackWords> *hdcTask;
	static HdcSensorHardwareInterface *activeSensors[Maxi2cTempSensors];

	uint8_t sensoraddr;
	HdcSensorType type;
	TemperatureError lastResult;
	float lastTemperature, lastHumidity;
	size_t badTemperatureCount;

	volatile uint16_t lastPulseTime;
	volatile size_t numPulses;
	uint16_t pulses[41];			// 1 start bit + 40 data bits
	uint16_t recv_temp;			// Recv temp
	uint16_t recv_hum;			// Recv Hum
};

// This class represents a HDC temperature sensor
class HdcTemperatureSensor : public TemperatureSensor
{
public:
	HdcTemperatureSensor(unsigned int channel);
	~HdcTemperatureSensor();

	GCodeResult Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) override;
	void Init() override;

protected:
	TemperatureError TryGetTemperature(float& t) override;
};

// This class represents a HDC humidity sensor
class HdcHumiditySensor : public TemperatureSensor
{
public:
	HdcHumiditySensor(unsigned int channel);
	~HdcHumiditySensor();

	GCodeResult Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) override;
	void Init() override;

protected:
	TemperatureError TryGetTemperature(float& t) override;
};

#endif

#endif /* SRC_HEATING_SENSORS_HDCSENSOR_H_ */
