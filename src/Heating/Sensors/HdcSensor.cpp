/*
 * HDC1011Sensor.cpp
 *
 *  Created on: 16 may. 2019
 *      Author: agarciamoreno
 */


#include <Heating/Sensors/HdcSensor.h>
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"
#include "Hardware/I2C.h"
#ifdef BCN3D_DEV

constexpr uint32_t MinimumReadInterval = 2000;		// ms
constexpr uint32_t MaximumReadTime = 8;			// ms

# include "Tasks.h"

// Static data members of class HdcSensorHardwareInterface
Mutex HdcSensorHardwareInterface::hdcMutex;
//Task<HdcSensorHardwareInterface::HdcTaskStackWords> *HdcSensorHardwareInterface::hdcTask = nullptr;
HdcSensorHardwareInterface *HdcSensorHardwareInterface::activeSensors[Maxi2cTempSensors] = { 0 };

/*extern "C" void HdcTask(void * pvParameters)
{
	HdcSensorHardwareInterface::SensorTask();
}*/

// Pulse ISR
//extern "C" void HdcDataTransition(CallbackParameter cp)
//{
//	static_cast<HdcSensorHardwareInterface*>(cp.vp)->Interrupt();
//}

HdcSensorHardwareInterface::HdcSensorHardwareInterface(uint8_t addr)
	: sensoraddr(addr), type(HdcSensorType::none), lastResult(TemperatureError::notInitialised),
	  lastTemperature(BadErrorTemperature), lastHumidity(BadErrorTemperature), badTemperatureCount(0)
{

}

TemperatureError HdcSensorHardwareInterface::GetTemperatureOrHumidity(float& t, bool wantHumidity) const
{
	t = (wantHumidity) ? lastHumidity : lastTemperature;
	return lastResult;
}

/*static*/ GCodeResult HdcSensorHardwareInterface::Configure(TemperatureSensor *ts, unsigned int relativeChannel, unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	MutexLocker lock(hdcMutex);

	if (relativeChannel >= Maxi2cTempSensors || activeSensors[relativeChannel] == nullptr)
	{
		reply.copy("invalid channel");
		return GCodeResult::error;
	}

	return activeSensors[relativeChannel]->Configure(ts, mCode, heater, gb, reply);
}

GCodeResult HdcSensorHardwareInterface::Configure(TemperatureSensor *ts, unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	GCodeResult rslt = GCodeResult::ok;
	if (mCode == 305)
	{
		bool seen = false;
		ts->TryConfigureHeaterName(gb, seen);

		if (gb.Seen('T'))
		{
			seen = true;

			const int hdcType = gb.GetIValue();
			switch (hdcType)
			{
			case 1010:
				type = HdcSensorType::Hdc1010;
				break;
			default:
				reply.copy("Invalid HCD sensor type");
				rslt = GCodeResult::error;
				break;
			}
		}

		if (!seen && !gb.Seen('X'))
		{
			ts->CopyBasicHeaterDetails(heater, reply);

			const char *sensorTypeString;
			switch (type)
			{
			case HdcSensorType::Hdc1010:
				sensorTypeString = "HDC1010";
				break;
			default:
				sensorTypeString = "unknown";
				break;
			}
			reply.catf(", sensor type %s", sensorTypeString);
		}
	}
	return rslt;
}

// Create a hardware interface object for the specified channel if there isn't already
HdcSensorHardwareInterface *HdcSensorHardwareInterface::Create(unsigned int relativeChannel)
{
	if (relativeChannel >= Maxi2cTempSensors)
	{
		return nullptr;
	}

	MutexLocker lock(hdcMutex);

	if (activeSensors[relativeChannel] == nullptr)
	{
		activeSensors[relativeChannel] = new HdcSensorHardwareInterface((uint8_t)(64+relativeChannel));
	}
/*
	if (hdcTask == nullptr)
	{
		hdcTask = new Task<HdcTaskStackWords>;
		hdcTask->Create(HdcTask, "HDCSENSOR", nullptr, TaskPriority::HdcPriority);
	}*/

	return activeSensors[relativeChannel];
}

/*static*/ void HdcSensorHardwareInterface::InitStatic()
{
	hdcMutex.Create("HDC");
}

/*static*/ TemperatureError HdcSensorHardwareInterface::GetTemperatureOrHumidity(unsigned int relativeChannel, float& t, bool wantHumidity)
{
	if (relativeChannel >= Maxi2cTempSensors)
	{
		t = BadErrorTemperature;
		return TemperatureError::unknownChannel;
	}

	MutexLocker lock(hdcMutex);

	if (activeSensors[relativeChannel] == nullptr)
	{
		t = BadErrorTemperature;
		return TemperatureError::notInitialised;
	}

	return activeSensors[relativeChannel]->GetTemperatureOrHumidity(t, wantHumidity);
}


void HdcSensorHardwareInterface::DoI2CTransaction(const uint8_t command[], size_t numToSend, size_t numToReceive, uint32_t& rslt, uint16_t address) const pre(numToSend <= 8){


	size_t bytesTransferred;
	uint8_t bValues[MaxI2cBytes];
	if (numToSend + numToReceive != 0)
	{
		for (size_t i = 0; i < numToSend; ++i)
		{
			bValues[i] = (uint8_t)command[i];
		}

		I2C::Init();

		bytesTransferred = I2C::Transfer(address, bValues, numToSend, numToReceive);

		if (bytesTransferred < numToSend)
		{
		rslt = 0xffff;
		return;

		}
		else if (numToReceive != 0)
		{

			if (bytesTransferred == numToSend)
			{
				rslt = 0xffff;
				return;
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
		if(bytesTransferred != numToSend + numToReceive){
			rslt = 0xffff;
		}
		return;
	}
	return;


}

void HdcSensorHardwareInterface::TakeReading()
{
	if (type != HdcSensorType::none)			// if sensor has been configured
	{

		//TaskCriticalSectionLocker lock;		// make sure the Heat task doesn't interrupt the sequence

		const uint8_t comand_start[3] = {0x02, 0x00, 0x00};			// Configure device normal operation and acquire in sequence, Temperature First
		const uint8_t comand_temp[1] = {0x00};			// Read Memory from temp
		const uint8_t comand_hum[1] = {0x01};			// Read Memory from hum

		uint32_t rawVal = 33330; // 43.9ºC

		// Sensor Setup
		//TaskCriticalSectionLocker lock;		// make sure the Heat task doesn't interrupt the sequence
		DoI2CTransaction(comand_start, ARRAY_SIZE(comand_start), 0, rawVal, sensoraddr);

		delay(1);
		//Get Tem

		DoI2CTransaction(comand_temp, ARRAY_SIZE(comand_temp), 0, rawVal, sensoraddr); //request data

		delay(10);

		DoI2CTransaction(comand_temp,0, 2, rawVal, sensoraddr);

		recv_temp = (uint16_t)(rawVal);
		//recv_hum = recv_temp;
		/*
		recv_temp = (uint16_t)(rawVal>>16);
		recv_hum = (uint16_t)(rawVal && 0xffff);*/
		delay(1);
		//Get Hum

		DoI2CTransaction(comand_hum, ARRAY_SIZE(comand_hum), 0, rawVal, sensoraddr); //request data

		delay(10);

		DoI2CTransaction(comand_hum,0, 2, rawVal, sensoraddr);

		recv_hum = (uint16_t)rawVal;


		//delay(MaximumReadTime);

		//detachInterrupt(sensorPin);
		//recv_temp = 33333;
		//recv_hum = 32222;

		// Attempt to convert the signal into temp+RH values
		const TemperatureError rslt = ProcessReadings();
		if (rslt == TemperatureError::success)
		{
			lastResult = rslt;
			badTemperatureCount = 0;
		}
		else if (badTemperatureCount < MaxBadTemperatureCount)
		{
			badTemperatureCount++;
		}
		else
		{
			lastResult = rslt;
			lastTemperature = BadErrorTemperature;
			lastHumidity = BadErrorTemperature;
		}
	}
}

// Code executed by the HDC sensor task.
// This is run at the same priority as the Heat task, so it must not sit in any spin loops.
/*static*/ /*void HdcSensorHardwareInterface::SensorTask()
{
	for (;;)
	{
		for (HdcSensorHardwareInterface *&sensor : activeSensors)
		{
			{
				MutexLocker lock(hdcMutex);

				if (sensor != nullptr)
				{
					sensor->TakeReading();
				}
			}
			delay(MinimumReadInterval/Maxi2cTempSensors);
		}
	}
}*/

/*static*/ void HdcSensorHardwareInterface::Spin()
{

	const uint32_t now = millis();
	if (now - lastTime >= MinimumReadInterval)
	{
		for (HdcSensorHardwareInterface *&sensor : activeSensors)
		{
			{
				MutexLocker lock(hdcMutex);

				if (sensor != nullptr)
				{
					sensor->TakeReading();
				}
			}
		}
		lastTime = millis();
	}

}

// Process a reading. If success then update the temperature and humidity and return TemperatureError::success.
// Else return the TemperatureError code but do not update the readings.
TemperatureError HdcSensorHardwareInterface::ProcessReadings()
{

	// Generate final results
	//reprap.GetPlatform().MessageF(HttpMessage, "Get Info\n");
	if(recv_temp < 100 || recv_temp > 65436)
		return TemperatureError::hardwareError;
	if(recv_hum < 100 || recv_hum > 65436)
			return TemperatureError::hardwareError;

	switch (type)
	{
	case HdcSensorType::Hdc1010:
		lastTemperature = (recv_temp * 165.0 / 65536.0 )-40.0;
		lastHumidity = (recv_hum) * 100.0 / 65536.0;
		return TemperatureError::success;

	default:
		return TemperatureError::notInitialised;
	}
}

// Class HdcTemperatureSensor members
HdcTemperatureSensor::HdcTemperatureSensor(unsigned int channel)
	: TemperatureSensor(channel, "HDC-temperature")
{

}

void HdcTemperatureSensor::Init()
{
	HdcSensorHardwareInterface::Create(GetSensorChannel() - FirstHDC1011TempChannel);
}

GCodeResult HdcTemperatureSensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	return HdcSensorHardwareInterface::Configure(this, GetSensorChannel() - FirstHDC1011TempChannel, mCode, heater, gb, reply);
}

HdcTemperatureSensor::~HdcTemperatureSensor()
{
	// We don't delete the hardware interface object because the humidity channel may still be using it
}

TemperatureError HdcTemperatureSensor::TryGetTemperature(float& t)
{
	//reprap.GetPlatform().MessageF(HttpMessage, "Get Info\n");
	return HdcSensorHardwareInterface::GetTemperatureOrHumidity(GetSensorChannel() - FirstHDC1011TempChannel, t, false);
}

// Class HdcHumiditySensor members
HdcHumiditySensor::HdcHumiditySensor(unsigned int channel)
	: TemperatureSensor(channel, "HDC-humidity")
{
}

void HdcHumiditySensor::Init()
{
	HdcSensorHardwareInterface::Create(GetSensorChannel() - FirstHDC1011HumChannel);
}

GCodeResult HdcHumiditySensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	return HdcSensorHardwareInterface::Configure(this, GetSensorChannel() - FirstHDC1011HumChannel, mCode, heater, gb, reply);
}

HdcHumiditySensor::~HdcHumiditySensor()
{
	// We don't delete the hardware interface object because the temperature channel may still be using it
}

TemperatureError HdcHumiditySensor::TryGetTemperature(float& t)
{
	return HdcSensorHardwareInterface::GetTemperatureOrHumidity(GetSensorChannel() - FirstHDC1011HumChannel, t, true);
}

#endif

// End
