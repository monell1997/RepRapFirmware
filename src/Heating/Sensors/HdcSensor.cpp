/*
 * HDC1011Sensor.cpp
 *
 *  Created on: 16 may. 2019
 *      Author: agarciamoreno
 */


#include <Heating/Sensors/HdcSensor.h>
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"

#ifdef BCN3D_DEV

constexpr uint32_t MinimumReadInterval = 2000;		// ms
constexpr uint32_t MaximumReadTime = 20;			// ms
constexpr uint32_t MinimumOneBitLength = 50;		// microseconds

# include "Tasks.h"

// Static data members of class HdcSensorHardwareInterface
Mutex HdcSensorHardwareInterface::hdcMutex;
Task<HdcSensorHardwareInterface::HdcTaskStackWords> *HdcSensorHardwareInterface::hdcTask = nullptr;
HdcSensorHardwareInterface *HdcSensorHardwareInterface::activeSensors[Maxi2cTempSensors] = { 0 };

extern "C" void HdcTask(void * pvParameters)
{
	HdcSensorHardwareInterface::SensorTask();
}

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

	if (hdcTask == nullptr)
	{
		hdcTask = new Task<HdcTaskStackWords>;
		hdcTask->Create(HdcTask, "HDCSENSOR", nullptr, TaskBase::HeatPriority);
	}

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

		MutexLocker lock(Tasks::GetI2CMutex());
		if (!lock)
		{
			rslt = 0xFFFF;
			return;
		}
		reprap.GetPlatform().InitI2c();
		bytesTransferred = I2C_IFACE.Transfer(address, bValues, numToSend, numToReceive);
		/*reprap.GetPlatform().MessageF(GenericMessage, "address I2C: %d\n", int(address));
		reprap.GetPlatform().MessageF(GenericMessage, "bytesTransferred I2C: %d\n", int(bytesTransferred));
		reprap.GetPlatform().MessageF(GenericMessage, "numToSend I2C: %d\n", int(numToSend));
		reprap.GetPlatform().MessageF(GenericMessage, "numToReceive I2C: %d\n", int(numToReceive));*/
		if (bytesTransferred < numToSend)
		{
		rslt = 0xFFFF;
		return;

		}
		else if (numToReceive != 0)
		{

			if (bytesTransferred == numToSend)
			{
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
			rslt = 0xFFFF;
		}
		return ;
	}
	return;


}

void HdcSensorHardwareInterface::TakeReading()
{
	if (type != HdcSensorType::none)			// if sensor has been configured
	{

		TaskCriticalSectionLocker lock;		// make sure the Heat task doesn't interrupt the sequence
		delay(1);

		//const uint8_t command_1[3] = {0x02, 0x80, 0x00};			// Configure device reset software
		const uint8_t command_2[3] = {0x02, 0x00, 0x00};			// Configure device normal operation
		//const uint16_t addr = 80;//default dir
		uint32_t rawVal;

		DoI2CTransaction(command_2, ARRAY_SIZE(command_2), 0, rawVal, sensoraddr);
		delay(5);

		static const uint8_t command[1] = {0x01};			// Read Memory from dir 0x52 hum
		static const uint8_t command2[1] = {0x00};			// Read Memory from dir 0x52 hum



		//reprap.GetPlatform().MessageF(GenericMessage, "%u millis(): %lu \n",addr, millis());

		//Get Tem

		DoI2CTransaction(command, ARRAY_SIZE(command), 0, rawVal, sensoraddr); //request data

		delay(MaximumReadTime);

		DoI2CTransaction(command,0, 2, rawVal, sensoraddr);

		recv_temp = (uint16_t)rawVal;

		delay(1);

		//Get Hum

		DoI2CTransaction(command2, ARRAY_SIZE(command2), 0, rawVal, sensoraddr); //request data

		delay(MaximumReadTime);

		DoI2CTransaction(command2,0, 2, rawVal, sensoraddr);

		recv_hum = (uint16_t)rawVal;


		delay(MaximumReadTime);

		//detachInterrupt(sensorPin);

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
/*static*/ void HdcSensorHardwareInterface::SensorTask()
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
}

// Process a reading. If success then update the temperature and humidity and return TemperatureError::success.
// Else return the TemperatureError code but do not update the readings.
TemperatureError HdcSensorHardwareInterface::ProcessReadings()
{

	// Generate final results
	reprap.GetPlatform().MessageF(HttpMessage, "Get Info\n");
	switch (type)
	{
	case HdcSensorType::Hdc1010:

		lastHumidity = (recv_hum) * 100.0 / 65536.0;
		lastTemperature = (recv_temp * 165.0 / 65536.0 )-40.0;
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
