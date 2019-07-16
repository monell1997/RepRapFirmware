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
#include "Hardware/I2C.h"
#include <SpoolSupplier/SpoolSupplier.h>
#ifdef BCN3D_DEV

//#define DEBUG_HDC

// Define the minimum interval between readings.
const uint32_t MinimumReadInterval = 2000;		// minimum interval between reads, in milliseconds
const uint32_t Default_delay = 1;		// minimum interval between request and send, in milliseconds

HDC1010Sensor::HDC1010Sensor(unsigned int addr_offset)
	: I2CTemHumSensor(addr_offset, "HDC1010 Sensor I2C")// addr between 64-67
{
	addr = addr_offset - FirstHDC1010Channel + 64;
}
void HDC1010Sensor::Init()
{
	I2C::Init();
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
			temorhum = true;
		}else{
			temorhum = false;
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

	const uint8_t command_1[3] = {0x02, 0x80, 0x00};			// Configure device reset software
	const uint8_t command_2[3] = {0x02, 0x00, 0x00};			// Configure device normal operation
	//const uint16_t addr = 80;//default dir
	uint32_t rawVal;

	TemperatureError sts = DoI2CTransaction(command_1, ARRAY_SIZE(command_1), 0, rawVal, addr);

	if(sts!=TemperatureError::success) return sts;

	delay(5);

	sts = DoI2CTransaction(command_2, ARRAY_SIZE(command_2), 0, rawVal, addr);

	return sts;
}
TemperatureError HDC1010Sensor::TryGetTemperature(float& t)
{

	if (inInterrupt() || millis()-(250*(addr-64)) - lastReadingTime < MinimumReadInterval)// Reserve time slots for different Sensors i2c
	{
		t = lastTemperature;
	}
	else
	{

		lastReadingTime = millis() + (250*(addr-64));

		static const uint8_t command[1] = {0x01};			// Read  hum
		static const uint8_t command2[1] = {0x00};			// Read  temp


		TemperatureError sts;
		uint32_t rawVal;

		if(temorhum){
		sts = DoI2CTransaction(command, ARRAY_SIZE(command), 0, rawVal, addr); //request data hum
		//delay(2);
		delayMicroseconds(1800);
		}else{
		sts = DoI2CTransaction(command2, ARRAY_SIZE(command2), 0, rawVal, addr); //request data temp
		delay(1);
		}



		if(temorhum){
		sts = DoI2CTransaction(command,0, 2, rawVal, addr);
		}else{
		sts = DoI2CTransaction(command2,0, 2, rawVal, addr);
		}
		if (sts != TemperatureError::success)
		{
			//lastResult = sts;
			static bool fail = false;
			if(!fail){
				fail = true;
				reprap.GetPlatform().MessageF(HttpMessage, "I2c Rx Error\n");
			}
			//
		}
		else
		{

			if(temorhum){
				lastHumidity = ((float) rawVal *100.0 / 65536.0);
				size_t index = 0;
				if(N_Spools <= Maxi2cTempSensors){
					if(N_Spools != Maxi2cTempSensors){
						if((addr-64) == 0){//0
							index = 0;
						}else if(addr-64 == 3){//1
							index = 1;
						}else if(addr-64 == 2){//2
							index = 2;
						}else if(addr-64 == 1){//3
							index = 3;
						}
						reprap.GetSpoolSupplier().Set_Current_Humidity(index,lastHumidity);
					}

				}
#ifdef DEBUG_HDC
				reprap.GetPlatform().MessageF(HttpMessage, "Humidity %.1f\n",(double)lastHumidity);
#endif
			}else{
				lastTemperature = ((float) rawVal *165.0 / 65536.0)-40.0;
#ifdef DEBUG_HDC
				reprap.GetPlatform().MessageF(HttpMessage, "Temp %.1f\n",(double)lastTemperature);
#endif
			}

		}

		if(temorhum)temorhum=false;
		else temorhum = true;


	}
	t = lastTemperature;
	return lastResult;
}

#endif
