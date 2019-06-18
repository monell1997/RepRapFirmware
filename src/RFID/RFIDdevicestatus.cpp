/*
 * RFIDdevicestatus.cpp
 *
 *  Created on: 31 may. 2019
 *      Author: agarciamoreno
 */

#include "RFID/RFIDdevicestatus.h"
const char* RFID_reader_status(RFID_device_status name)
{
	switch(name)
	{
	case RFID_device_status::online:				return "RFID online";
	case RFID_device_status::connectionfailed:		return "Connection failed";
	case RFID_device_status::disconnected:			return "Disconnected";
	default:										return "unknown status";
	}
}

