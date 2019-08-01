/*
 * RFIDdevicestatus.h
 *
 *  Created on: 31 may. 2019
 *      Author: agarciamoreno
 */

#ifndef SRC_RFID_RFIDDEVICESTATUS_H_
#define SRC_RFID_RFIDDEVICESTATUS_H_
#include <cstdint>
enum class RFID_device_status
		: uint8_t
	{
		online,
	connectionfailed,
	disconnected,
	lastPN532_status = disconnected
};

const char* RFID_reader_status(RFID_device_status name);
#endif /* SRC_RFID_RFIDDEVICESTATUS_H_ */
