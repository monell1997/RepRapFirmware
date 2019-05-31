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
	failedconnection,
	disconnected,
	lastPN532_status = disconnected
};

#endif /* SRC_RFID_RFIDDEVICESTATUS_H_ */
