/*
 * TagReaderWriter.cpp
 *
 *  Created on: 29 abr. 2019
 *      Author: agarciamoreno
 */
#include "RFID/SpoolRDIFReader.h"
#include "RFID/PN532Handler.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Tasks.h"
#include "Wire.h"
#include "SharedSpi.h"

#ifdef BCN3D_DEV

// Static data
Mutex PN532Handler::PN532HandlerMutex;

uint8_t pn532ack[] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };
uint8_t pn532response_firmwarevers[] = { 0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03 };

// Uncomment these lines to enable debug output for PN532(SPI) and/or MIFARE related code
// #define PN532DEBUG
// #define MIFAREDEBUG

// If using Native Port on Arduino Zero or Due define as SerialUSB
//#define PN532DEBUGPRINT Serial
//#define PN532DEBUGPRINT SerialUSB

#define PN532_PACKBUFFSIZ 64
uint8_t pn532_packetbuffer[PN532_PACKBUFFSIZ];

#ifndef _BV
#define _BV(bit) (1<<(bit))
#endif

const uint32_t PN532_Frequency = 400000;	// maximum for PN532 is also 400kHz

const uint8_t PN532_SpiMode = SPI_MODE_0;

/**************************************************************************/
/*!
 @brief  Instantiates a new PN532 class using hardware SPI.

 @param  ss        SPI chip select pin (CS/SSEL)
 */
/**************************************************************************/
PN532Handler::PN532Handler(uint8_t spool)
	{
	device.csPin = SpiTempSensorCsPins[spool];	// CS1 up to CS4
	device.csPolarity = false;						// active low chip select
	device.spiMode = PN532_SpiMode;
	device.clockFrequency = PN532_Frequency;

	_PN532_status = RFID_device_status::disconnected;
	_RW_State = RW_State::none;

	timeoutWR = 0;
	lastTime = 0;
	spool_index	= spool;
	PN532HandlerMutex.Create("TagReaderWriter");
}

/**************************************************************************/
/*!
 @brief  Setups the HW
 */
/**************************************************************************/
void PN532Handler::Init() {

	sspi_master_init(&device, 8);

	sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

	delay(2);

	// not exactly sure why but we have to send a dummy command to get synced up
	pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
	if(sendCommandCheckAck(pn532_packetbuffer, 1)){
		_PN532_status = RFID_device_status::online;
	}else{
		_PN532_status = RFID_device_status::connectionfailed;
	}

	// ignore response!

	delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);

	SAMConfig();

	_RW_State = RW_State::none;
	lastTime = millis();
}

/**************************************************************************/
/*!
 @brief  Prints a hexadecimal value in plain characters

 @param  data      Pointer to the uint8_t data
 @param  numuint8_ts  Data length in uint8_ts
 */
/**************************************************************************/
void PN532Handler::PrintHex(const uint8_t * data,
		const uint32_t numBytes) {
	uint32_t szPos;
	for (szPos = 0; szPos < numBytes; szPos++) {
		reprap.GetPlatform().MessageF(HttpMessage, "0x");
		// Append leading 0 for small values

		reprap.GetPlatform().MessageF(HttpMessage, "%02x",
				data[szPos] & 0xff);

		if ((numBytes > 1) && (szPos != numBytes - 1)) {
			reprap.GetPlatform().MessageF(HttpMessage, " ");
		}
	}
	reprap.GetPlatform().MessageF(HttpMessage, "\n");
}

/**************************************************************************/
/*!
 @brief  Prints a hexadecimal value in plain characters, along with
 the char equivalents in the following format

 00 00 00 00 00 00  ......

 @param  data      Pointer to the uint8_t data
 @param  numuint8_ts  Data length in uint8_ts
 */
/**************************************************************************/
void PN532Handler::PrintHexChar(const uint8_t * data,
		const uint32_t numuint8_ts) {
	uint32_t szPos;
	for (szPos = 0; szPos < numuint8_ts; szPos++) {
		// Append leading 0 for small values

		reprap.GetPlatform().MessageF(HttpMessage, "0x");
		reprap.GetPlatform().MessageF(HttpMessage, "%02x", data[szPos]);

		if ((numuint8_ts > 1) && (szPos != numuint8_ts - 1)) {
			reprap.GetPlatform().MessageF(HttpMessage, " ");
		}
	}
	reprap.GetPlatform().MessageF(HttpMessage, "  ");
	for (szPos = 0; szPos < numuint8_ts; szPos++) {
		if (data[szPos] <= 0x1F)
			reprap.GetPlatform().MessageF(HttpMessage, ".");
		else
			reprap.GetPlatform().MessageF(HttpMessage, "%c",
					(char) data[szPos]);
	}
	reprap.GetPlatform().MessageF(HttpMessage, "\n");
}

/**************************************************************************/
/*!
 @brief  Checks the firmware version of the PN5xx chip

 @returns  The chip's firmware version and ID
 */
/**************************************************************************/
uint32_t PN532Handler::getFirmwareVersion(void) {
	uint32_t response;

	pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

	if (!sendCommandCheckAck(pn532_packetbuffer, 1)) {
		return 0;
	}

	// read data packet
	readdata(pn532_packetbuffer, 12);

	// check some basic stuff
	if (0
			!= strncmp((char *) pn532_packetbuffer,
					(char *) pn532response_firmwarevers, 6)) {
#ifdef PN532DEBUG
		reprap.GetPlatform().MessageF(HttpMessage, "Firmware doesn't match!\n");
#endif
		return 0;
	}

	int offset = 6;
	response = pn532_packetbuffer[offset++];
	response <<= 8;
	response |= pn532_packetbuffer[offset++];
	response <<= 8;
	response |= pn532_packetbuffer[offset++];
	response <<= 8;
	response |= pn532_packetbuffer[offset++];

	return response;
}

/**************************************************************************/
/*!
 @brief  Sends a command and waits a specified period for the ACK

 @param  cmd       Pointer to the command buffer
 @param  cmdlen    The size of the command in uint8_ts
 @param  timeout   timeout before giving up

 @returns  1 if everything is OK, 0 if timeout occured before an
 ACK was recieved
 */
/**************************************************************************/
// default timeout of one second
bool PN532Handler::sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen,
		uint16_t timeout) {

	// write the command
	writecommand(cmd, cmdlen);

	// Wait for chip to say its ready!
	if (!waitready(timeout)) {
		return false;
	}


	// read acknowledgement
	if (!readack()) {
#ifdef PN532DEBUG
		reprap.GetPlatform().MessageF(HttpMessage, "No ACK frame received!\n");
#endif
		return false;
	}

	// For SPI only wait for the chip to be ready again.

	if (!waitready(timeout)) {
		return false;
	}

	return true; // ack'd command
}


/**************************************************************************/
/*!
 @brief  Configures the SAM (Secure Access Module)
 */
/**************************************************************************/
bool PN532Handler::SAMConfig(void) {
	pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
	pn532_packetbuffer[1] = 0x01; // normal mode;
	pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
	pn532_packetbuffer[3] = 0x01; // use IRQ pin!

	if (!sendCommandCheckAck(pn532_packetbuffer, 4))
		return false;

	// read data packet
	readdata(pn532_packetbuffer, 8);

	int offset = 5;
	return (pn532_packetbuffer[offset] == 0x15);
}

/**************************************************************************/
/*!
 Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register

 @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
 after mxRetries

 @returns 1 if everything executed properly, 0 for an error
 */
/**************************************************************************/
bool PN532Handler::setPassiveActivationRetries(uint8_t maxRetries) {
	pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
	pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
	pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
	pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
	pn532_packetbuffer[4] = maxRetries;

#ifdef MIFAREDEBUG
	reprap.GetPlatform().MessageF(HttpMessage, "Setting MxRtyPassiveActivation to "); reprap.GetPlatform().MessageF(HttpMessage, "%u", maxRetries); reprap.GetPlatform().MessageF(HttpMessage, " \n");
#endif

	if (!sendCommandCheckAck(pn532_packetbuffer, 5))
		return 0x0;  // no ACK

	return 1;
}


/***** Mifare Ultralight Functions ******/

/**************************************************************************/
/*!
 Tries to read an entire 4-uint8_t page at the specified address.

 @param  page        The page number (0..63 in most cases)
 @param  buffer      Pointer to the uint8_t array that will hold the
 retrieved data (if any)
 */
/**************************************************************************/
uint8_t PN532Handler::mifareultralight_ReadPage(uint8_t page,
		uint8_t * buffer) {
	if (page >= 64) {
#ifdef MIFAREDEBUG
		reprap.GetPlatform().MessageF(HttpMessage, "Page value out of range\n");
#endif
		return 0;
	}

#ifdef MIFAREDEBUG
	reprap.GetPlatform().MessageF(HttpMessage, "Reading page ");reprap.GetPlatform().MessageF(HttpMessage, "%u\n",page);
#endif

	/* Prepare the command */
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1; /* Card number */
	pn532_packetbuffer[2] = MIFARE_CMD_READ; /* Mifare Read command = 0x30 */
	pn532_packetbuffer[3] = page; /* Page Number (0..63 in most cases) */

	/* Send the command */
	if (!sendCommandCheckAck(pn532_packetbuffer, 4)) {
#ifdef MIFAREDEBUG
		reprap.GetPlatform().MessageF(HttpMessage, "Failed to receive ACK for write command\n");
#endif
		return 0;
	}

	/* Read the response packet */
	readdata(pn532_packetbuffer, 26);
#ifdef MIFAREDEBUG
	reprap.GetPlatform().MessageF(HttpMessage, "Received: \n");
	PN532Handler::PrintHexChar(pn532_packetbuffer, 26);
#endif

	/* If uint8_t 8 isn't 0x00 we probably have an error */
	if (pn532_packetbuffer[7] == 0x00) {
		/* Copy the 4 data uint8_ts to the output buffer         */
		/* Block content starts at uint8_t 9 of a valid response */
		/* Note that the command actually reads 16 uint8_t or 4  */
		/* pages at a time ... we simply discard the last 12  */
		/* uint8_ts                                              */
		memcpy(buffer, pn532_packetbuffer + 8, 4);
	} else {
#ifdef MIFAREDEBUG
		reprap.GetPlatform().MessageF(HttpMessage, "Unexpected response reading block: \n");
		PN532Handler::PrintHexChar(pn532_packetbuffer, 26);
#endif
		return 0;
	}

	/* Display data for debug if requested */
#ifdef MIFAREDEBUG
	reprap.GetPlatform().MessageF(HttpMessage, "Page ");reprap.GetPlatform().MessageF(HttpMessage, "%u",page);reprap.GetPlatform().MessageF(HttpMessage, ":\n");
	PN532Handler::PrintHexChar(buffer, 4);
#endif

	// Return OK signal
	return 1;
}

/**************************************************************************/
/*!
 Tries to write an entire 4-uint8_t page at the specified block
 address.

 @param  page          The page number to write.  (0..63 for most cases)
 @param  data          The uint8_t array that contains the data to write.
 Should be exactly 4 uint8_ts long.

 @returns 1 if everything executed properly, 0 for an error
 */
/**************************************************************************/
uint8_t PN532Handler::mifareultralight_WritePage(uint8_t page,
		uint8_t * data) {

	if (page >= 64) {
#ifdef MIFAREDEBUG
		reprap.GetPlatform().MessageF(HttpMessage, "Page value out of range\n");
#endif
		// Return Failed Signal
		return 0;
	}

#ifdef MIFAREDEBUG
	reprap.GetPlatform().MessageF(HttpMessage, "Trying to write 4 uint8_t page");reprap.GetPlatform().MessageF(HttpMessage, "%u\n",page);
#endif

	/* Prepare the first command */
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1; /* Card number */
	pn532_packetbuffer[2] = MIFARE_ULTRALIGHT_CMD_WRITE; /* Mifare Ultralight Write command = 0xA2 */
	pn532_packetbuffer[3] = page; /* Page Number (0..63 for most cases) */
	memcpy(pn532_packetbuffer + 4, data, 4); /* Data Payload */

	/* Send the command */
	if (!sendCommandCheckAck(pn532_packetbuffer, 8)) {
#ifdef MIFAREDEBUG
		reprap.GetPlatform().MessageF(HttpMessage, "Failed to receive ACK for write command\n");
#endif

		// Return Failed Signal
		return 0;
	}
	delay(10);

	/* Read the response packet */
	readdata(pn532_packetbuffer, 26);

	// Return OK Signal
	return 1;
}

/**************************************************************************/
/*!
 @brief  Tries to read the SPI or I2C ACK signal
 */
/**************************************************************************/
bool PN532Handler::readack() {
	uint8_t ackbuff[6];

	readdata(ackbuff, 6);

	return (0 == strncmp((char *) ackbuff, (char *) pn532ack, 6));
}

/**************************************************************************/
/*!
 @brief  Return true if the PN532 is ready with a response.
 */
/**************************************************************************/
bool PN532Handler::isready() {

	uint8_t dataOut[1] = { 0 };
	uint8_t rawBytes[1] = { 0 };

	MutexLocker lock(Tasks::GetSpiMutex(), 10);
	if (!lock) {
		reprap.GetPlatform().MessageF(HttpMessage, " !lock\n");
		return 0;
	}
	sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

	//delay(2);

	dataOut[0] = data_lsbfirst(PN532_SPI_STATREAD);
	sspi_write_packet(dataOut, 1);

	sspi_read_packet(rawBytes, 1);

	delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);

	// read uint8_t
	uint8_t x = data_lsbfirst(rawBytes[0]);
	//reprap.GetPlatform().MessageF(HttpMessage, "0x%02x ", x);
	// Check if status is ready.
	return x == PN532_SPI_READY;
}

/**************************************************************************/
/*!
 @brief  Waits until the PN532 is ready.

 @param  timeout   Timeout before giving up
 */
/**************************************************************************/
bool PN532Handler::waitready(uint16_t timeout) {
	uint16_t timer = 0;
	while (!isready()) {
		if (timeout != 0) {
			timer += 10;
			if (timer > timeout) {
				reprap.GetPlatform().MessageF(HttpMessage, "TIMEOUT!\n");
				return false;
			}
		}
		delay(10);
	}
	return true;
}
/**************************************************************************/
/*!
 @brief  Waits until the PN532 is ready.

 @param  timeout   Timeout before giving up
 */
/**************************************************************************/
bool PN532Handler::waitready2(uint16_t timeout) {
	uint16_t timer = 0;
	while (!isready()) {
		if (timeout != 0) {
			timer += 10;
			if (timer > timeout) {
				reprap.GetPlatform().MessageF(HttpMessage, "TIMEOUT!\n");
				return false;
			}
		}
		delay(10);
	}
	return true;
}

/**************************************************************************/
/*!
 @brief  Reads n uint8_ts of data from the PN532 via SPI or I2C.

 @param  buff      Pointer to the buffer where data will be written
 @param  n         Number of uint8_ts to be read
 */
/**************************************************************************/
void PN532Handler::readdata(uint8_t* buff, uint8_t n) {

	// SPI write.
	uint8_t dataOut[1] = { 0 };
	uint8_t rawBytes[1] = { 0 };

	MutexLocker lock(Tasks::GetSpiMutex(), 10);
	if (!lock) {
		reprap.GetPlatform().MessageF(HttpMessage, " !lock\n");
		return;
	}

	sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

	//delay(2);

	dataOut[0] = data_lsbfirst(PN532_SPI_DATAREAD);
	sspi_write_packet(dataOut, 1);

#ifdef PN532DEBUG
	reprap.GetPlatform().MessageF(HttpMessage, "Reading: ");
#endif

	for (uint8_t i = 0; i < n; i++) {
		delayMicroseconds(1);
		sspi_read_packet(rawBytes, 1);
		buff[i] = data_lsbfirst(rawBytes[0]);
#ifdef PN532DEBUG
		reprap.GetPlatform().MessageF(HttpMessage, " 0x");
		reprap.GetPlatform().MessageF(HttpMessage, "%02x",buff[i]);
#endif
	}

#ifdef PN532DEBUG
	reprap.GetPlatform().MessageF(HttpMessage, "\n");
#endif

	delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);
}

/**************************************************************************/
/*!
 @brief  Writes a command to the PN532, automatically inserting the
 preamble and required frame details (checksum, len, etc.)

 @param  cmd       Pointer to the command buffer
 @param  cmdlen    Command length in uint8_ts
 */
/**************************************************************************/
void PN532Handler::writecommand(uint8_t* cmd, uint8_t cmdlen) {

	// SPI command write
	uint8_t dataOut_1[1] = { 0 };
	uint8_t checksum;
	cmdlen++;

#ifdef PN532DEBUG
	//reprap.GetPlatform().MessageF(HttpMessage, "\nSending: ");
#endif

	MutexLocker lock(Tasks::GetSpiMutex(), 10);
	if (!lock) {
		reprap.GetPlatform().MessageF(HttpMessage, " !lock\n");
		return;
	}

	sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

	//delay(2);     // or whatever the delay is for waking up the board

	dataOut_1[0] = data_lsbfirst(PN532_SPI_DATAWRITE);
	sspi_write_packet(dataOut_1, 1);
	delayMicroseconds(1);

	dataOut_1[0] = data_lsbfirst(PN532_PREAMBLE);
	sspi_write_packet(dataOut_1, 1);
	delayMicroseconds(1);

	dataOut_1[0] = data_lsbfirst(PN532_PREAMBLE);
	sspi_write_packet(dataOut_1, 1);
	delayMicroseconds(1);

	dataOut_1[0] = data_lsbfirst(PN532_STARTCODE2);
	sspi_write_packet(dataOut_1, 1);
	delayMicroseconds(1);

	dataOut_1[0] = data_lsbfirst(cmdlen);
	sspi_write_packet(dataOut_1, 1);
	delayMicroseconds(1);

	dataOut_1[0] = data_lsbfirst((uint8_t) (~cmdlen + 1));
	sspi_write_packet(dataOut_1, 1);
	delayMicroseconds(1);

	dataOut_1[0] = data_lsbfirst(PN532_HOSTTOPN532);
	sspi_write_packet(dataOut_1, 1);
	delayMicroseconds(1);

	checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
	checksum += PN532_HOSTTOPN532;

#ifdef PN532DEBUG
	reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)PN532_PREAMBLE);
	reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)PN532_PREAMBLE);
	reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)PN532_STARTCODE2);
	reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)cmdlen);
	reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)(~cmdlen + 1));
	reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)PN532_HOSTTOPN532);
#endif

	for (uint8_t i = 0; i < cmdlen - 1; i++) {

		dataOut_1[0] = data_lsbfirst(cmd[i]);
		sspi_write_packet(dataOut_1, 1);
		delayMicroseconds(1);
		checksum += cmd[i];
#ifdef PN532DEBUG
		reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)cmd[i]);
#endif
	}

	dataOut_1[0] = data_lsbfirst((~checksum));
	sspi_write_packet(dataOut_1, 1);
	delayMicroseconds(1);

	dataOut_1[0] = data_lsbfirst(PN532_POSTAMBLE);
	sspi_write_packet(dataOut_1, 1);

	delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);

#ifdef PN532DEBUG
	reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)~checksum);
	reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x\n",(uint8_t)PN532_POSTAMBLE);

#endif

}
/************** low level SPI */

// bit shifting
uint8_t PN532Handler::data_lsbfirst(uint8_t b) {

	uint8_t byte = 0x00;

	for (uint8_t i = 0; i < 8; i++) {
		if ((b & (1 << i))) {
			byte |= 1 << (7 - i);
		}

	}
	//reprap.GetPlatform().MessageF(HttpMessage, " 0x"); 	reprap.GetPlatform().MessageF(HttpMessage, "%02x",(uint8_t)byte);
	return byte;
}

/*!
  @brief - ProcessStates aims to process the PN532 request without getting stuck inside. The idea is
  process each state and the wait periods being running the main loop to avoid timeout in
  another process of the firmware.

  @variables - _RW_State is the current request state

  @extra - Request pulling of 1s

  By Alejandro
*/
void PN532Handler::ProcessStates() {

	const uint32_t now = millis();
	static uint16_t loop_state = LOOP_PROCESS_READ_NONE;
	uint8_t datapage[32] = {0};
	//uint8_t success; // Flag to check if there was an error with the PN532
#ifdef MIFAREDEBUG
	uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 }; // Buffer to store the returned UID
#endif
	static uint8_t uidLength=0; // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
	//uint8_t currentblock; // Counter to keep track of which block we're on
	//bool authenticated = false; // Flag to indicate if the sector is authenticated
	//uint8_t data[16];          // Array to store block data during reads

	// Keyb on NDEF and Mifare Classic should be the same
	//uint8_t keyuniversal[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	// Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
	// 'uid' will be populated with the UID, and uidLength will indicate
	// if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)



	switch(_RW_State){////////// 1º establish Connection with card - 2º Read Data from card (depents type) - 3º Write something(not implemented yet)

	case RW_State::none:
		if (now - lastTime >= 1000) {
			lastTime = millis();
			_RW_State = RW_State::writecommand;
			loop_state = LOOP_PROCESS_GETUID;
		}
		break;
	case RW_State::writecommand://Request data
		switch(loop_state){
			case LOOP_PROCESS_GETUID:
				{
					pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
					pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
					pn532_packetbuffer[2] = PN532_MIFARE_ISO14443A;
					writecommand(pn532_packetbuffer, 3);
					#ifdef MIFAREDEBUG
					reprap.GetPlatform().MessageF(HttpMessage, "Get ID Card %d\n", (int)device.csPin);
					#endif
				}
				break;
			case LOOP_PROCESS_READ_PAGE1:
			case LOOP_PROCESS_READ_PAGE2:
				{
					if (LOOP_PROCESS_READ_PAGE2 >= 64) {
				#ifdef MIFAREDEBUG
						reprap.GetPlatform().MessageF(HttpMessage, "Page value out of range\n");
				#endif
						RESET_LOOP;
					}

				#ifdef MIFAREDEBUG
					reprap.GetPlatform().MessageF(HttpMessage, "Reading page ");reprap.GetPlatform().MessageF(HttpMessage, "%u\n",loop_state);
				#endif

					/* Prepare the command */
					pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
					pn532_packetbuffer[1] = 1; /* Card number */
					pn532_packetbuffer[2] = MIFARE_CMD_READ; /* Mifare Read command = 0x30 */
					pn532_packetbuffer[3] = loop_state; /* Page Number (0..63 in most cases) */
					writecommand(pn532_packetbuffer, 4);
				}
				break;
		}
		_RW_State = RW_State::waitready;
		timeoutCount = 0;
		timeoutWR = 100;
		break;

	case RW_State::waitready://wait for next read
		{

			if (now - lastTime >= 10) {
				lastTime = millis();
				if(isready()){
					_RW_State = RW_State::readack;
				}else{
					if(timeoutWR > timeoutCount){
						timeoutCount += 10;
					}else{

						RESET_LOOP;
						//Timeout Message
					}
				}
			}
		}
		break;
	case RW_State::readack://wait until ack is received

		if (!readack()) {
			//No Ack received
			RESET_LOOP;
		}else{
			_RW_State = RW_State::waitready2;
			timeoutCount = 0;
			timeoutWR = 100;
		}
		break;
	case RW_State::waitready2://wait for next read
		{
			if (now - lastTime >= 10) {
				lastTime = millis();
				if(isready()){
					_RW_State = RW_State::readdata;
				}else{
					if(timeoutWR > timeoutCount){
						timeoutCount += 10;
					}else{
						RESET_LOOP;
					}
				}
			}
		}
		break;
	case RW_State::readdata://Read data ---> 1º Read Card UID, 2º Read 4bytes for spool material ID, 3º Read 1byte for remaining filament
		switch(loop_state){
		case LOOP_PROCESS_GETUID:
			{
				// read data packet
				readdata(pn532_packetbuffer, 20);
				// check some basic stuff

				/* ISO14443A card response should be in the following format:

				 uint8_t            Description
				 -------------   ------------------------------------------
				 b0..6           Frame header and preamble
				 b7              Tags Found
				 b8              Tag Number (only one used in this example)
				 b9..10          SENS_RES
				 b11             SEL_RES
				 b12             NFCID Length
				 b13..NFCIDLen   NFCID                                      */

			#ifdef MIFAREDEBUG
				reprap.GetPlatform().MessageF(HttpMessage, "Found "); reprap.GetPlatform().MessageF(HttpMessage, "%u", pn532_packetbuffer[7]); reprap.GetPlatform().MessageF(HttpMessage, " tags\n");
			#endif
				if (pn532_packetbuffer[7] != 1){
					RESET_LOOP;
				}


				uint16_t sens_res = pn532_packetbuffer[9];
				sens_res <<= 8;
				sens_res |= pn532_packetbuffer[10];
			#ifdef MIFAREDEBUG
				reprap.GetPlatform().MessageF(HttpMessage, "ATQA: 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x\n",sens_res);
				reprap.GetPlatform().MessageF(HttpMessage, "SAK: 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x\n",pn532_packetbuffer[11]);
			#endif

				/* Card appears to be Mifare Classic */
				uidLength = pn532_packetbuffer[12];
			#ifdef MIFAREDEBUG
				reprap.GetPlatform().MessageF(HttpMessage, "UID:");
			#endif
				for (uint8_t i = 0; i < pn532_packetbuffer[12]; i++) {
					_uid[i] = pn532_packetbuffer[13 + i];
			#ifdef MIFAREDEBUG
					reprap.GetPlatform().MessageF(HttpMessage, " 0x"); reprap.GetPlatform().MessageF(HttpMessage, "%02x",_uid[i]);
			#endif
				}
			#ifdef MIFAREDEBUG
				reprap.GetPlatform().MessageF(HttpMessage, "\n");
			#endif

/*
				// Display some basic information about the card

				reprap.GetPlatform().MessageF(HttpMessage,
						"Found an ISO14443A card\n");
				reprap.GetPlatform().MessageF(HttpMessage, "  UID Length: %d",
						uidLength);
				reprap.GetPlatform().MessageF(HttpMessage, " bytes\n");
				reprap.GetPlatform().MessageF(HttpMessage, "  UID Value: ");
				PrintHex(uid, uidLength);
				reprap.GetPlatform().MessageF(HttpMessage, "\n");
*/
				_RW_State = RW_State::writecommand;
				loop_state = LOOP_PROCESS_READ_PAGE1;


			}
			break;
		case LOOP_PROCESS_READ_PAGE1:
		case LOOP_PROCESS_READ_PAGE2:
			{
				if(uidLength == 4){
					#ifdef MIFAREDEBUG
					reprap.GetPlatform().MessageF(HttpMessage, "Only Mifare Ultralight cards \n");
					#endif
					RESET_LOOP;

				}else if(uidLength == 7){ // Mifare Ultralight
					/* Read the response packet */
						readdata(pn532_packetbuffer, 26);
					#ifdef MIFAREDEBUG
						reprap.GetPlatform().MessageF(HttpMessage, "Received: \n");
						PN532Handler::PrintHex(pn532_packetbuffer, 26);
					#endif

						/* If uint8_t 8 isn't 0x00 we probably have an error */
						if (pn532_packetbuffer[7] == 0x00) {
							/* Copy the 4 data uint8_ts to the output buffer         */
							/* Block content starts at uint8_t 9 of a valid response */
							/* Note that the command actually reads 16 uint8_t or 4  */
							/* pages at a time ... we simply discard the last 12  */
							/* uint8_ts                                              */
							memcpy(datapage, pn532_packetbuffer + 8, 4);
						} else {
					#ifdef MIFAREDEBUG
							reprap.GetPlatform().MessageF(HttpMessage, "Unexpected response reading block: \n");
							PN532Handler::PrintHex(pn532_packetbuffer, 26);
					#endif
							RESET_LOOP;
						}

						/* Display data for debug if requested */
					#ifdef MIFAREDEBUG
						reprap.GetPlatform().MessageF(HttpMessage, "Page ");reprap.GetPlatform().MessageF(HttpMessage, "%u",loop_state);reprap.GetPlatform().MessageF(HttpMessage, ":\n");
						PN532Handler::PrintHex(datapage, 4);
					#endif
				}
				if(loop_state == LOOP_PROCESS_READ_PAGE1){
					reprap.GetSpoolSupplier().Set_Spool_id(spool_index,datapage,4);
					_RW_State = RW_State::writecommand;
					loop_state = LOOP_PROCESS_READ_PAGE2;
				}else if(loop_state == LOOP_PROCESS_READ_PAGE2){
					reprap.GetSpoolSupplier().Set_Spool_Remaining(spool_index,datapage,1);
					_RW_State = RW_State::none;
					loop_state = LOOP_PROCESS_READ_NONE;
				}
			}
			break;

		}

		lastTime = millis();
		break;

	}

}



RFID_device_status PN532Handler::Get_PN532_Status(){
	return _PN532_status;
}

void PN532Handler::Spin() {
	MutexLocker lock(PN532HandlerMutex);

	if (_PN532_status == RFID_device_status::online) {

		ProcessStates();

	}

}
#endif
