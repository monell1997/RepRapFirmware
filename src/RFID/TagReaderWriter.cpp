/*
 * TagReaderWriter.cpp
 *
 *  Created on: 29 abr. 2019
 *      Author: agarciamoreno
 */
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Tasks.h"
#include "Wire.h"
#include <RFID/TagReaderWriter.h>
#include "SharedSpi.h"

uint8_t pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
uint8_t pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

// Uncomment these lines to enable debug output for PN532(SPI) and/or MIFARE related code
 #define PN532DEBUG
 #define MIFAREDEBUG

// If using Native Port on Arduino Zero or Due define as SerialUSB
#define PN532DEBUGPRINT Serial
//#define PN532DEBUGPRINT SerialUSB


#define PN532_PACKBUFFSIZ 64
uint8_t pn532_packetbuffer[PN532_PACKBUFFSIZ];

#ifndef _BV
    #define _BV(bit) (1<<(bit))
#endif




/**************************************************************************/
/*!
    @brief  Instantiates a new PN532 class using hardware SPI.

    @param  ss        SPI chip select pin (CS/SSEL)
*/
/**************************************************************************/
TagReaderWriter::TagReaderWriter(uint8_t ss, uint8_t spiMode, uint32_t clockFrequency):
  _usingSPI(true),
  _hardwareSPI(true)
{
	device.csPin = SpiTempSensorCsPins[ss];// CS1 up to CS4
	device.csPolarity = false;						// active low chip select
	device.spiMode = spiMode;
	device.clockFrequency = clockFrequency;
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
void TagReaderWriter::begin() {

    sspi_master_init(&device, 8);


    sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

    delay(1000);

    // not exactly sure why but we have to send a dummy command to get synced up
    pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
    sendCommandCheckAck(pn532_packetbuffer, 1);

    // ignore response!

    delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);


}

/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters

    @param  data      Pointer to the uint8_t data
    @param  numuint8_ts  Data length in uint8_ts
*/
/**************************************************************************/
void TagReaderWriter::PrintHex(const uint8_t * data, const uint32_t numuint8_ts)
{
  uint32_t szPos;
  for (szPos=0; szPos < numuint8_ts; szPos++)
  {
	reprap.GetPlatform().MessageF(GenericMessage, "0x");
    // Append leading 0 for small values
    if (data[szPos] <= 0xF){
    	reprap.GetPlatform().MessageF(GenericMessage, "0");
    	reprap.GetPlatform().MessageF(GenericMessage, "%02x", data[szPos]&0xff);
    }
    if ((numuint8_ts > 1) && (szPos != numuint8_ts - 1))
    {
    	reprap.GetPlatform().MessageF(GenericMessage, " ");
    }
  }
  reprap.GetPlatform().MessageF(GenericMessage, "\n");
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
void TagReaderWriter::PrintHexChar(const uint8_t * data, const uint32_t numuint8_ts)
{
  uint32_t szPos;
  for (szPos=0; szPos < numuint8_ts; szPos++)
  {
    // Append leading 0 for small values
    if (data[szPos] <= 0xF){
    	reprap.GetPlatform().MessageF(GenericMessage, "0");
    	reprap.GetPlatform().MessageF(GenericMessage, "%02x", data[szPos]);
    }
    if ((numuint8_ts > 1) && (szPos != numuint8_ts - 1))
    {
    	reprap.GetPlatform().MessageF(GenericMessage, " ");
    }
  }
  reprap.GetPlatform().MessageF(GenericMessage, "  ");
  for (szPos=0; szPos < numuint8_ts; szPos++)
  {
    if (data[szPos] <= 0x1F)
    	reprap.GetPlatform().MessageF(GenericMessage, ".");
    else
    	reprap.GetPlatform().MessageF(GenericMessage, "%c", (char)data[szPos]);
  }
  reprap.GetPlatform().MessageF(GenericMessage, "\n");
}

/**************************************************************************/
/*!
    @brief  Checks the firmware version of the PN5xx chip

    @returns  The chip's firmware version and ID
*/
/**************************************************************************/
uint32_t TagReaderWriter::getFirmwareVersion(void) {
  uint32_t response;

  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

  if (! sendCommandCheckAck(pn532_packetbuffer, 1)) {
    return 0;
  }

  // read data packet
  readdata(pn532_packetbuffer, 12);

  // check some basic stuff
  if (0 != strncmp((char *)pn532_packetbuffer, (char *)pn532response_firmwarevers, 6)) {
#ifdef PN532DEBUG
      reprap.GetPlatform().MessageF(GenericMessage, "Firmware doesn't match!\n");
#endif
    return 0;
  }

  int offset = _usingSPI ? 6 : 7;  // Skip a response uint8_t when using I2C to ignore extra data.
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
bool TagReaderWriter::sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) {

  // write the command
  writecommand(cmd, cmdlen);

  // Wait for chip to say its ready!
  if (!waitready(timeout)) {
    return false;
  }

  #ifdef PN532DEBUG
    if (!_usingSPI) {
    	reprap.GetPlatform().MessageF(GenericMessage, "IRQ received\n");
    }
  #endif

  // read acknowledgement
  if (!readack()) {
    #ifdef PN532DEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "No ACK frame received!\n");
    #endif
    return false;
  }

  // For SPI only wait for the chip to be ready again.
  // This is unnecessary with I2C.
  if (_usingSPI) {
    if (!waitready(timeout)) {
      return false;
    }
  }

  return true; // ack'd command
}

/**************************************************************************/
/*!
    Writes an 8-bit value that sets the state of the PN532's GPIO pins

    @warning This function is provided exclusively for board testing and
             is dangerous since it will throw an error if any pin other
             than the ones marked "Can be used as GPIO" are modified!  All
             pins that can not be used as GPIO should ALWAYS be left high
             (value = 1) or the system will become unstable and a HW reset
             will be required to recover the PN532.

             pinState[0]  = P30     Can be used as GPIO
             pinState[1]  = P31     Can be used as GPIO
             pinState[2]  = P32     *** RESERVED (Must be 1!) ***
             pinState[3]  = P33     Can be used as GPIO
             pinState[4]  = P34     *** RESERVED (Must be 1!) ***
             pinState[5]  = P35     Can be used as GPIO

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool TagReaderWriter::writeGPIO(uint8_t pinstate) {

  // Make sure pinstate does not try to toggle P32 or P34
  pinstate |= (1 << PN532_GPIO_P32) | (1 << PN532_GPIO_P34);

  // Fill command buffer
  pn532_packetbuffer[0] = PN532_COMMAND_WRITEGPIO;
  pn532_packetbuffer[1] = PN532_GPIO_VALIDATIONBIT | pinstate;  // P3 Pins
  pn532_packetbuffer[2] = 0x00;    // P7 GPIO Pins (not used ... taken by SPI)

  #ifdef PN532DEBUG
	 reprap.GetPlatform().MessageF(GenericMessage, "Writing P3 GPIO: ");
	 reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",pn532_packetbuffer[1]);
  #endif

  // Send the WRITEGPIO command (0x0E)
  if (! sendCommandCheckAck(pn532_packetbuffer, 3))
    return 0x0;

  // Read response packet (00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0F) DATACHECKSUM 00)
  readdata(pn532_packetbuffer, 8);

  #ifdef PN532DEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Received: ");
    PrintHex(pn532_packetbuffer, 8);
    reprap.GetPlatform().MessageF(GenericMessage, "\n");
  #endif

  int offset = _usingSPI ? 5 : 6;
  return  (pn532_packetbuffer[offset] == 0x0F);
}

/**************************************************************************/
/*!
    Reads the state of the PN532's GPIO pins

    @returns An 8-bit value containing the pin state where:

             pinState[0]  = P30
             pinState[1]  = P31
             pinState[2]  = P32
             pinState[3]  = P33
             pinState[4]  = P34
             pinState[5]  = P35
*/
/**************************************************************************/
uint8_t TagReaderWriter::readGPIO(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_READGPIO;

  // Send the READGPIO command (0x0C)
  if (! sendCommandCheckAck(pn532_packetbuffer, 1))
    return 0x0;

  // Read response packet (00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0D) P3 P7 IO1 DATACHECKSUM 00)
  readdata(pn532_packetbuffer, 11);

  /* READGPIO response should be in the following format:

    uint8_t            Description
    -------------   ------------------------------------------
    b0..5           Frame header and preamble (with I2C there is an extra 0x00)
    b6              P3 GPIO Pins
    b7              P7 GPIO Pins (not used ... taken by SPI)
    b8              Interface Mode Pins (not used ... bus select pins)
    b9..10          checksum */

  int p3offset = _usingSPI ? 6 : 7;

  #ifdef PN532DEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Received: ");
    PrintHex(pn532_packetbuffer, 11);
    reprap.GetPlatform().MessageF(GenericMessage, "\n");
    reprap.GetPlatform().MessageF(GenericMessage, "P3 GPIO: 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",pn532_packetbuffer[p3offset]);
    reprap.GetPlatform().MessageF(GenericMessage, "P7 GPIO: 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",pn532_packetbuffer[p3offset+1]);
    reprap.GetPlatform().MessageF(GenericMessage, "IO GPIO: 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",pn532_packetbuffer[p3offset+2]);
    // Note: You can use the IO GPIO value to detect the serial bus being used
    switch(pn532_packetbuffer[p3offset+2])
    {
      case 0x00:    // Using UART
    	  reprap.GetPlatform().MessageF(GenericMessage, "Using UART (IO = 0x00)");
        break;
      case 0x01:    // Using I2C
    	  reprap.GetPlatform().MessageF(GenericMessage, "Using I2C (IO = 0x01)");
        break;
      case 0x02:    // Using SPI
    	  reprap.GetPlatform().MessageF(GenericMessage, "Using SPI (IO = 0x02)");
        break;
    }
  #endif

  return pn532_packetbuffer[p3offset];
}

/**************************************************************************/
/*!
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
bool TagReaderWriter::SAMConfig(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (! sendCommandCheckAck(pn532_packetbuffer, 4))
    return false;

  // read data packet
  readdata(pn532_packetbuffer, 8);

  int offset = _usingSPI ? 5 : 6;
  return  (pn532_packetbuffer[offset] == 0x15);
}

/**************************************************************************/
/*!
    Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register

    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool TagReaderWriter::setPassiveActivationRetries(uint8_t maxRetries) {
  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
  pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
  pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
  pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
  pn532_packetbuffer[4] = maxRetries;

  #ifdef MIFAREDEBUG
  reprap.GetPlatform().MessageF(GenericMessage, "Setting MxRtyPassiveActivation to "); reprap.GetPlatform().MessageF(GenericMessage, "%u", maxRetries); reprap.GetPlatform().MessageF(GenericMessage, " \n");
  #endif

  if (! sendCommandCheckAck(pn532_packetbuffer, 5))
    return 0x0;  // no ACK

  return 1;
}

/***** ISO14443A Commands ******/

/**************************************************************************/
/*!
    Waits for an ISO14443A target to enter the field

    @param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 uint8_ts)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool TagReaderWriter::readPassiveTargetID(uint8_t cardbaudrate, uint8_t * uid, uint8_t * uidLength, uint16_t timeout) {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate;

  if (!sendCommandCheckAck(pn532_packetbuffer, 3, timeout))
  {
    #ifdef PN532DEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "No card(s) read");
    #endif
    return 0x0;  // no cards read
  }

  // wait for a card to enter the field (only possible with I2C)
  if (!_usingSPI) {
    #ifdef PN532DEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Waiting for IRQ (indicates card presence)");
    #endif
    if (!waitready(timeout)) {
      #ifdef PN532DEBUG
    	reprap.GetPlatform().MessageF(GenericMessage, "IRQ Timeout");
      #endif
      return 0x0;
    }
  }

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
    reprap.GetPlatform().MessageF(GenericMessage, "Found "); reprap.GetPlatform().MessageF(GenericMessage, "%u", pn532_packetbuffer[7]); reprap.GetPlatform().MessageF(GenericMessage, " tags\n");
  #endif
  if (pn532_packetbuffer[7] != 1)
    return 0;

  uint16_t sens_res = pn532_packetbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_packetbuffer[10];
  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "ATQA: 0x");  reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",sens_res);
    reprap.GetPlatform().MessageF(GenericMessage, "SAK: 0x");  reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",pn532_packetbuffer[11]);
  #endif

  /* Card appears to be Mifare Classic */
  *uidLength = pn532_packetbuffer[12];
  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "UID:");
  #endif
  for (uint8_t i=0; i < pn532_packetbuffer[12]; i++)
  {
    uid[i] = pn532_packetbuffer[13+i];
    #ifdef MIFAREDEBUG
      reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",uid[i]);
    #endif
  }
  #ifdef MIFAREDEBUG
  reprap.GetPlatform().MessageF(GenericMessage, "\n");
  #endif

  return 1;
}

/**************************************************************************/
/*!
    @brief  Exchanges an APDU with the currently inlisted peer

    @param  send            Pointer to data to send
    @param  sendLength      Length of the data to send
    @param  response        Pointer to response data
    @param  responseLength  Pointer to the response data length
*/
/**************************************************************************/
bool TagReaderWriter::inDataExchange(uint8_t * send, uint8_t sendLength, uint8_t * response, uint8_t * responseLength) {
  if (sendLength > PN532_PACKBUFFSIZ-2) {
    #ifdef PN532DEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "APDU length too long for packet buffer\n");
    #endif
    return false;
  }
  uint8_t i;

  pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = _inListedTag;
  for (i=0; i<sendLength; ++i) {
    pn532_packetbuffer[i+2] = send[i];
  }

  if (!sendCommandCheckAck(pn532_packetbuffer,sendLength+2,1000)) {
    #ifdef PN532DEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Could not send APDU\n");
    #endif
    return false;
  }

  if (!waitready(1000)) {
    #ifdef PN532DEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Response never received for APDU...\n");
    #endif
    return false;
  }

  readdata(pn532_packetbuffer,sizeof(pn532_packetbuffer));

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      #ifdef PN532DEBUG
    	reprap.GetPlatform().MessageF(GenericMessage, "Length check invalid\n");
        reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",length);
        reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",(~length)+1);
      #endif
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INDATAEXCHANGE) {
      if ((pn532_packetbuffer[7] & 0x3f)!=0) {
        #ifdef PN532DEBUG
    	  reprap.GetPlatform().MessageF(GenericMessage, "Status code indicates an error\n");
        #endif
        return false;
      }

      length -= 3;

      if (length > *responseLength) {
        length = *responseLength; // silent truncation...
      }

      for (i=0; i<length; ++i) {
        response[i] = pn532_packetbuffer[8+i];
      }
      *responseLength = length;

      return true;
    }
    else {
      reprap.GetPlatform().MessageF(GenericMessage, "Don't know how to handle this command: ");
      reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",pn532_packetbuffer[6]);
      return false;
    }
  }
  else {
	  reprap.GetPlatform().MessageF(GenericMessage, "Preamble missing");
    return false;
  }
}

/**************************************************************************/
/*!
    @brief  'InLists' a passive target. PN532 acting as reader/initiator,
            peer acting as card/responder.
*/
/**************************************************************************/
bool TagReaderWriter::inListPassiveTarget() {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;
  pn532_packetbuffer[2] = 0;

  #ifdef PN532DEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "About to inList passive target");
  #endif

  if (!sendCommandCheckAck(pn532_packetbuffer,3,1000)) {
    #ifdef PN532DEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Could not send inlist message\n");
    #endif
    return false;
  }

  if (!waitready(30000)) {
    return false;
  }

  readdata(pn532_packetbuffer,sizeof(pn532_packetbuffer));

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      #ifdef PN532DEBUG
    	reprap.GetPlatform().MessageF(GenericMessage, "Length check invalid\n");
		reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",length);
		reprap.GetPlatform().MessageF(GenericMessage, "%02x\n",(~length)+1);
      #endif
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INLISTPASSIVETARGET) {
      if (pn532_packetbuffer[7] != 1) {
        #ifdef PN532DEBUG
		  reprap.GetPlatform().MessageF(GenericMessage, "Unhandled number of targets inlisted\n");
        #endif
    	  reprap.GetPlatform().MessageF(GenericMessage, "Number of tags inlisted:\n");

        PN532DEBUGPRINT.println(pn532_packetbuffer[7]);
        return false;
      }

      _inListedTag = pn532_packetbuffer[8];
      reprap.GetPlatform().MessageF(GenericMessage, "Tag number: ");
      PN532DEBUGPRINT.println(_inListedTag);

      return true;
    } else {
      #ifdef PN532DEBUG
        reprap.GetPlatform().MessageF(GenericMessage, "Unexpected response to inlist passive host");
      #endif
      return false;
    }
  }
  else {
    #ifdef PN532DEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Preamble missing\n");
    #endif
    return false;
  }

  return true;
}


/***** Mifare Classic Functions ******/

/**************************************************************************/
/*!
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
bool TagReaderWriter::mifareclassic_IsFirstBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock) % 4 == 0);
  else
    return ((uiBlock) % 16 == 0);
}

/**************************************************************************/
/*!
      Indicates whether the specified block number is the sector trailer
*/
/**************************************************************************/
bool TagReaderWriter::mifareclassic_IsTrailerBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock + 1) % 4 == 0);
  else
    return ((uiBlock + 1) % 16 == 0);
}

/**************************************************************************/
/*!
    Tries to authenticate a block of memory on a MIFARE card using the
    INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  uid           Pointer to a uint8_t array containing the card UID
    @param  uidLen        The length (in uint8_ts) of the card's UID (Should
                          be 4 for MIFARE Classic)
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  keyNumber     Which key type to use during authentication
                          (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
    @param  keyData       Pointer to a uint8_t array containing the 6 uint8_t
                          key value

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t TagReaderWriter::mifareclassic_AuthenticateBlock (uint8_t * uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t * keyData)
{
  uint8_t i;

  // Hang on to the key and uid data
  memcpy (_key, keyData, 6);
  memcpy (_uid, uid, uidLen);
  _uidLen = uidLen;

  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Trying to authenticate card ");
    TagReaderWriter::PrintHex(_uid, _uidLen);
    reprap.GetPlatform().MessageF(GenericMessage, "Using authentication KEY ");PN532DEBUGPRINT.print(keyNumber ? 'B' : 'A');reprap.GetPlatform().MessageF(GenericMessage, ": ");
    TagReaderWriter::PrintHex(_key, 6);
  #endif

  // Prepare the authentication command //
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
  pn532_packetbuffer[1] = 1;                              /* Max card numbers */
  pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
  pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
  memcpy (pn532_packetbuffer+4, _key, 6);
  for (i = 0; i < _uidLen; i++)
  {
    pn532_packetbuffer[10+i] = _uid[i];                /* 4 uint8_t card ID */
  }

  if (! sendCommandCheckAck(pn532_packetbuffer, 10+_uidLen))
    return 0;

  // Read the response packet
  readdata(pn532_packetbuffer, 12);

  // check if the response is valid and we are authenticated???
  // for an auth success it should be uint8_ts 5-7: 0xD5 0x41 0x00
  // Mifare auth error is technically uint8_t 7: 0x14 but anything other and 0x00 is not good
  if (pn532_packetbuffer[7] != 0x00)
  {
    #ifdef PN532DEBUG
      reprap.GetPlatform().MessageF(GenericMessage, "Authentification failed: ");
      TagReaderWriter::PrintHexChar(pn532_packetbuffer, 12);
    #endif
    return 0;
  }

  return 1;
}

/**************************************************************************/
/*!
    Tries to read an entire 16-uint8_t data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          Pointer to the uint8_t array that will hold the
                          retrieved data (if any)

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t TagReaderWriter::mifareclassic_ReadDataBlock (uint8_t blockNumber, uint8_t * data)
{
  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Trying to read 16 uint8_ts from block ");reprap.GetPlatform().MessageF(GenericMessage, "%u",blockNumber);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 4))
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Failed to receive ACK for read command\n");
    #endif
    return 0;
  }

  /* Read the response packet */
  readdata(pn532_packetbuffer, 26);

  /* If uint8_t 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] != 0x00)
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Unexpected response\n");
      TagReaderWriter::PrintHexChar(pn532_packetbuffer, 26);
    #endif
    return 0;
  }

  /* Copy the 16 data uint8_ts to the output buffer        */
  /* Block content starts at uint8_t 9 of a valid response */
  memcpy (data, pn532_packetbuffer+8, 16);

  /* Display data for debug if requested */
  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Block ");
    reprap.GetPlatform().MessageF(GenericMessage, "%u\n",blockNumber);
    TagReaderWriter::PrintHexChar(data, 16);
  #endif

  return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 16-uint8_t data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          The uint8_t array that contains the data to write.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t TagReaderWriter::mifareclassic_WriteDataBlock (uint8_t blockNumber, uint8_t * data)
{
  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Trying to write 16 uint8_ts to block ");reprap.GetPlatform().MessageF(GenericMessage, "%u\n",blockNumber);
  #endif

  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
  memcpy (pn532_packetbuffer+4, data, 16);          /* Data Payload */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 20))
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Failed to receive ACK for write command\n");
    #endif
    return 0;
  }
  delay(10);

  /* Read the response packet */
  readdata(pn532_packetbuffer, 26);

  return 1;
}

/**************************************************************************/
/*!
    Formats a Mifare Classic card to store NDEF Records

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t TagReaderWriter::mifareclassic_FormatNDEF (void)
{
  uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
  // for the MAD sector in NDEF records (sector 0)

  // Write block 1 and 2 to the card
  if (!(mifareclassic_WriteDataBlock (1, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock (2, sectorbuffer2)))
    return 0;
  // Write key A and access rights card
  if (!(mifareclassic_WriteDataBlock (3, sectorbuffer3)))
    return 0;

  // Seems that everything was OK (?!)
  return 1;
}

/**************************************************************************/
/*!
    Writes an NDEF URI Record to the specified sector (1..15)

    Note that this function assumes that the Mifare Classic card is
    already formatted to work as an "NFC Forum Tag" and uses a MAD1
    file system.  You can use the NXP TagWriter app on Android to
    properly format cards for this.

    @param  sectorNumber  The sector that the URI record should be written
                          to (can be 1..15 for a 1K card)
    @param  uriIdentifier The uri identifier code (0 = none, 0x01 =
                          "http://www.", etc.)
    @param  url           The uri text to write (max 38 characters).

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t TagReaderWriter::mifareclassic_WriteNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char * url)
{
  // Figure out how long the string is
  uint8_t len = strlen(url);

  // Make sure we're within a 1K limit for the sector number
  if ((sectorNumber < 1) || (sectorNumber > 15))
    return 0;

  // Make sure the URI payload is between 1 and 38 chars
  if ((len < 1) || (len > 38))
    return 0;

  // Note 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A
  // in NDEF records

  // Setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
  uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, (uint8_t)(len+5), 0xD1, 0x01, (uint8_t)(len+1), 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (len <= 6)
  {
    // Unlikely we'll get a url this short, but why not ...
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer1[len+9] = 0xFE;
  }
  else if (len == 7)
  {
    // 0xFE needs to be wrapped around to next block
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer2[0] = 0xFE;
  }
  else if ((len > 7) && (len <= 22))
  {
    // Url fits in two blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer2[len-7] = 0xFE;
  }
  else if (len == 23)
  {
    // 0xFE needs to be wrapped around to final block
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer3[0] = 0xFE;
  }
  else
  {
    // Url fits in three blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, 16);
    memcpy (sectorbuffer3, url+23, len-24);
    sectorbuffer3[len-22] = 0xFE;
  }

  // Now write all three blocks back to the card
  if (!(mifareclassic_WriteDataBlock (sectorNumber*4, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+1, sectorbuffer2)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+2, sectorbuffer3)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+3, sectorbuffer4)))
    return 0;

  // Seems that everything was OK (?!)
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
uint8_t TagReaderWriter::mifareultralight_ReadPage (uint8_t page, uint8_t * buffer)
{
  if (page >= 64)
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Page value out of range\n");
    #endif
    return 0;
  }

  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Reading page ");reprap.GetPlatform().MessageF(GenericMessage, "%u\n",page);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                   /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = page;                /* Page Number (0..63 in most cases) */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 4))
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Failed to receive ACK for write command\n");
    #endif
    return 0;
  }

  /* Read the response packet */
  readdata(pn532_packetbuffer, 26);
  #ifdef MIFAREDEBUG
  	reprap.GetPlatform().MessageF(GenericMessage, "Received: \n");
    TagReaderWriter::PrintHexChar(pn532_packetbuffer, 26);
  #endif

  /* If uint8_t 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] == 0x00)
  {
    /* Copy the 4 data uint8_ts to the output buffer         */
    /* Block content starts at uint8_t 9 of a valid response */
    /* Note that the command actually reads 16 uint8_t or 4  */
    /* pages at a time ... we simply discard the last 12  */
    /* uint8_ts                                              */
    memcpy (buffer, pn532_packetbuffer+8, 4);
  }
  else
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Unexpected response reading block: \n");
      TagReaderWriter::PrintHexChar(pn532_packetbuffer, 26);
    #endif
    return 0;
  }

  /* Display data for debug if requested */
  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Page ");reprap.GetPlatform().MessageF(GenericMessage, "%u",page);reprap.GetPlatform().MessageF(GenericMessage, ":\n");
    TagReaderWriter::PrintHexChar(buffer, 4);
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
uint8_t TagReaderWriter::mifareultralight_WritePage (uint8_t page, uint8_t * data)
{

  if (page >= 64)
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Page value out of range\n");
    #endif
    // Return Failed Signal
    return 0;
  }

  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Trying to write 4 uint8_t page");reprap.GetPlatform().MessageF(GenericMessage, "%u\n",page);
  #endif

  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_ULTRALIGHT_CMD_WRITE;       /* Mifare Ultralight Write command = 0xA2 */
  pn532_packetbuffer[3] = page;            /* Page Number (0..63 for most cases) */
  memcpy (pn532_packetbuffer+4, data, 4);          /* Data Payload */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 8))
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Failed to receive ACK for write command\n");
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


/***** NTAG2xx Functions ******/

/**************************************************************************/
/*!
    Tries to read an entire 4-uint8_t page at the specified address.

    @param  page        The page number (0..63 in most cases)
    @param  buffer      Pointer to the uint8_t array that will hold the
                        retrieved data (if any)
*/
/**************************************************************************/
uint8_t TagReaderWriter::ntag2xx_ReadPage (uint8_t page, uint8_t * buffer)
{
  // TAG Type       PAGES   USER START    USER STOP
  // --------       -----   ----------    ---------
  // NTAG 203       42      4             39
  // NTAG 213       45      4             39
  // NTAG 215       135     4             129
  // NTAG 216       231     4             225

  if (page >= 231)
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Page value out of range\n");
    #endif
    return 0;
  }

  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Reading page ");reprap.GetPlatform().MessageF(GenericMessage, "%u\n",page);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                   /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = page;                /* Page Number (0..63 in most cases) */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 4))
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Failed to receive ACK for write command\n");
    #endif
    return 0;
  }

  /* Read the response packet */
  readdata(pn532_packetbuffer, 26);
  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Received: \n");
    TagReaderWriter::PrintHexChar(pn532_packetbuffer, 26);
  #endif

  /* If uint8_t 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] == 0x00)
  {
    /* Copy the 4 data uint8_ts to the output buffer         */
    /* Block content starts at uint8_t 9 of a valid response */
    /* Note that the command actually reads 16 uint8_t or 4  */
    /* pages at a time ... we simply discard the last 12  */
    /* uint8_ts                                              */
    memcpy (buffer, pn532_packetbuffer+8, 4);
  }
  else
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Unexpected response reading block: \n");
      TagReaderWriter::PrintHexChar(pn532_packetbuffer, 26);
    #endif
    return 0;
  }

  /* Display data for debug if requested */
  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Page ");reprap.GetPlatform().MessageF(GenericMessage, "%u",page);reprap.GetPlatform().MessageF(GenericMessage, ":\n");
    TagReaderWriter::PrintHexChar(buffer, 4);
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
uint8_t TagReaderWriter::ntag2xx_WritePage (uint8_t page, uint8_t * data)
{
  // TAG Type       PAGES   USER START    USER STOP
  // --------       -----   ----------    ---------
  // NTAG 203       42      4             39
  // NTAG 213       45      4             39
  // NTAG 215       135     4             129
  // NTAG 216       231     4             225

  if ((page < 4) || (page > 225))
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Page value out of range\n");
    #endif
    // Return Failed Signal
    return 0;
  }

  #ifdef MIFAREDEBUG
    reprap.GetPlatform().MessageF(GenericMessage, "Trying to write 4 uint8_t page");reprap.GetPlatform().MessageF(GenericMessage, "%u\n",page);
  #endif

  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                              /* Card number */
  pn532_packetbuffer[2] = MIFARE_ULTRALIGHT_CMD_WRITE;    /* Mifare Ultralight Write command = 0xA2 */
  pn532_packetbuffer[3] = page;                           /* Page Number (0..63 for most cases) */
  memcpy (pn532_packetbuffer+4, data, 4);                 /* Data Payload */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 8))
  {
    #ifdef MIFAREDEBUG
	  reprap.GetPlatform().MessageF(GenericMessage, "Failed to receive ACK for write command\n");
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
    Writes an NDEF URI Record starting at the specified page (4..nn)

    Note that this function assumes that the NTAG2xx card is
    already formatted to work as an "NFC Forum Tag".

    @param  uriIdentifier The uri identifier code (0 = none, 0x01 =
                          "http://www.", etc.)
    @param  url           The uri text to write (null-terminated string).
    @param  dataLen       The size of the data area for overflow checks.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t TagReaderWriter::ntag2xx_WriteNDEFURI (uint8_t uriIdentifier, char * url, uint8_t dataLen)
{
  uint8_t pageBuffer[4] = { 0, 0, 0, 0 };

  // Remove NDEF record overhead from the URI data (pageHeader below)
  uint8_t wrapperSize = 12;

  // Figure out how long the string is
  uint8_t len = strlen(url);

  // Make sure the URI payload will fit in dataLen (include 0xFE trailer)
  if ((len < 1) || (len+1 > (dataLen-wrapperSize)))
    return 0;

  // Setup the record header
  // See NFCForum-TS-Type-2-Tag_1.1.pdf for details
  uint8_t pageHeader[12] =
  {
    /* NDEF Lock Control TLV (must be first and always present) */
    0x01,         /* Tag Field (0x01 = Lock Control TLV) */
    0x03,         /* Payload Length (always 3) */
    0xA0,         /* The position inside the tag of the lock uint8_ts (upper 4 = page address, lower 4 = uint8_t offset) */
    0x10,         /* Size in bits of the lock area */
    0x44,         /* Size in uint8_ts of a page and the number of uint8_ts each lock bit can lock (4 bit + 4 bits) */
    /* NDEF Message TLV - URI Record */
    0x03,         /* Tag Field (0x03 = NDEF Message) */
	(uint8_t)(len+5),        /* Payload Length (not including 0xFE trailer) */
    0xD1,         /* NDEF Record Header (TNF=0x1:Well known record + SR + ME + MB) */
    0x01,         /* Type Length for the record type indicator */
	(uint8_t)(len+1),        /* Payload len */
    0x55,         /* Record Type Indicator (0x55 or 'U' = URI Record) */
    uriIdentifier /* URI Prefix (ex. 0x01 = "http://www.") */
  };

  // Write 12 uint8_t header (three pages of data starting at page 4)
  memcpy (pageBuffer, pageHeader, 4);
  if (!(ntag2xx_WritePage (4, pageBuffer)))
    return 0;
  memcpy (pageBuffer, pageHeader+4, 4);
  if (!(ntag2xx_WritePage (5, pageBuffer)))
    return 0;
  memcpy (pageBuffer, pageHeader+8, 4);
  if (!(ntag2xx_WritePage (6, pageBuffer)))
    return 0;

  // Write URI (starting at page 7)
  uint8_t currentPage = 7;
  char * urlcopy = url;
  while(len)
  {
    if (len < 4)
    {
      memset(pageBuffer, 0, 4);
      memcpy(pageBuffer, urlcopy, len);
      pageBuffer[len] = 0xFE; // NDEF record footer
      if (!(ntag2xx_WritePage (currentPage, pageBuffer)))
        return 0;
      // DONE!
      return 1;
    }
    else if (len == 4)
    {
      memcpy(pageBuffer, urlcopy, len);
      if (!(ntag2xx_WritePage (currentPage, pageBuffer)))
        return 0;
      memset(pageBuffer, 0, 4);
      pageBuffer[0] = 0xFE; // NDEF record footer
      currentPage++;
      if (!(ntag2xx_WritePage (currentPage, pageBuffer)))
        return 0;
      // DONE!
      return 1;
    }
    else
    {
      // More than one page of data left
      memcpy(pageBuffer, urlcopy, 4);
      if (!(ntag2xx_WritePage (currentPage, pageBuffer)))
        return 0;
      currentPage++;
      urlcopy+=4;
      len-=4;
    }
  }

  // Seems that everything was OK (?!)
  return 1;
}


/************** high level communication functions (handles both I2C and SPI) */


/**************************************************************************/
/*!
    @brief  Tries to read the SPI or I2C ACK signal
*/
/**************************************************************************/
bool TagReaderWriter::readack() {
  uint8_t ackbuff[6];

  readdata(ackbuff, 6);

  return (0 == strncmp((char *)ackbuff, (char *)pn532ack, 6));
}


/**************************************************************************/
/*!
    @brief  Return true if the PN532 is ready with a response.
*/
/**************************************************************************/
bool TagReaderWriter::isready() {

    // SPI read status and check if ready.
    #ifdef SPI_HAS_TRANSACTION
      if (_hardwareSPI) SPI.beginTransaction(PN532_SPI_SETTING);
    #endif
    //digitalWrite(_ss, LOW);
    delay(2);

    const uint8_t dataOut[1] = {PN532_SPI_STATREAD};
    uint8_t rawBytes[8];
	MutexLocker lock(Tasks::GetSpiMutex(), 10);
	if (!lock)
	{
		return 0;
	}

	sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

    sspi_transceive_packet(dataOut, rawBytes, 1);

    delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);

    // read uint8_t
    uint8_t x = rawBytes[0];

    //digitalWrite(_ss, HIGH);
    #ifdef SPI_HAS_TRANSACTION
      if (_hardwareSPI) SPI.endTransaction();
    #endif

    // Check if status is ready.
    return x == PN532_SPI_READY;
}

/**************************************************************************/
/*!
    @brief  Waits until the PN532 is ready.

    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
bool TagReaderWriter::waitready(uint16_t timeout) {
  uint16_t timer = 0;
  while(!isready()) {
    if (timeout != 0) {
      timer += 10;
      if (timer > timeout) {
    	  reprap.GetPlatform().MessageF(GenericMessage, "TIMEOUT!\n");
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
void TagReaderWriter::readdata(uint8_t* buff, uint8_t n) {
  if (_usingSPI) {
    // SPI write.
    #ifdef SPI_HAS_TRANSACTION
      if (_hardwareSPI) SPI.beginTransaction(PN532_SPI_SETTING);
    #endif

    sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

    const uint8_t dataOut[1] = {PN532_SPI_DATAREAD};

	sspi_transceive_packet(dataOut, buff, n);

    //spi_write(PN532_SPI_DATAREAD);

    #ifdef PN532DEBUG
      reprap.GetPlatform().MessageF(GenericMessage, "Reading: ");
    #endif
    for (uint8_t i=0; i<n; i++) {
      /*delay(1);
      buff[i] = spi_read();*/
      #ifdef PN532DEBUG
        reprap.GetPlatform().MessageF(GenericMessage, " 0x");
        reprap.GetPlatform().MessageF(GenericMessage, "%02x",buff[i]);
      #endif
    }

    #ifdef PN532DEBUG
    	reprap.GetPlatform().MessageF(GenericMessage, "\n");
    #endif

	delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);
    #ifdef SPI_HAS_TRANSACTION
      if (_hardwareSPI) SPI.endTransaction();
    #endif
  }
}

/**************************************************************************/
/*!
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in uint8_ts
*/
/**************************************************************************/
void TagReaderWriter::writecommand(uint8_t* cmd, uint8_t cmdlen) {
  if (_usingSPI) {
    // SPI command write.
    uint8_t checksum;

    cmdlen++;

    #ifdef PN532DEBUG
      reprap.GetPlatform().MessageF(GenericMessage, "\nSending: ");
    #endif

    #ifdef SPI_HAS_TRANSACTION
      if (_hardwareSPI) SPI.beginTransaction(PN532_SPI_SETTING);
    #endif
    sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

    uint8_t dataOut[1] = {PN532_SPI_DATAWRITE};
    uint8_t rawBytes[8];

    //spi_write(PN532_SPI_DATAWRITE);
    sspi_transceive_packet(dataOut, rawBytes, 0);

    checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
    //spi_write(PN532_PREAMBLE);
    dataOut[0] = PN532_PREAMBLE;
    sspi_transceive_packet(dataOut, rawBytes, 0);
    //spi_write(PN532_PREAMBLE);
    dataOut[0] = PN532_PREAMBLE;
    sspi_transceive_packet(dataOut, rawBytes, 0);
    //spi_write(PN532_STARTCODE2);
    dataOut[0] = PN532_STARTCODE2;
    sspi_transceive_packet(dataOut, rawBytes, 0);
    //spi_write(cmdlen);
    dataOut[0] = cmdlen;
    sspi_transceive_packet(dataOut, rawBytes, 0);
    //spi_write(~cmdlen + 1);
    dataOut[0] = (~cmdlen + 1);
    sspi_transceive_packet(dataOut, rawBytes, 0);
    //spi_write(PN532_HOSTTOPN532);
    dataOut[0] = PN532_HOSTTOPN532;
    sspi_transceive_packet(dataOut, rawBytes, 0);
    checksum += PN532_HOSTTOPN532;

    #ifdef PN532DEBUG
    reprap.GetPlatform().MessageF(GenericMessage, " 0x"); 	reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)PN532_PREAMBLE);
      reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)PN532_PREAMBLE);
      reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)PN532_STARTCODE2);
      reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)cmdlen);
      reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)(~cmdlen + 1));
      reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)PN532_HOSTTOPN532);
    #endif

    for (uint8_t i=0; i<cmdlen-1; i++) {
      //spi_write(cmd[i]);
      dataOut[0] = cmd[i];
      sspi_transceive_packet(dataOut, rawBytes, 0);
      checksum += cmd[i];
      #ifdef PN532DEBUG
        reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)cmd[i]);
      #endif
    }

    //spi_write(~checksum);
    dataOut[0] = (~checksum);
    sspi_transceive_packet(dataOut, rawBytes, 0);
    //spi_write(PN532_POSTAMBLE);
    dataOut[0] = PN532_POSTAMBLE;
    sspi_transceive_packet(dataOut, rawBytes, 0);

    delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);
    #ifdef SPI_HAS_TRANSACTION
      if (_hardwareSPI) SPI.endTransaction();
    #endif

    #ifdef PN532DEBUG
      reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)~checksum);
      reprap.GetPlatform().MessageF(GenericMessage, " 0x"); reprap.GetPlatform().MessageF(GenericMessage, "%02x",(uint8_t)PN532_POSTAMBLE);
      PN532DEBUGPRINT.println();
    #endif
  }
}
/************** low level SPI */


