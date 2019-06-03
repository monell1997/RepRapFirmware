/*
 * SpoolRDIFReader.h
 *
 *  Created on: 3 jun. 2019
 *      Author: agarciamoreno
 */

#include "RFID/PN532Handler.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Tasks.h"
#include "Wire.h"
#include "SharedSpi.h"

#ifndef SRC_RFID_SPOOLRDIFREADER_H_
#define SRC_RFID_SPOOLRDIFREADER_H_


class PN532Handler;
class SpoolRDIF_Reader {
public:
	SpoolRDIF_Reader();

	void Spin();
	// Initialise or re-initialise the temperature sensor
	//virtual void Init() = 0;

	// Virtual destructor
	virtual ~SpoolRDIF_Reader();

	// Factory method
	void Create(uint8_t channel);

private:
	PN532Handler* reader[MaxSpiTempSensors];										// A PID controller for each heater

};


#endif /* SRC_RFID_SPOOLRDIFREADER_H_ */
