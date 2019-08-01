/*
 * SpoolRDIFReader.h
 *
 *  Created on: 3 jun. 2019
 *      Author: agarciamoreno
 */




#ifndef SRC_RFID_SPOOLRDIFREADER_H_
#define SRC_RFID_SPOOLRDIFREADER_H_

#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Tasks.h"
#include "Wire.h"
#include "SharedSpi.h"
#ifdef BCN3D_DEV

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

#endif
#endif /* SRC_RFID_SPOOLRDIFREADER_H_ */
