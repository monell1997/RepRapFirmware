/*
 * SpoolRDIFReader.cpp
 *
 *  Created on: 3 jun. 2019
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


// Constructor
SpoolRDIF_Reader::SpoolRDIF_Reader() {

	for (size_t pn : ARRAY_INDICES(reader))
	{
		reader[pn] = new PN532Handler(pn);
	}

}

SpoolRDIF_Reader::~SpoolRDIF_Reader() {
	// TODO Auto-generated destructor stub
}

// Factory method
void SpoolRDIF_Reader::Create(uint8_t channel)
{
	PN532Handler *sr = nullptr;

	if(channel < MaxSpiTempSensors){
		sr = new PN532Handler(channel);
	}

	if (sr != nullptr)
	{
		sr->Init();
		reader[channel] = sr;
	}
	return;
}
void SpoolRDIF_Reader::Spin()
{

	for (PN532Handler *& r : reader)
	{
		r->Spin();
	}
	//reader[0]->Spin();

}
