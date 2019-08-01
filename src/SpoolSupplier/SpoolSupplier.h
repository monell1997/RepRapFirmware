/*
 * SpoolSupplier.h
 *
 *  Created on: 25 abr. 2019
 *      Author: agarciamoreno
 */
#include "RepRap.h"
#include "RFID/RFIDdevicestatus.h"
#include "FilamentDictionary.h"
#include "FilamentMonitors/FilamentMonitor.h"
#ifndef SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_
#define SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_
#ifdef BCN3D_DEV

#define N_Spools		 	4
#define Default_temp	 	-273.15
#define Default_hum	 		0


enum class ChangeFilStatus : uint8_t
{
	ok,
	requested,
	failed
};
class SpoolSupplier {
public:
	SpoolSupplier();
	void Spin(void);

	uint8_t getSpoolRemaining(size_t idex);
	void setSpoolRemaining(size_t idex, uint8_t rem);
	void setSpoolRemaining(size_t idex, const uint8_t * data, const uint32_t numBytes);

	float getCurrentHumidity(size_t idex);
	void setCurrentHumidity(size_t idex, float current);

	float getTargetTemperature(size_t idex);
	void setTargetTemperature(size_t idex, float target);

	float getCurrentTemperature(size_t idex);
	void updateCurrentTemperature(size_t idex, float temp);

	FilamentDictionary getSpoolID(size_t idex);
	void setSpoolID(size_t idex, uint32_t id);
	void setSpoolID(size_t idex, const uint8_t * data, const uint32_t numBytes);
	void setSpoolFRS(size_t idex, int frs);

	void setMasterStatus(bool status);
	bool getMasterStatus();

	void setLoadedFlag(size_t idex, uint8_t val);
	bool getSpoolAvailable(size_t idex);

	void setChangeFilStatus(size_t idex, ChangeFilStatus status);

	void sendToPrinter(const MessageType type);

	void printJSON(const MessageType type);
	void printStatus(const MessageType type);

private:

	float targetTemperature[N_Spools];
	float currentTemperature[N_Spools];
	float currentHumidity[N_Spools];

	ChangeFilStatus change_fil_status[N_Spools]; //

	uint8_t spoolRemaining[N_Spools];
	uint8_t spoolLoaded[N_Spools];	// bobina cargada? 0 no , 1 extr, 2 extr

	FilamentSensorStatus spoolFRS[N_Spools];	//1 no fil , 0 is fil

	FilamentDictionary spoolID[N_Spools];

	static Mutex SpoolSupplierMutex;

	RFID_device_status RWrfid_s;

	bool master; // true whether is Edurne
	bool online; // true if edurne is connected to the printer

	uint32_t lastTime;											// The last time our Spin() was called



};


#endif
#endif /* SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_ */
