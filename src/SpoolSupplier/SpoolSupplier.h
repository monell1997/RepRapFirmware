/*
 * SpoolSupplier.h
 *
 *  Created on: 25 abr. 2019
 *      Author: agarciamoreno
 */
#include "RepRap.h"
#include "RFID/RFIDdevicestatus.h"
#include "FilamentDictionary.h"
#ifndef SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_
#define SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_
#ifdef BCN3D_DEV

#define N_Spools		 	2
#define Default_temp	 	-273.15
#define Default_hum	 		0
class SpoolSupplier {
public:
	SpoolSupplier();
	void Spin(void);

	uint8_t Get_Spool_Remaining(size_t idex);
	void Set_Spool_Remaining(size_t idex, uint8_t rem);
	void Set_Spool_Remaining(size_t idex, const uint8_t * data, const uint32_t numBytes);

	float Get_Current_Humidity(size_t idex);
	void Set_Current_Humidity(size_t idex, float current);

	float Get_Target_Temperature(size_t idex);
	void Set_Target_Temperature(size_t idex, float target);

	float Get_Current_Temperature(size_t idex);
	void Update_Current_Temperature(size_t idex, float temp);

	FilamentDictionary Get_Spool_id(size_t idex);
	void Set_Spool_id(size_t idex, uint32_t id);
	void Set_Spool_id(size_t idex, const uint8_t * data, const uint32_t numBytes);

	void Set_Master_Status(bool status);

	void SendtoPrinter(const MessageType type);

	void PrintJSON(const MessageType type);
	void PrintStatus(const MessageType type);

private:
	float target_temperature[N_Spools];
	float current_temperature[N_Spools];
	float current_humidity[N_Spools];
	uint8_t spool_remaining[N_Spools];
	FilamentDictionary spool_id[N_Spools];
	static Mutex SpoolSupplierMutex;

	RFID_device_status RWrfid_s;

	bool master; // true if Edurne
	bool online; // true if Edurne
	uint32_t lastTime;											// The last time our Spin() was called



};


#endif
#endif /* SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_ */
