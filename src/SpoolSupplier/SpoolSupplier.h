/*
 * SpoolSupplier.h
 *
 *  Created on: 25 abr. 2019
 *      Author: agarciamoreno
 */
#include "RepRap.h"
#ifndef SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_
#define SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_
#ifdef BCN3D_DEV

#define N_Spools		 	1
#define Default_temp	 	-273.15
#define Default_id 			0
class SpoolSupplier {
public:
	SpoolSupplier();
	void Spin(void);
	uint8_t Get_Spool_Remaining(size_t idex);
	void Set_Spool_Remaining(size_t idex, uint8_t rem);
	float Get_Target_Temperature(size_t idex);
	void Set_Target_Temperature(size_t idex, float target);
	float Get_Current_Temperature(size_t idex);
	void Update_Current_Temperature(size_t idex, float temp);
	unsigned int Get_Spool_id(size_t idex);
	void Set_Master_Status(bool status);
	void Set_Spool_id(size_t idex, unsigned int id);
	void SendtoPrinter(const MessageType type);
	void PrintJSON(const MessageType type);
private:
	float target_temperature[N_Spools];
	float current_temperature[N_Spools];
	uint8_t spool_remaining[N_Spools];
	unsigned int spool_id[N_Spools];

	static Mutex SpoolSupplierMutex;

	bool master; // true if Edurne
	bool online; // true if Edurne
	uint32_t lastTime;											// The last time our Spin() was called
};


#endif
#endif /* SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_ */
