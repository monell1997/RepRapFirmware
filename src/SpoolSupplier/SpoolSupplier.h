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

#define N_Spools		 	2
#define Default_temp	 	70
#define Default_id 			0
class SpoolSupplier {
public:
	SpoolSupplier();
	float Get_Target_Temperature(size_t idex);
	void Set_Target_Temperature(size_t idex, float target);
	float Get_Current_Temperature(size_t idex);
	void Update_Current_Temperature(size_t idex, float temp);
	unsigned int Get_Spool_id(size_t idex);
	void Set_Spool_id(size_t idex, unsigned int id);

private:
	float target_temperature[N_Spools];
	float current_temperature[N_Spools];
	uint8_t spool_remaining[N_Spools];
	unsigned int spool_id[N_Spools];

};


#endif
#endif /* SRC_SPOOLSUPPLIER_SPOOLSUPPLIER_H_ */
