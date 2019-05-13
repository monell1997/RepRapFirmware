/*
 * SpoolSupplier.cpp
 *
 *  Created on: 25 abr. 2019
 *      Author: agarciamoreno
 */
#include "RepRap.h"
#include <SpoolSupplier/SpoolSupplier.h>
#ifdef BCN3D_DEV
SpoolSupplier::SpoolSupplier() {
	// Set Default target temp
	for(int i = 0; i<N_Spools;i++){
		target_temperature[i] = Default_temp;

	// Set Default a default temp

		current_temperature[i] = 0;

	// Set Default filament remaining

		spool_remaining[i] = 100;

	// Set Default filament id

		spool_id[i] = Default_id;
	}
	master = false;
}
float SpoolSupplier::Get_Target_Temperature(size_t idex){
	return target_temperature[idex];
}
void SpoolSupplier::Set_Target_Temperature(size_t idex, float target){
	target_temperature[idex] = target;
}
float SpoolSupplier::Get_Current_Temperature(size_t idex){
	return current_temperature[idex];
}
void SpoolSupplier::Update_Current_Temperature(size_t idex, float temp){
	current_temperature[idex] = temp;
}
unsigned int SpoolSupplier::Get_Spool_id(size_t idex){
	return spool_id[idex];
}
void SpoolSupplier::Set_Spool_id(size_t idex, unsigned int id){
	spool_id[idex] = id;
}
void SpoolSupplier::SendtoPrinter(void){

	{
	#ifdef SERIAL_AUX_DEVICE
		OutputBuffer *response;
		if (OutputBuffer::Allocate(response))
		{
			// change Send the Spool Config to Slave printer

			response->copy("{");
			for(int i = 0; i<N_Spools;i++){

				response->catf("[\"spool_id_%d\":\"%u\",\"target_temp\":\"%.1f\",\"target_temp\":\"%.1f\"]",i, spool_id[i], (double)target_temperature[i], (double)current_temperature[i]);

			}
			response->cat("}\n");
			reprap.GetPlatform().AppendAuxReply(response, true);
			reprap.GetPlatform().FlushAuxMessages();
		}
	#endif
	}
}
void SpoolSupplier::Spin(void){

}

#endif
