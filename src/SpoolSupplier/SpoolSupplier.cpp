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
	}
	// Set Default a default temp
	for(int i = 0; i<N_Spools;i++){
		current_temperature[i] = 0;
	}
	// Set Default filament remaining
	for(int i = 0; i<N_Spools;i++){
		spool_remaining[i] = 100;
	}
	// Set Default filament id
	for(int i = 0; i<N_Spools;i++){
		spool_id[i] = Default_id;
	}
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

#endif
