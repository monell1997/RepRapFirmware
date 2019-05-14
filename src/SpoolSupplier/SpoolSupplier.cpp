/*
 * SpoolSupplier.cpp
 *
 *  Created on: 25 abr. 2019
 *      Author: agarciamoreno
 */
#include "RepRap.h"
#include "Heating/Heat.h"
#include <SpoolSupplier/SpoolSupplier.h>
#include "OutputMemory.h"
constexpr uint32_t SpoolSupplierIntervalMillisRefresh = 2000;		// interval spoolsupplier data refresh
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
void SpoolSupplier::Set_Master_Status(bool status){
	master = status;
}
void SpoolSupplier::SendtoPrinter(void){
	if(master){
#ifdef SERIAL_AUX_DEVICE
		OutputBuffer *r;
		if (!OutputBuffer::Allocate(r))
		{
			return;
		}

		r->copy("{");
		for(int i = 0; i<N_Spools;i++){

			r->catf("[\"spool_id_%d\":\"%u\",\"current_temp\":\"%.1f\",\"target_temp\":\"%.1f\"]",i, spool_id[i], (double)current_temperature[i], (double)target_temperature[i]);

		}
		r->cat("}\n");

		reprap.GetPlatform().Message(Uart0_duet2, r);
		//reprap.GetPlatform().MessageF(Uart0_duet2, "Rece");
		//OutputBuffer::ReleaseAll(r);
		//reprap.GetPlatform().FlushAuxMessages();

#endif
	}

}
void SpoolSupplier::Spin(void){

	if(master){
		// See if it is time to spin the PIDs
		const uint32_t now = millis();
		Heat& heat = reprap.GetHeat();
		if (now - lastTime >= SpoolSupplierIntervalMillisRefresh)
		{
			if(N_Spools <= NumChamberHeaters){

				for(uint8_t i = 0; i<N_Spools;i++){

				int8_t heater = (NumChamberHeaters > i) ? heat.GetChamberHeater(i) : -1;


				target_temperature[i] = heat.GetActiveTemperature(heater);



				current_temperature[i] = heat.GetTemperature(heater);
				}
			}
			lastTime = millis();
		}
	}
}

#endif
