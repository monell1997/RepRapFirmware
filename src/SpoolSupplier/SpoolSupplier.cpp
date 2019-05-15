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
constexpr uint32_t SpoolSupplierIntervalMillisRefresh = 5000;		// interval spoolsupplier data refresh
#ifdef BCN3D_DEV
// Static data
Mutex SpoolSupplier::SpoolSupplierMutex;

SpoolSupplier::SpoolSupplier() {
	// Set Default target temp
	for(int i = 0; i<N_Spools;i++){
		target_temperature[i] = 0;

	// Set Default a default temp

		current_temperature[i] = Default_temp;

	// Set Default filament remaining

		spool_remaining[i] = 0;

	// Set Default filament id

		spool_id[i] = Default_id;
	}
	master = false;
	online = false;
	SpoolSupplierMutex.Create("SpoolSupplier");
}
uint8_t SpoolSupplier::Get_Spool_Remaining(size_t idex){
	return spool_remaining[idex];
}
void SpoolSupplier::Set_Spool_Remaining(size_t idex, uint8_t rem){
	spool_remaining[idex] = rem;
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
void SpoolSupplier::Update_Current_Temperature(size_t idex, float temp){ // if the temperature is updated, we assume that Edurne is connected
	if(!master)lastTime = millis();
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
void SpoolSupplier::SendtoPrinter(const MessageType type){
	MutexLocker lock(SpoolSupplierMutex);

	if(master){
#ifdef SERIAL_AUX_DEVICE
		OutputBuffer *r;
		if (!OutputBuffer::Allocate(r))
		{
			return;
		}

		//Gcode Format, easier to parse
		r->copy("M1061");
		//Report id
		r->cat(" B");
		size_t i;
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%u",spool_id[i]);
		}
		r->cat(" R");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%u",spool_remaining[i]);
		}
		r->cat(" C");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%.1f",(double)current_temperature[i]);
		}
		r->cat(" S");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%.1f",(double)target_temperature[i]);
		}
		r->cat("\n");

		reprap.GetPlatform().Message(type, r);
		//reprap.GetPlatform().MessageF(Uart0_duet2, "Rece");
		//OutputBuffer::ReleaseAll(r);
		//reprap.GetPlatform().FlushAuxMessages();

#endif
	}

}
void SpoolSupplier::PrintJSON(const MessageType type){

	MutexLocker lock(SpoolSupplierMutex);
	OutputBuffer *r;
	if (!OutputBuffer::Allocate(r))
	{
		return;
	}
	//JSON Format
	if(!master && !online){
		r->copy("{[\"edurne_online\":\"false\"]}\n");
		reprap.GetPlatform().Message(type, r);
		return;
	}
	r->copy("{");
	if(!master){
		r->cat("[\"edurne_online\":\"true\"]");
	}
	for(int i = 0; i<N_Spools;i++){

		r->catf("[\"spool_id_%d\":\"%u\"",i, spool_id[i]);
		r->catf(",\"spool_rem\":\"%u\"",spool_remaining[i]);
		r->catf(",\"current_temp\":\"%.1f\"",(double)current_temperature[i]);
		r->catf(",\"target_temp\":\"%.1f\"]",(double)target_temperature[i]);
	}
	r->cat("}\n");
	reprap.GetPlatform().Message(type, r);
}
void SpoolSupplier::Spin(void){
	MutexLocker lock(SpoolSupplierMutex);
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
			SendtoPrinter(ImmediateDirectUart0_duet2Message);
			//PrintJSON(Uart0_duet2);
			lastTime = millis();
		}
	}else{
		const uint32_t now = millis();
		if (now - lastTime >= SpoolSupplierIntervalMillisRefresh*2)
		{
			online = false;
		}else{
			online = true;
		}
	}
}

#endif
