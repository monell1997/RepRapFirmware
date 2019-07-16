/*
 * SpoolSupplier.cpp
 *
 *  Created on: 25 abr. 2019
 *      Author: agarciamoreno
 */
#include "RepRap.h"
#include "Heating/Heat.h"
#include "SpoolSupplier/SpoolSupplier.h"
#include "OutputMemory.h"
#include "FilamentMonitors/FilamentMonitor.h"
#include "Tools/Tool.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/Move.h"
#include "PrintMonitor.h"
#include "Tools/FilamentHandler.h"


constexpr uint32_t SpoolSupplierIntervalMillisRefresh = 2500;		// interval spoolsupplier data refresh between Printer and Edurne


#ifdef BCN3D_DEV
// Static data
Mutex SpoolSupplier::SpoolSupplierMutex;

SpoolSupplier::SpoolSupplier() {

	for(int i = 0; i<N_Spools;i++){
	// Set Default target temp

		target_temperature[i] = 0;

	// Set Default a default temp

		current_temperature[i] = Default_temp;

	// Set Default a default hum

		current_humidity[i] = Default_hum;

	// Set Default filament remaining

		spool_remaining[i] = 0;

	// Set Default filament id

		spool_id[i] = FilamentDictionary::defauld_filament;

	// Set Default FRS value

		spool_FRS[i] = FilamentSensorStatus::ok;
	// Set Default Spool State

		spool_loaded[i] = 0;

	// Set Default Change Fil Status

		change_fil_status[i] = ChangeFilStatus::ok;
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
void SpoolSupplier::Set_Spool_Remaining(size_t idex, const uint8_t * data, const uint32_t numBytes){
	uint8_t rem = 0;
	for(size_t i = 0; i< numBytes; i++){// should be 1
		rem |= (data[i]<<8*i);
	}
	spool_remaining[idex] = rem;
}
float SpoolSupplier::Get_Current_Humidity(size_t idex){
	return current_humidity[idex];
}
void SpoolSupplier::Set_Current_Humidity(size_t idex, float current){
	current_humidity[idex] = current;
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
FilamentDictionary SpoolSupplier::Get_Spool_id(size_t idex){
	return spool_id[idex];
}
void SpoolSupplier::Set_Spool_id(size_t idex, uint32_t id){//Manually
	spool_id[idex] = (FilamentDictionary)id;
}
void SpoolSupplier::Set_Spool_FRS(size_t idex, int frs){//Manually
	spool_FRS[idex] = (FilamentSensorStatus)frs;
}
void SpoolSupplier::Set_Spool_id(size_t idex, const uint8_t * data, const uint32_t numBytes){//Auto from RFID tag, auto heat-up
	uint32_t id = 0;
	Heat& heat = reprap.GetHeat();
	for(size_t i = 0; i< numBytes; i++){// should be 4
		id |= (data[i]<<8*i);
	}

	if((FilamentDictionary)id != spool_id[idex]){

		int8_t heater = (NumChamberHeaters > idex) ? heat.GetChamberHeater(idex) : -1;

		heat.Activate(heater);
		heat.SetActiveTemperature(heater, FilamentDictionaryTargetTemp((FilamentDictionary)id));

	}
	spool_id[idex] = (FilamentDictionary)id;



}
void SpoolSupplier::Set_Master_Status(bool status){//True is edurne, false is a printer
	master = status;
}
bool SpoolSupplier::Get_Master_Status(){//True is edurne, false is a printer
	return master;
}
void SpoolSupplier::Set_Loaded_flag(size_t idex, uint8_t val){//True is edurne, false is a printer
	spool_loaded[idex] = val;
}
bool SpoolSupplier::Get_Spool_Available(size_t idex){

	if(spool_loaded[idex]){
		return 0;				//this spool is in use
	}
	if(spool_FRS[idex]!=FilamentSensorStatus::ok){
		return 0;				 //this spool isn't ok
	}

	return 1; // Spool available
}
void SpoolSupplier::Set_Change_Fil_Status(size_t idex, ChangeFilStatus status){
	change_fil_status[idex] = status;
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

			r->catf("%lu",(uint32_t)spool_id[i]);
		}
		r->cat(" R");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%u",spool_remaining[i]);
		}
		r->cat(" H");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%.1f",(double)current_humidity[i]);
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
		r->cat(" F");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%d",(int)spool_FRS[i]);
		}
		r->cat(" L");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%d",(int)spool_loaded[i]);
		}
		r->cat("\n");

		reprap.GetPlatform().Message(type, r);
		//reprap.GetPlatform().MessageF(Uart0_duet2, "Rece");
		//OutputBuffer::ReleaseAll(r);
		//reprap.GetPlatform().FlushAuxMessages();

#endif
	}

}
void SpoolSupplier::PrintStatus(const MessageType type){

	MutexLocker lock(SpoolSupplierMutex);
	OutputBuffer *r;
	if (!OutputBuffer::Allocate(r))
	{
		return;
	}
	//JSON Format
	if(!master && !online){
		r->copy("Edurne Disconnected\n");
		reprap.GetPlatform().Message(type, r);
		return;
	}
	r->copy("Edurne Status: \n");
	for(int i = 0; i<N_Spools;i++){
		r->catf("Spool %d: -> ", i);
		r->cat("Material ");
		r->cat(FilamentDictionaryString(spool_id[i]));
		r->cat(", ");
		r->catf("Filament Remaining %u%%, ",spool_remaining[i]);
		r->catf("Chamber Humidity %.1f%%, ",(double)current_humidity[i]);
		r->catf("Chamber Temperature %.1f/%.1f ",(double)current_temperature[i],(double)target_temperature[i]);
		//r->catf("FRS %d: \n", (int)spool_FRS[i]);
		r->catf("Loaded: %d ,",(int)spool_loaded[i]);
		r->catf("FRS: ");
		r->cat(FilamentMonitor::GetErrorMessage(spool_FRS[i]));
		r->catf(" \n");
	}
	reprap.GetPlatform().Message(type, r);
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

		r->catf("[\"sl_id_%d\":\"%lu\"",i, (uint32_t)spool_id[i]);
		r->catf(",\"sl_rem\":\"%u\"",spool_remaining[i]);
		r->catf(",\"c_h\":\"%.1f\"",(double)current_humidity[i]);
		r->catf(",\"c_t\":\"%.1f\"",(double)current_temperature[i]);
		r->catf(",\"t_t\":\"%.1f\"",(double)target_temperature[i]);
		r->catf(",\"frs\":\"%d\"]",(int)spool_FRS[i]);
	}
	r->cat("}\n");
	reprap.GetPlatform().Message(type, r);
}
void SpoolSupplier::Spin(void){
	MutexLocker lock(SpoolSupplierMutex);
	GCodes& gCodes = reprap.GetGCodes();
	if(master){
		// See if it is time to spin the PIDs
		const uint32_t now = millis();
		Heat& heat = reprap.GetHeat();
		if (now - lastTime >= SpoolSupplierIntervalMillisRefresh)
		{
			if(N_Spools <= NumChamberHeaters){

				for(uint8_t i = 0; i<N_Spools;i++){

				int8_t heater = (NumChamberHeaters > i) ? heat.GetChamberHeater(i) : -1;

				target_temperature[i]  = heat.GetActiveTemperature(heater);
				current_temperature[i] = heat.GetTemperature(heater);
				//FilamentMonitor::InitStatic();
				spool_FRS[i] = FilamentMonitor::GetFilamentMonitorState(i);
				//reprap.GetHdcSensorHardwareInterface().GetTemperatureOrHumidity(i==0?0:3,current_temperature[i],false);
				//reprap.GetHdcSensorHardwareInterface().GetTemperatureOrHumidity(i==0?0:3,current_humidity[i],true);

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
		if(online){

			const Tool * const currTool = reprap.GetCurrentTool();
			int toolnumber = currTool->Number();

			if(toolnumber == 0)// tool LEFT
			{
				for(size_t i = 0; i<N_Spools;i++){

						FilamentSensorStatus fstat = spool_FRS[i];

						if (fstat != FilamentSensorStatus::ok)
						{

							//
							if(spool_loaded[i] == 1){
								if(change_fil_status[i] == ChangeFilStatus::ok){
									static bool flag_enter = true;
									if(gCodes.IsReallyPrinting() && flag_enter){

										gCodes.FilamentError(i, fstat);
										flag_enter = false;

									}
									if(gCodes.IsPaused() && fstat != FilamentSensorStatus::ok ){

										// Request unload filament due Edurne
										flag_enter = true;
										uint8_t rq[4]={0};
										rq[0] = 155;//unload
										rq[1] = (uint8_t)i;// spool
										rq[2] = (uint8_t)(toolnumber);// extruder
										reprap.GetFilamentHandler().Request(rq);


										// Search for a filament available
										size_t j = 0;
										while(j < N_Spools){

											if(spool_id[i] == spool_id[j]){

												if(spool_FRS[j] == FilamentSensorStatus::ok){

													rq[0] = 55;//load
													rq[1] = (uint8_t)j;// spool
													rq[2] = (uint8_t)(toolnumber);// extruder
													rq[3] = 100;// resume
													reprap.GetFilamentHandler().Request(rq);
													break;

												}
											}

											j++;
										}
										if(j == N_Spools){
											reprap.GetPlatform().MessageF(HttpMessage, "Only Unload Request, there is not spool available\n");
										}



									}
								}
							}
						}


				}
			}else if(toolnumber == 1){// not ready

			}


		}



	}

}

/*
// Check for and respond to filament errors
void GCodes::CheckFilament()
{
	if (   lastFilamentError != FilamentSensorStatus::ok			// check for a filament error
		&& IsReallyPrinting()
		&& autoPauseGCode->IsCompletelyIdle()
		&& LockMovement(*autoPauseGCode)							// need to lock movement before executing the pause macro
	   )
	{
		String<MediumStringLength> filamentErrorString;
		filamentErrorString.printf("Extruder %u reports %s", lastFilamentErrorExtruder, FilamentMonitor::GetErrorMessage(lastFilamentError));
		DoPause(*autoPauseGCode, PauseReason::filament, filamentErrorString.c_str());
		lastFilamentError = FilamentSensorStatus::ok;
		platform.Message(LogMessage, filamentErrorString.c_str());
	}
}
*/

#endif
