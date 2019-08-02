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

		targetTemperature[i] = 0;

	// Set Default a default temp

		currentTemperature[i] = Default_temp;

	// Set Default a default hum

		currentHumidity[i] = Default_hum;

	// Set Default filament remaining

		spoolRemaining[i] = 0;

	// Set Default filament id

		spoolID[i] = FilamentDictionary::defauld_filament;

	// Set Default FRS value

		spoolFRS[i] = FilamentSensorStatus::ok;
	// Set Default Spool State

		spoolLoaded[i] = 0;

	// Set Default Change Fil Status

		change_fil_status[i] = ChangeFilStatus::ok;
	}
	master = false;
	online = false;
	SpoolSupplierMutex.Create("SpoolSupplier");
}
uint8_t SpoolSupplier::getSpoolRemaining(size_t idex){
	return spoolRemaining[idex];
}
void SpoolSupplier::setSpoolRemaining(size_t idex, uint8_t rem){
	spoolRemaining[idex] = rem;
}
void SpoolSupplier::setSpoolRemaining(size_t idex, const uint8_t * data, const uint32_t numBytes){
	uint8_t rem = 0;
	for(size_t i = 0; i< numBytes; i++){// should be 1
		rem |= (data[i]<<8*i);
	}
	spoolRemaining[idex] = rem;
}
float SpoolSupplier::getCurrentHumidity(size_t idex){
	return currentHumidity[idex];
}
void SpoolSupplier::setCurrentHumidity(size_t idex, float current){
	currentHumidity[idex] = current;
}
float SpoolSupplier::getTargetTemperature(size_t idex){
	return targetTemperature[idex];
}
void SpoolSupplier::setTargetTemperature(size_t idex, float target){
	targetTemperature[idex] = target;
}
float SpoolSupplier::getCurrentTemperature(size_t idex){
	return currentTemperature[idex];
}
void SpoolSupplier::updateCurrentTemperature(size_t idex, float temp){ // if the temperature is updated, we assume that Edurne is connected
	if(!master)lastTime = millis();
	currentTemperature[idex] = temp;
}
FilamentDictionary SpoolSupplier::getSpoolID(size_t idex){
	return spoolID[idex];
}
void SpoolSupplier::setSpoolID(size_t idex, uint32_t id){//Manually
	Heat& heat = reprap.GetHeat();
	if((FilamentDictionary)id != spoolID[idex]){

		int8_t heater = (NumChamberHeaters > idex) ? heat.GetChamberHeater(idex) : -1;

		heat.Activate(heater);
		heat.SetActiveTemperature(heater, FilamentDictionaryTargetTemp((FilamentDictionary)id));

	}
	spoolID[idex] = (FilamentDictionary)id;

}
void SpoolSupplier::setSpoolFRS(size_t idex, int frs){//Manually
	spoolFRS[idex] = (FilamentSensorStatus)frs;
}
void SpoolSupplier::setSpoolID(size_t idex, const uint8_t * data, const uint32_t numBytes){//Auto from RFID tag, auto heat-up
	uint32_t id = 0;
	Heat& heat = reprap.GetHeat();
	for(size_t i = 0; i< numBytes; i++){// should be 4
		id |= (data[i]<<8*i);
	}

	if((FilamentDictionary)id != spoolID[idex]){

		int8_t heater = (NumChamberHeaters > idex) ? heat.GetChamberHeater(idex) : -1;

		heat.Activate(heater);
		heat.SetActiveTemperature(heater, FilamentDictionaryTargetTemp((FilamentDictionary)id));

	}
	spoolID[idex] = (FilamentDictionary)id;


}
void SpoolSupplier::setMasterStatus(bool status){//True is edurne, false is a printer
	master = status;
}
bool SpoolSupplier::getMasterStatus(){//True is edurne, false is a printer
	return master;
}
void SpoolSupplier::setLoadedFlag(size_t idex, uint8_t val){//True is edurne, false is a printer
	spoolLoaded[idex] = val;
}
bool SpoolSupplier::getSpoolAvailable(size_t idex){

	if(spoolLoaded[idex]){
		return 0;				//this spool is in use
	}
	if(spoolFRS[idex]!=FilamentSensorStatus::ok){
		return 0;				 //this spool isn't ok
	}

	return 1; // Spool available
}
void SpoolSupplier::setChangeFilStatus(size_t idex, ChangeFilStatus status){
	change_fil_status[idex] = status;
}
void SpoolSupplier::sendToPrinter(const MessageType type){
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

			r->catf("%lu",(uint32_t)spoolID[i]);
		}
		r->cat(" R");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%u",spoolRemaining[i]);
		}
		r->cat(" H");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%.1f",(double)currentHumidity[i]);
		}
		r->cat(" C");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%.1f",(double)currentTemperature[i]);
		}
		r->cat(" S");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%.1f",(double)targetTemperature[i]);
		}
		r->cat(" F");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%d",(int)spoolFRS[i]);
		}
		r->cat(" L");
		for(i = 0; i<N_Spools;i++){

			if(i >0){r->cat(":");}

			r->catf("%d",(int)spoolLoaded[i]);
		}
		r->cat("\n");

		reprap.GetPlatform().Message(type, r);
		//reprap.GetPlatform().MessageF(Uart0_duet2, "Rece");
		//OutputBuffer::ReleaseAll(r);
		//reprap.GetPlatform().FlushAuxMessages();

#endif
	}

}
void SpoolSupplier::printStatus(const MessageType type){

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
		r->cat(FilamentDictionaryString(spoolID[i]));
		r->cat(", ");
		r->catf("Filament Remaining %u%%, ",spoolRemaining[i]);
		r->catf("Chamber Humidity %.1f%%, ",(double)currentHumidity[i]);
		r->catf("Chamber Temperature %.1f/%.1f ",(double)currentTemperature[i],(double)targetTemperature[i]);
		//r->catf("FRS %d: \n", (int)spool_FRS[i]);
		r->catf("Loaded: %d ,",(int)spoolLoaded[i]);
		r->catf("FRS: ");
		r->cat(FilamentMonitor::GetErrorMessage(spoolFRS[i]));
		r->catf(" \n");
	}
	reprap.GetPlatform().Message(type, r);
}
void SpoolSupplier::printJSON(const MessageType type){

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

		r->catf("[\"sl_id_%d\":\"%lu\"",i, (uint32_t)spoolID[i]);
		r->catf(",\"sl_rem\":\"%u\"",spoolRemaining[i]);
		r->catf(",\"c_h\":\"%.1f\"",(double)currentHumidity[i]);
		r->catf(",\"c_t\":\"%.1f\"",(double)currentTemperature[i]);
		r->catf(",\"t_t\":\"%.1f\"",(double)targetTemperature[i]);
		r->catf(",\"frs\":\"%d\"]",(int)spoolFRS[i]);
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

				targetTemperature[i]  = heat.GetActiveTemperature(heater);
				currentTemperature[i] = heat.GetTemperature(heater);
				//FilamentMonitor::InitStatic();
				spoolFRS[i] = FilamentMonitor::GetFilamentMonitorState(i);
				//reprap.GetHdcSensorHardwareInterface().GetTemperatureOrHumidity(i==0?0:3,current_temperature[i],false);
				//reprap.GetHdcSensorHardwareInterface().GetTemperatureOrHumidity(i==0?0:3,current_humidity[i],true);

				if(spoolFRS[i] != FilamentSensorStatus::ok){
					heat.SetActiveTemperature(heater, 0.0);// apagar
				}else{
					heat.Activate(heater);
					heat.SetActiveTemperature(heater, FilamentDictionaryTargetTemp(spoolID[i]));
				}


				}
			}
			sendToPrinter(ImmediateDirectUart0_duet2Message);
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

			if(toolnumber == 0 || toolnumber == 1)// tool LEFT o RIGHT // solo single extruder
			{
				for(size_t i = 0; i<N_Spools;i++){

						FilamentSensorStatus fstat = spoolFRS[i];

						if (fstat != FilamentSensorStatus::ok)
						{

							//
							if(spoolLoaded[i] == (toolnumber+1)){ //tool 0 is 1 , tool1 is 2
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

											if(spoolID[i] == spoolID[j] && spoolLoaded[j] != 1 && spoolLoaded[j] != 2){// buscando Spool libre

												if(spoolFRS[j] == FilamentSensorStatus::ok){

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
