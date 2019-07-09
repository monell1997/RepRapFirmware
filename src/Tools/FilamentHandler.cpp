/*
 * FilamentHandler.cpp
 *
 *  Created on: 18 jun. 2019
 *      Author: agarciamoreno
 */


#include "FilamentHandler.h"
#include "Tool.h"
#include "Filament.h"

#include "GCodes/GCodes.h"
#include "Heating/Heat.h"
#include "Platform.h"
#include "RepRap.h"

// Static data
Mutex FilamentHandler::FilamentHandlerMutex;

FilamentHandler::FilamentHandler() {
	const size_t numExtruders = reprap.GetGCodes().GetNumExtruders();
	for (size_t i = 0; i<numExtruders;i++)
	{
		ToolFilaments[i] = FilamentDictionary::defauld_filament;
	}
	for (size_t i = 0; i<numExtruders;i++)
	{
		isChangingFilamentACK[i] = 0;
	}
	timeout_timer = millis();
	isACK = false;
	FilamentHandlerMutex.Create("FilamentHandler");
}
void FilamentHandler::SetBusyState(int busy){

	isBusy = busy;

}
void FilamentHandler::SetAckState(int ack){

	isACK = (ack == 1 ? true : false);

}
void FilamentHandler::SetFilState(bool fil){
	isFil = fil;
}
void FilamentHandler::SetChangingFilamenACK(uint8_t extruder, uint8_t state)
{
	isChangingFilamentACK[extruder] = state;
}
bool FilamentHandler::isChangingFilamenACK(uint8_t extruder)
{
	return isChangingFilamentACK[extruder];
}
void FilamentHandler::loadfsm(){

	switch(loading_status){
		case load_state::edurne_request:
			reprap.GetPlatform().MessageF(HttpMessage, "enqueuing load\n"); // ASK to edurne if it can do the process
			isBusy = true;
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1090 S%d P1\n",int(status[1])); // ASK to edurne if it can do the process
			loading_status = load_state::edurne_wait;
			isACK = false;
			timeout_timer = millis();
			break;
		case load_state::edurne_wait:
			{
				if(millis() < timeout_timer + 2500){
					if(isACK){
						loading_status = load_state::edurne_accept;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort\n"); // timeout is equal to not ready or yet
					loading_status = load_state::none;
					isACK = false;
				}
			}
			break;
		case load_state::edurne_accept:
			loading_status = load_state::edurne_start;
			reprap.GetPlatform().MessageF(Uart0_duet2, "M705 C%d\n",int(status[1]));
			reprap.GetPlatform().MessageF(Uart0_duet2, "M706 S1\n"); // Request Filament push until it receives a stop
			isFil = false;
			SetChangingFilamenACK(status[2], 1);
			isACK = false;
			timeout_timer = millis();
			break;
		case load_state::edurne_start:
			{
				if(millis() < timeout_timer + 40000){
					if(isFil){// btw we set manually the flag
						loading_status = load_state::printerwaitingfrs;
						isFil = false;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort\n"); // retry fil push
					loading_status = load_state::none;
					//loading_status = load_state::edurne_accept;
					//isACK = false;
				}
			}
			break;
		case load_state::printerwaitingfrs:
			loading_status = load_state::edurneprinterpushboth;
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1091 \n"); // Printer FRS detected, edurne stop pushing request
			isACK = false;
			timeout_timer = millis();
			break;
		case load_state::edurneprinterpushboth:
			{
				if(millis() < timeout_timer + 3000){
					if(isACK){//edurne stops
						reprap.GetGCodes().Exec_pushboth_f_Edurne();
						loading_status = load_state::ending;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort\n"); // S spool and P load
					loading_status = load_state::none;
				}
			}
			break;
		case load_state::ending:
			{
				if(!isBusy){

					loading_status = load_state::edurnetorest;
				}
			}
			break;
		case load_state::edurnetorest:
			reprap.GetPlatform().MessageF(HttpMessage, "Success, filament loaded\n");
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1095 S%d \n", int(status[1])); // Confirm Loading
			loading_status = load_state::none;
			break;
		/*case load_state::printerpush:
			break;
			*/
		default:
			status[0] = 0;
			isBusy = false;
			break;
	}

}
void FilamentHandler::unloadfsm(){

	switch(unloading_status){
		case unload_state::edurne_request:
			//reprap.GetPlatform().MessageF(HttpMessage, "edurne_request\n");
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1090 S%d P0\n",int(status[1])); // ASK to edurne if it can do the process
			unloading_status = unload_state::edurne_wait;
			isACK = false;
			timeout_timer = millis();
			break;
		case unload_state::edurne_wait:
			{
				if(millis() < timeout_timer + 2500){
					if(isACK){
						unloading_status = unload_state::edurne_accept;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort\n"); // timeout is equal to not ready or yet
					loading_status = load_state::none;
				}
			}
			break;
		case unload_state::edurne_accept:
			//reprap.GetPlatform().MessageF(HttpMessage, "edurne_accept\n");
			unloading_status = unload_state::printer_start;
			isBusy = true;
			reprap.GetGCodes().Exec_pushunloadalone_Edurne();
			break;
		case unload_state::printer_start:
			{
				if(!isBusy){
					//reprap.GetPlatform().MessageF(HttpMessage, "printer_start\n");
					unloading_status = unload_state::printer_sendtoedurne_end;
				}
			}
			break;
		case unload_state::printer_sendtoedurne_end:
			//reprap.GetPlatform().MessageF(HttpMessage, "printer_sendtoedurne_end\n");
			unloading_status = unload_state::edurnewaitingfrs;
			reprap.GetPlatform().MessageF(Uart0_duet2, "M705 C%d\n",int(status[1]));
			reprap.GetPlatform().MessageF(Uart0_duet2, "M706 S0\n"); // Request Filament push until it receives a stop // Request Filament push until it receives a stop
			isACK = false;
			timeout_timer = millis();
			break;
		case unload_state::edurnewaitingfrs:
			{
				if(millis() < timeout_timer + 40000){
					if(isACK){
						unloading_status = unload_state::edurnetorest;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort\n"); // timeout is equal to not ready or yet
					unloading_status = unload_state::none;
				}
			}
			break;
		case unload_state::edurnetorest:
			reprap.GetPlatform().MessageF(HttpMessage, "Success, filament unloaded\n");
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1096 S%d \n", int(status[1])); // Confirm Unloading
			unloading_status = unload_state::none;
			break;
		default:
			status[0] = 0;
			isBusy = false;
			break;
	}

}
void FilamentHandler::Request(uint8_t *rq){

	status[0] = rq[0];//process
	status[1] = rq[1];//spool requested
	status[2] = rq[2];//extruder destination
	if(status[0]==55){
		loading_status = load_state::edurne_request;
	}else if(status[0]==155){
		unloading_status = unload_state::edurne_request;
	}
	//reprap.GetPlatform().MessageF(HttpMessage, "recv request\n");
}
void FilamentHandler::Spin(){
	MutexLocker lock(FilamentHandlerMutex);
	if(!reprap.GetSpoolSupplier().Get_Master_Status()){//printer mode
		switch(status[0]){ // process request
			case 55: //go load
				loadfsm();
				break;
			case 155: //go unload
				unloadfsm();
				break;
			default:
				break;
		}
	}


}



