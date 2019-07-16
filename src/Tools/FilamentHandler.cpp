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

constexpr uint32_t time_between_requests = 6000;		// time between request


size_t current_pos_queue = 0;
size_t write_pos_queue = 0;


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
	lastTime = millis();
	isACK = false;
	FilamentHandlerMutex.Create("FilamentHandler");
}
void FilamentHandler::SetBusyState(int busy){

	isBusy = busy;

}
void FilamentHandler::SetAckState(int ack){

	isACK = ack;

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
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1090 S%d P1\n",int(status[1][current_pos_queue])); // ASK to edurne if it can do the process
			loading_status = load_state::edurne_wait;
			isACK = false;
			timeout_timer = millis();
			break;
		case load_state::edurne_wait:
			{
				if(millis() < timeout_timer + 2500){
					if(isACK == 1){
						loading_status = load_state::edurne_accept;
					}else if(isACK == 2){
						reprap.GetPlatform().MessageF(HttpMessage, "Request denied, Edurne is busy\n");
						reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::failed);
						loading_status = load_state::none;
						isACK = 0;

					}else if(isACK == 3){
						reprap.GetPlatform().MessageF(HttpMessage, "Request denied, Spool is not ready\n");
						reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::failed);
						loading_status = load_state::none;
						isACK = 0;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort\n"); // timeout is equal to not ready or yet
					loading_status = load_state::none;
					isACK = 0;
				}
			}
			break;
		case load_state::edurne_accept:
			loading_status = load_state::edurne_start;
			reprap.GetPlatform().MessageF(Uart0_duet2, "M705 C%d\n",int(status[1][current_pos_queue]));
			reprap.GetPlatform().MessageF(Uart0_duet2, "M706 S1\n"); // Request Filament push until it receives a stop
			isFil = false;
			SetChangingFilamenACK(status[2][current_pos_queue], 1);
			isACK = 0;
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
					reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::failed);
					loading_status = load_state::none;
					//loading_status = load_state::edurne_accept;
					//isACK = false;
				}
			}
			break;
		case load_state::printerwaitingfrs:
			loading_status = load_state::edurneprinterpushboth;
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1091 \n"); // Printer FRS detected, edurne stop pushing request
			isACK = 0;
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
					reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::failed);
					loading_status = load_state::none;
				}
			}
			break;
		case load_state::ending:// cuidao
			{
				if(!isBusy){

					loading_status = load_state::edurnetorest;
				}
			}
			break;
		case load_state::edurnetorest:
			reprap.GetPlatform().MessageF(HttpMessage, "Success, filament loaded\n");
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1095 S%d \n", int(status[1][current_pos_queue])); // Confirm Loading
			loading_status = load_state::none;

			reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::ok);

			if(status[3][current_pos_queue] == 100){//resume after request pls
				reprap.GetGCodes().autoresume_Edurne();
				status[3][current_pos_queue] = 0;
			}

			break;
		/*case load_state::printerpush:
			break;
			*/
		default:
			status[0][current_pos_queue] = 0;
			isBusy = false;
			queue_len = (queue_len-1);
			current_pos_queue = (current_pos_queue + 1)%QUEUE_LEN_RQ;
			lastTime = millis();
			break;
	}

}
void FilamentHandler::unloadfsm(){

	switch(unloading_status){
		case unload_state::edurne_request:
			reprap.GetPlatform().MessageF(HttpMessage, "enqueuing unload\n"); // ASK to edurne if it can do the process
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1090 S%d P0\n",int(status[1][current_pos_queue])); // ASK to edurne if it can do the process
			unloading_status = unload_state::edurne_wait;
			isACK = 0;
			timeout_timer = millis();
			break;
		case unload_state::edurne_wait:
			{
				if(millis() < timeout_timer + 2500){
					if(isACK == 1){
						unloading_status = unload_state::edurne_accept;
					}else if(isACK == 2){
						reprap.GetPlatform().MessageF(HttpMessage, "Request denied, Edurne is busy\n");
						reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::failed);
						unloading_status = unload_state::none;
						isACK = 0;

					}else if(isACK == 3){
						reprap.GetPlatform().MessageF(HttpMessage, "Request denied, Spool is not ready\n");
						reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::failed);
						unloading_status = unload_state::none;
						isACK = 0;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort\n"); // timeout is equal to not ready or yet
					loading_status = load_state::none;
				}
			}
			break;
		case unload_state::edurne_accept:
			reprap.GetPlatform().MessageF(HttpMessage, "edurne_accept\n");
			unloading_status = unload_state::printer_start;
			isBusy = true;
			reprap.GetGCodes().Exec_pushunloadalone_Edurne();
			break;
		case unload_state::printer_start://cuidao
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
			reprap.GetPlatform().MessageF(Uart0_duet2, "M705 C%d\n",int(status[1][current_pos_queue]));
			reprap.GetPlatform().MessageF(Uart0_duet2, "M706 S0\n"); // Request Filament push until it receives a stop // Request Filament push until it receives a stop
			isACK = 0;
			timeout_timer = millis();
			break;
		case unload_state::edurnewaitingfrs:
			{
				if(millis() < timeout_timer + 40000){
					if(isACK == 1){
						unloading_status = unload_state::edurnetorest;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort\n"); // timeout is equal to not ready or yet
					reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::failed);
					unloading_status = unload_state::none;
					isACK = 0;
				}
			}
			break;
		case unload_state::edurnetorest:
			reprap.GetPlatform().MessageF(HttpMessage, "Success, filament unloaded\n");
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1096 S%d \n", int(status[1][current_pos_queue])); // Confirm Unloading
			reprap.GetSpoolSupplier().Set_Loaded_flag((size_t)status[1][current_pos_queue], 0);// Immediate update
			reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) status[1][current_pos_queue], ChangeFilStatus::ok);
			unloading_status = unload_state::none;
			break;
		default://ending
			status[0][current_pos_queue] = 0;
			isBusy = false;
			queue_len = (queue_len-1);
			current_pos_queue = (current_pos_queue + 1)%QUEUE_LEN_RQ;
			lastTime = millis();
			break;
	}

}
void FilamentHandler::Request(uint8_t *rq){

	if(queue_len<QUEUE_LEN_RQ){

		status[0][write_pos_queue] = rq[0];//process
		status[1][write_pos_queue] = rq[1];//spool requested
		status[2][write_pos_queue] = rq[2];//extruder destination
		status[3][write_pos_queue] = rq[3];//resume print_after success flag
		if(status[0][write_pos_queue]==55){
			loading_status = load_state::edurne_request;
			reprap.GetPlatform().MessageF(HttpMessage, "recv load request\n");
		}else if(status[0][write_pos_queue]==155){
			unloading_status = unload_state::edurne_request;
			reprap.GetPlatform().MessageF(HttpMessage, "recv unload request\n");
		}
		reprap.GetSpoolSupplier().Set_Change_Fil_Status((size_t) rq[1], ChangeFilStatus::requested);


		write_pos_queue = (write_pos_queue + 1)%QUEUE_LEN_RQ;
		queue_len += 1;
	}else{
		reprap.GetPlatform().MessageF(HttpMessage, "queue buffer is full \n");
	}


}
void FilamentHandler::Spin(){
	MutexLocker lock(FilamentHandlerMutex);
	if(!reprap.GetSpoolSupplier().Get_Master_Status()){//printer mode

		if(queue_len > 0){
			const uint32_t now = millis();
			if (now - lastTime >= time_between_requests)
			{
				switch(status[0][current_pos_queue]){ // process request
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
	}


}

//buflen = (buflen-1);
//bufindr = (bufindr + 1)%BUFSIZE;

