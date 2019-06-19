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

FilamentHandler::FilamentHandler() {
	const size_t numExtruders = reprap.GetGCodes().GetNumExtruders();
	for (size_t i = 0; i<numExtruders;i++)
	{
		ToolFilaments[i] = FilamentDictionary::defauld_filament;
	}
	timeout_timer = millis();
	isACK = false;

}
void FilamentHandler::loadfsm(){

	switch(loading_status){
		case load_state::edurne_request:
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
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort"); // timeout is equal to not ready or yet
					loading_status = load_state::none;
					isACK = false;
				}
			}
			break;
		case load_state::edurne_accept:
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1080 S%d", int(status[1])); // Request Filament push until it receives a stop
			isACK = false;
			timeout_timer = millis();
			break;
		case load_state::edurne_start:
			{
				if(millis() < timeout_timer + 3000){
					if(isACK){
						loading_status = load_state::printerwaitingfrs;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort"); // retry fil push
					loading_status = load_state::none;
					//loading_status = load_state::edurne_accept;
					//isACK = false;
				}
			}
			break;
		case load_state::printerwaitingfrs:
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1091 \n"); // Printer FRS detected, edurne stop pushing request
			isACK = false;
			timeout_timer = millis();
			break;
		case load_state::edurneprinterpushboth:
			{
				if(millis() < timeout_timer + 3000){
					if(isACK){
						reprap.GetGCodes().Exec_pushboth_Edurne();
						loading_status = load_state::ending;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort"); // S spool and P load
					loading_status = load_state::none;
				}
			}
			break;
		case load_state::ending:
			{
				if(!isBusy){

					loading_status = load_state::none;
				}
			}
			break;
		/*case load_state::edurnetorest:
			break;
		case load_state::printerpush:
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
			isBusy = true;
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
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort"); // timeout is equal to not ready or yet
					loading_status = load_state::none;
				}
			}
			break;
		case unload_state::edurne_accept:
			isBusy = true;
			reprap.GetGCodes().Exec_pushunloadalone_Edurne();
			break;
		case unload_state::printer_start:
			{
				if(!isBusy){

					unloading_status = unload_state::printer_sendtoedurne_end;
				}
			}
			break;
		case unload_state::printer_sendtoedurne_end:
			reprap.GetPlatform().MessageF(Uart0_duet2, "M1081 S%d", int(status[1])); // Request Filament push until it receives a stop
			isACK = false;
			timeout_timer = millis();
			break;
		case unload_state::edurnewaitingfrs:
			{
				if(millis() < timeout_timer + 2500){
					if(isACK){
						unloading_status = unload_state::none;
					}
				}else{
					reprap.GetPlatform().MessageF(HttpMessage, "ACK confirmation not received from edurne, Abort"); // timeout is equal to not ready or yet
					unloading_status = unload_state::none;
				}
			}
			break;
		case unload_state::edurnetorest:
			break;
		default:
			status[0] = 0;
			isBusy = false;
			break;
	}

}
void FilamentHandler::Spin(){

	switch(status[0]){
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



