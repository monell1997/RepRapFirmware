/*
 * FilamentHandler.h
 *
 *  Created on: 18 jun. 2019
 *      Author: agarciamoreno
 */

#ifndef SRC_TOOLS_FILAMENTHANDLER_H_
#define SRC_TOOLS_FILAMENTHANDLER_H_

#include "RepRap.h"
#include "SpoolSupplier/FilamentDictionary.h"

#define QUEUE_LEN_RQ 2

enum class load_state
	: uint8_t
	{
		none,
		edurne_request,				// request a filament to load
		edurne_wait,				// wait for edurne response
		edurne_accept,				// edurne is ready to load the filament requested
		edurne_start,				// edurne start load
		printerwaitingfrs, 		 	// printer detects that the filament arrives to the frs
		edurneprinterpushboth,		// printer extruder start to push slowly and edurne reduce the speed at same as the printer a fix distance
		ending,
		edurnetorest,				// edurne has done her job, then go to rest, printer continues purging
		/*printerpush					// printer push filament until ends
		*/
};
enum class unload_state
	: uint8_t
	{
		none,
		edurne_request,				// request a filament to unload
		edurne_wait,				// wait for edurne response
		edurne_accept,				// edurne is ready to unload the filament requested
		printer_start,				// printer start unload a fix distance until the filament is out of the extruder
		printer_sendtoedurne_end,	// printer notify to edurne that she can continue pushing
		edurnewaitingfrs, 		 	// printer detects that the filament arrives to the frs
		edurneprinterpushboth,
		edurnetorest				// edurne has done her job, then go to rest, printer continues purging

};
class FilamentHandler {
public:
	FilamentHandler();

	void Spin();

	FilamentDictionary ToolFilaments[MaxExtruders];
	void SetBusyState(int busy);
	void SetAckState(int ack);
	void Request(uint8_t *rq);
	void SetFilState(bool fil);
	bool isChangingFilamenACK(uint8_t extruder);
	void SetChangingFilamenACK(uint8_t extruder, uint8_t state);

private:
	uint32_t timeout_timer;
	static Mutex FilamentHandlerMutex;
	uint8_t queue_len;
	uint8_t status[4][QUEUE_LEN_RQ];				// status[0] = { 0 is nothing to do, 55 request a load, 155 request unload}
									// status[1] = { up to N_spools} starting with 1
									// status[2] = { Extruder destination
	void loadfsm();
	void unloadfsm();
	uint8_t isChangingFilamentACK[MaxExtruders];
	load_state loading_status;
	unload_state unloading_status;
	int isACK;				//Is an ACK after request
	bool isFil;				//Is Fil detected
	bool isBusy;			//Is Busy

	uint32_t lastTime;											// The last time
};



#endif /* SRC_TOOLS_FILAMENTHANDLER_H_ */
