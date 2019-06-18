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

class FilamentHandler {
public:
	FilamentHandler();

	FilamentDictionary ToolFilaments[MaxExtruders];

private:

};



#endif /* SRC_TOOLS_FILAMENTHANDLER_H_ */
