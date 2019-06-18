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

}




