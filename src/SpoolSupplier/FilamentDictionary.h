/*
 * FilamentDictionary.h
 *
 *  Created on: 4 jun. 2019
 *      Author: agarciamoreno
 */

#ifndef SRC_SPOOLSUPPLIER_FILAMENTDICTIONARY_H_
#define SRC_SPOOLSUPPLIER_FILAMENTDICTIONARY_H_

#include <cstdint>

// Result codes returned by temperature sensor drivers
enum class FilamentDictionary : uint32_t
{
	PLA = 3508469,
	PVA = 77811945,
	defauld_filament = 0

};

const char* FilamentDictionaryString(FilamentDictionary err);


#endif /* SRC_SPOOLSUPPLIER_FILAMENTDICTIONARY_H_ */
