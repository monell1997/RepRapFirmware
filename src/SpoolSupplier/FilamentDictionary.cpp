/*
 * FilamentDictionary.cpp
 *
 *  Created on: 4 jun. 2019
 *      Author: agarciamoreno
 */

#include "FilamentDictionary.h"

const char* FilamentDictionaryString(FilamentDictionary err)
{
	switch(err)
	{
	case FilamentDictionary::PLA:					return "PLA";
	case FilamentDictionary::PVA:					return "PVA";
	default:										return "unknown filament";
	}
}

// End
