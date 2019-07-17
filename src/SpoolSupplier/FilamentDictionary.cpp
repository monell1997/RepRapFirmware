/*
 * FilamentDictionary.cpp
 *
 *  Created on: 4 jun. 2019
 *      Author: agarciamoreno
 */

#include "FilamentDictionary.h"

const char* FilamentDictionaryString(FilamentDictionary name)
{
	switch(name)
	{
	case FilamentDictionary::PLA:					return "PLA";
	case FilamentDictionary::PVA:					return "PVA";
	case FilamentDictionary::PET_G:					return "PET_G";
	case FilamentDictionary::Nylon:					return "Nylon";
	case FilamentDictionary::ABS:					return "ABS";
	case FilamentDictionary::TPU:					return "TPU";
	default:										return "unknown filament";
	}
}
const float FilamentDictionaryTargetTemp(FilamentDictionary name)
{
	switch(name)
	{
	case FilamentDictionary::PLA:					return 45.0;
	case FilamentDictionary::PVA:					return 50.0;
	case FilamentDictionary::PET_G:					return 55.0;
	case FilamentDictionary::Nylon:					return 60.0;
	case FilamentDictionary::ABS:					return 50.0;
	case FilamentDictionary::TPU:					return 58.0;
	default:										return 0.0;
	}
}

// End
