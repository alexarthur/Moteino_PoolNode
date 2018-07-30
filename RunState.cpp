/*
 * RunState.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */
#include "RunState.h"

char* getRunStateLabel( RunState runState ) {

	char *label = "Unknown";

	switch ( runState ) {
		case Stopped:
			label = "Stopped";
			break;
    	case Running:
    		label = "Running";
    		break;
    	default:
    		label = "Unknown";
	}

	return label;
}




