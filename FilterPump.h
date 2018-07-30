/*
 * FilterPump.h
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#ifndef FILTERPUMP_H_
#define FILTERPUMP_H_

#include <Arduino.h>
#include "RunState.h"

class FilterPump {
private:
	uint8_t		relay_pin;

public:
	FilterPump( uint8_t relay_pin );
	virtual ~FilterPump();

	RunState getRunState();
	RunState setRunState( RunState desiredState );
	RunState toggleRunState();
};

#endif /* FILTERPUMP_H_ */
