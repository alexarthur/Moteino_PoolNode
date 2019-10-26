/*
 * CleanerPump.h
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#ifndef CLEANERPUMP_H_
#define CLEANERPUMP_H_

#include <Arduino.h>
#include "RunState.h"
#include "FilterPump.h"
#include "FlowSensor.h"
#include "Valve.h"
#include "assert.h"

#define CLEANER_PUMP_FLOW_MLPS_MINIMUM 50

class CleanerPump {
private:
	FilterPump	*filterPump;
	FlowSensor	*flowSensor;
	Valve		*cleanerValve;
	RunState	state;
	uint8_t		relay_pin;

public:
	CleanerPump( FilterPump *filterPump, FlowSensor *flowSensor, Valve *cleanerValve, uint8_t relay_pin );
	virtual ~CleanerPump();

	RunState	getRunState();
	RunState	setRunState( RunState desiredState );
	RunState	toggleRunState();
	bool		canRun();
	unsigned	getFlow_MLPS(void);
};

#endif /* CLEANERPUMP_H_ */
