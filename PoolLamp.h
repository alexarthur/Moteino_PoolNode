/*
 * PoolLamp.h
 *
 *  Created on: Jul 6, 2018
 *      Author: artal03
 */

#ifndef POOLLAMP_H_
#define POOLLAMP_H_

#include <Arduino.h>

typedef enum { Off=0, On=1 } LampState;

class PoolLamp {

private:
	uint8_t		relay_pin;

public:
	PoolLamp( uint8_t lamp_pin );
	virtual ~PoolLamp();

	LampState getLampState();
	LampState setLampState( LampState desiredState );
	LampState toggleLampState();

};

#endif /* POOLLAMP_H_ */
