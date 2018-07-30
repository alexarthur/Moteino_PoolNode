/*
 * PoolLamp.cpp
 *
 *  Created on: Jul 6, 2018
 *      Author: artal03
 */

#include "PoolLamp.h"
#include "Debug.h"

/////////////////////////////////////////////////////////////////////////////
////                                                                     ////
//// Using SainSMART Relay Module - logic active LOW                     ////
////                                                                     ////
////   - pin normally HIGH, pull pin to ground (LOW) to activate relay   ////
////                                                                     ////
/////////////////////////////////////////////////////////////////////////////

PoolLamp::PoolLamp( uint8_t lamp_pin ) {

	this->relay_pin = lamp_pin;

	DEBUG( "PoolLamp on pin " );
	DEBUG( (int) this->relay_pin );
	DEBUGln();

	pinMode( this->relay_pin, OUTPUT );

	setLampState( Off );

}

PoolLamp::~PoolLamp() {
	// TODO Auto-generated destructor stub
}

LampState PoolLamp::getLampState() {

	return ( digitalRead( relay_pin ) == LOW ) ? On : Off;

}

LampState PoolLamp::setLampState( LampState desiredState ) {

	digitalWrite( relay_pin, ( desiredState == On ) ? LOW : HIGH );

	return getLampState();
}

LampState PoolLamp::toggleLampState() {

	 digitalWrite( relay_pin, !digitalRead( relay_pin ) );

	 return getLampState();
}
