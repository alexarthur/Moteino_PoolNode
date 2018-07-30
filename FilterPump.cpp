/*
 * FilterPump.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#include "FilterPump.h"
#include "Debug.h"

/////////////////////////////////////////////////////////////////////////////
////                                                                     ////
//// Using SainSMART Relay Module - logic active LOW                     ////
////                                                                     ////
////   - pin normally HIGH, pull pin to ground (LOW) to activate relay   ////
////                                                                     ////
/////////////////////////////////////////////////////////////////////////////

FilterPump::FilterPump( uint8_t relay_pin ) {

	this->relay_pin = relay_pin;

	DEBUG( "Filter pump relay on pin " );
	DEBUG( (int) this->relay_pin );
	DEBUGln();

	pinMode( this->relay_pin, OUTPUT );
	setRunState( Stopped );
}

FilterPump::~FilterPump() {
	// TODO Auto-generated destructor stub
}

 RunState FilterPump::getRunState() {
	 return digitalRead( relay_pin ) == LOW ? Running : Stopped;
}

 RunState FilterPump::setRunState( RunState desiredState ) {
	 digitalWrite( relay_pin, ( desiredState == Running ) ? LOW : HIGH );
	 return getRunState();
 }

 RunState FilterPump::toggleRunState() {
	 digitalWrite( relay_pin, !digitalRead( relay_pin ) );
	 return getRunState();
 }
