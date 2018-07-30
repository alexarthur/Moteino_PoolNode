/*
 * CleanerPump.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#include "CleanerPump.h"
#include "Debug.h"

/////////////////////////////////////////////////////////////////////////////
////                                                                     ////
//// Using SainSMART Relay Module - logic active LOW                     ////
////                                                                     ////
////   - pin normally HIGH, pull pin to ground (LOW) to activate relay   ////
////                                                                     ////
/////////////////////////////////////////////////////////////////////////////

CleanerPump::CleanerPump( FilterPump *filterPump, FlowSensor *flowSensor, uint8_t relay_pin ) {

	assert( filterPump != NULL );

	this->filterPump = filterPump;
	this->flowSensor = flowSensor;

	this->relay_pin = relay_pin;

	DEBUG( "Cleaner pump relay on pin " );
	DEBUG( (int) this->relay_pin );
	DEBUGln();

 	pinMode( this->relay_pin,	OUTPUT );

 	setRunState( Stopped );

}

CleanerPump::~CleanerPump() {
}

/*
 * Cleaner pump is a booster pump and can only run when the main filter pump
 * is active and there is sufficient water flowing through the pump circuit.
 */
bool CleanerPump::canRun() {

	/*
	 * CleanerPump can only run if...
	 * 1. Filter pump is running AND there is water flowing through the cleaner pump
	 */
	return (  ( filterPump->getRunState() == Running )
		&&	( flowSensor->getFlow_MLPS() >= CLEANER_PUMP_FLOW_MLPS_MINIMUM ) );

 }

 RunState CleanerPump::getRunState() {

   return ( digitalRead( relay_pin ) == LOW ? Running : Stopped );

 }

 RunState CleanerPump::setRunState( RunState desiredState ) {

	 switch ( desiredState ) {

		case Running:
			digitalWrite( relay_pin, ( canRun() ? LOW : HIGH ) ); // Permit Run ONLY if canRun()
			break;

		case Stopped:
		default:
			digitalWrite( relay_pin, HIGH );
			break;

	 }

	 return getRunState();

 }

 RunState CleanerPump::toggleRunState() {

	 return setRunState( getRunState() == Running ? Stopped : Running);

 }

 unsigned CleanerPump::getFlow_MLPS() {

	 return flowSensor->getFlow_MLPS();

 }
