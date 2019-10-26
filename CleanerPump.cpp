/*
 * CleanerPump.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#include "CleanerPump.h"
#include "Valve.h"
#include "Debug.h"

/////////////////////////////////////////////////////////////////////////////
////                                                                     ////
//// Using SainSMART Relay Module - logic active LOW                     ////
////                                                                     ////
////   - pin normally HIGH, pull pin to ground (LOW) to activate relay   ////
////                                                                     ////
/////////////////////////////////////////////////////////////////////////////

CleanerPump::CleanerPump( FilterPump *filterPump, FlowSensor *flowSensor, Valve *cleanerValve, uint8_t relay_pin ) {

	assert( filterPump != NULL );

	this->filterPump   = filterPump;
	this->flowSensor   = flowSensor;
	this->cleanerValve = cleanerValve;

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

	int  cleanerValveIsOpen  = ( cleanerValve->getValveState() == ValveState::ValveOpen );
	int  filterPumpIsRunning = ( filterPump->getRunState() == RunState::Running );
	int canRun = ( filterPumpIsRunning && cleanerValveIsOpen );

	return canRun;

 }

 RunState CleanerPump::getRunState() {
	 return this->state;
 }

 RunState CleanerPump::setRunState( RunState desiredState ) {

	 if ( canRun() && desiredState == RunState::Running ) {
			digitalWrite( relay_pin, LOW );
			this->state = RunState::Running;
	 } else {
			digitalWrite( relay_pin, HIGH );
			if ( !canRun() ) {
				this->state = RunState::Disabled;
			} else {
				this->state = RunState::Stopped;
			}
	 }
	 return this->state;
 }

 RunState CleanerPump::toggleRunState() {

	 return setRunState( getRunState() == Running ? Stopped : Running);

 }

 unsigned CleanerPump::getFlow_MLPS() {

	 return flowSensor->getFlow_MLPS();

 }
