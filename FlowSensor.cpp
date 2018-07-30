/*
 * FlowSensor.cpp - Measure liquid flow rate using a flow sensor producing pulses
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 *
 *  Based on "Liquid flow rate sensor -DIYhacking.com" by Arvind Sanjeev
 *
 *  Connect Vcc and Gnd of sensor to ?Volt power source
 *  Connect signal line to a digital pin supporting external interrupts (i.e. digital pin 2 or 3 on Arduino Uno/Moteino)
 *
 */

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////     W A T E R  F L O W  S E N S O R    ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "FlowSensor.h"

//byte sensorPin       = 3;  // Digital pin 3 is used since RFM69 radio on Moteino is already using pin 2 (INT0)
//byte sensorInterrupt = 1;  // INT1 is on digital pin 3

#define		DFLT_CALIBRATION_FACTOR 3.9

FlowSensor::FlowSensor( uint8_t sensor_pin, uint8_t ext_interrupt ) {
	this->sensor_pin = sensor_pin;

	DEBUG( "Flow sensor on pin " );
	DEBUG( (int) this->sensor_pin );
	DEBUGln();

	pinMode( this->sensor_pin,	INPUT_PULLUP );

	digitalWrite( this->sensor_pin, HIGH );

	this->ext_interrupt = ext_interrupt;

	this->calibrationFactor = DFLT_CALIBRATION_FACTOR;

	enableInterrupt();
}

FlowSensor::~FlowSensor() {
	// TODO Auto-generated destructor stub
}

volatile uint32_t pulseCount;

void FlowSensor::pulseCounter(void) {
	pulseCount++;	// Increment the pulse counter
}

void FlowSensor::enableInterrupt(void) {
	// The Hall-effect sensor is connected to pin 3 which uses interrupt 1.
	// Configured to trigger on a FALLING state change (transition from HIGH
	// state to LOW state)
	prevSampleTime = millis();
	attachInterrupt( ext_interrupt, FlowSensor::pulseCounter, FALLING);
}

void FlowSensor::disableInterrupt() {
	detachInterrupt( ext_interrupt );
}

// From amazon description:
// Instantaneous Flow Pulse Characteristic F=4.0x Q(flow) +/-10%
// The cumulative flow pulse conversion ratio 1L Water=234 pulse +/- 10%
//
// The hall-effect flow sensor outputs approximately 3.9 pulses per second per
// litre/minute of flow ( DFLT_CALIBRATION_FACTOR )
//


unsigned FlowSensor::getFlow_MLPS(void) {

	unsigned flowMLPS = 0;

	uint32_t now = millis();

	disableInterrupt();

//#ifdef SERIAL_EN
//	char buf[30];
//	sprintf(buf, "pulseCount=%d", pulseCount);
//	DEBUG( buf );
//	DEBUGln();
//#endif

	// Count the number of pulses accumulated since the last time we took a sample.
	// Based on the calibrationFactor (~3.9 pulses per second, for each litre/min),
	// calculate the flow rate in Litres Per Minute, based on the number of pulses
	// per second per units of measure (litres/minute in this case) coming from the sensor.
	float flowLPS = ((1000.0 / (now - prevSampleTime)) * pulseCount) / calibrationFactor;

	// Note the time this processing pass was executed. Note that because we've
	// disabled interrupts the millis() function won't actually be incrementing right
	// at this point, but it will still return the value it was set to just before
	// interrupts went away.
	prevSampleTime = now;

	pulseCount = 0; // reset pulse counter to start again when interrupts are enabled

	enableInterrupt();

	// Divide the flow rate in litres/minute by 60 to determine how many litres have
	// passed through the sensor in this 1 second interval, then multiply by 1000 to
	// convert to millilitres.
	flowMLPS = (flowLPS / 60) * 1000;

	return flowMLPS;

}

uint32_t FlowSensor::getPulseCount() {
	return pulseCount;
}
