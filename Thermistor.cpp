/*
 * Thermistor.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#include "Thermistor.h"
#include "Debug.h"

Thermistor::Thermistor( uint8_t sensor_pin ) {

	this->sensor_pin = sensor_pin;

	DEBUG( "Thermistor on pin " );
	DEBUG( (int) this->sensor_pin );
	DEBUGln();

	pinMode( sensor_pin, INPUT );

}

Thermistor::~Thermistor() {
}


double Thermistor::getCelsius() {

	uint32_t now = millis();

	float average;

	uint8_t	samples[THERMISTOR_SAMPLE_COUNT];

	uint8_t i;

	// take N samples in a row, with a slight delay
	for (i=0; i< THERMISTOR_SAMPLE_COUNT; i++) {
	   samples[i] = analogRead( sensor_pin );
	   delay( THERMISTOR_SAMPLE_INTERVAL_MILLIS / THERMISTOR_SAMPLE_COUNT );
	}

	// average all the samples out
	average = 0;
	for (i=0; i< THERMISTOR_SAMPLE_COUNT; i++) {
	  average += samples[i];
	}

	average /= THERMISTOR_SAMPLE_COUNT;

	// convert the value to resistance
	average = 1023 / average - 1;
	average = THERMISTOR_OHMS_OTHER / average;

	/*
	* You can easily calculate the resistance of NTC Thermistors at a given temperature using beta, but
	* there is an even more accurate way to do this using the Steinhart & Hart Equation.
	*/
	float steinhart;

	steinhart	=	average / THERMISTOR_OHMS_NOMINAL;			// (R/Ro)
	steinhart	=	log(steinhart);								// ln(R/Ro)
	steinhart	/=	THERMISTOR_BETA_COEFFICIENT;				// 1/B * ln(R/Ro)
	steinhart	+=	1.0 / (THERMISTOR_TEMPC_NOMINAL + 273.15);	// + (1/To)
	steinhart	=	1.0 / steinhart;							// Invert
	steinhart	-=	273.15;										// convert to C

	/*
	* Now, limit high and low values to reasonable values for the pool water (0*C - 38*C)
	*/
	float celsius;
	celsius = steinhart;

	if (celsius < 0.0) celsius = 0.0;	// Low temperature limited to 0*C (32*F)
	if (celsius > 38.0) celsius = 38.0;	// High temperature limited to 38*C (100.4*F)

//	DEBUG("Temperature ");
//	DEBUG( celsius );
//	DEBUG(" *C");
//  DEBUGln();

	return celsius;

 }
