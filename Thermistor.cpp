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


float Thermistor::getCelsius() {

	float average, resistance;

	uint16_t samples[THERMISTOR_SAMPLE_COUNT];

	// take N samples in a row, with a slight delay
	for (int i=0; i< THERMISTOR_SAMPLE_COUNT; i++) {
	   samples[i] = analogRead( sensor_pin );
	   DEBUG("sample ");DEBUG(i);DEBUG(":"); DEBUGln(samples[i]);
	   delay( THERMISTOR_SAMPLE_INTERVAL_MILLIS / THERMISTOR_SAMPLE_COUNT );
	}

	// average all the samples out
	average = 0;
	for (int i=0; i< THERMISTOR_SAMPLE_COUNT; i++) {
	  average += samples[i];
	}

	average /= THERMISTOR_SAMPLE_COUNT;
	DEBUG("average1:"); DEBUGln((int)average);

	// convert the value to resistance
	resistance = (1023 / average) - 1;
	resistance = THERMISTOR_OHMS_OTHER / resistance;
	DEBUG("resistance:"); DEBUGln((int)resistance);

	int Vo = average;

	float R1 = 10000;
	float logR2, R2, Tk, Tc;
	float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

	Vo = analogRead(sensor_pin);
	DEBUG("Analog pin: "); DEBUGln(Vo);
	R2 = R1 * (1023.0 / (float)Vo - 1.0);
	logR2 = log(R2);
	Tk = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
	Tc = Tk - 273.15;

	DEBUG("Tc: "); DEBUGln(Tc);

	return Tc;

 }

float Thermistor::getFahrenheit() {
	float Tc, Tf;
	Tc = getCelsius();
	Tf = (Tc * 9.0)/ 5.0 + 32.0;
	DEBUG("Tf: "); DEBUGln(Tf);
	return Tf;
}
