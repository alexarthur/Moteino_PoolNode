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

#define THERMISTOR_SAMPLE_COUNT 1<<5 // 32 samples



float Thermistor::getCelsius() {

	int sum=0, V_avg=0, resistance=0;

	// take N samples in a row, with a slight delay
	for (int i=0; i< THERMISTOR_SAMPLE_COUNT; i++) {
	   sum += analogRead( sensor_pin );
	   delay( 10 );
	}

	// average all the samples out
	// Since the sample count is a multiple of 2, we can shift right to divide
	V_avg = sum >> 5;
	DEBUG("V_avg: "); DEBUGln(V_avg);

	// convert the value to resistance
	resistance = (1023 / V_avg) - 1;
	resistance = THERMISTOR_OHMS_OTHER / resistance;
	DEBUG("resistance:"); DEBUGln((int)resistance);

	float R1 = 10000;
	float logR2, R2, Tk, Tc;
	float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

	R2 = R1 * (1023.0 / (float)V_avg - 1.0);
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
