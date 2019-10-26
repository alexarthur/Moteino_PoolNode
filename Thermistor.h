/*
 * Thermistor.h
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include <Arduino.h>

// measures analog (5%-95% of Vin)
class Thermistor {
private:
	// resistance at 25 degrees C
	const unsigned int THERMISTOR_OHMS_NOMINAL = 52600;

	// temp. for nominal resistance (almost always 25 C)
	const float THERMISTOR_TEMPC_NOMINAL = 23.6;

	// how many samples to take and average, more takes longer
	// but is more 'smooth'
	//const unsigned int THERMISTOR_SAMPLE_COUNT = 32;

	// The beta coefficient of the thermistor (usually 3000-4000)
	const unsigned int THERMISTOR_BETA_COEFFICIENT = 3950;

	// the value of the 'other' resistor
	const unsigned int THERMISTOR_OHMS_OTHER = 10000;

	// time between refreshes
	//const unsigned int THERMISTOR_SAMPLE_INTERVAL_MILLIS = 1000;

	uint8_t		sensor_pin;

public:

	 	 	 Thermistor( uint8_t sensor_pin );
	virtual ~Thermistor();

	float	getCelsius();
	float   getFahrenheit();
};

#endif /* THERMISTOR_H_ */
