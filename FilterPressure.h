/*
 * FilterPressure.h
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#ifndef FILTERPRESSURE_H_
#define FILTERPRESSURE_H_

#include <Arduino.h>

class FilterPressure {
private:
	uint8_t sensor_pin;

public:
	FilterPressure( uint8_t sensor_pin );
	virtual ~FilterPressure();

	unsigned getPSI();
};

#endif /* FILTERPRESSURE_H_ */
