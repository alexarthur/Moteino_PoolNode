/*
 * FilterPressure.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#include "FilterPressure.h"
#include "Debug.h"

FilterPressure::FilterPressure( uint8_t sensor_pin ) {

	this->sensor_pin = sensor_pin;

	DEBUG( "Filter pressure sensor on pin " );
	DEBUG( (int) this->sensor_pin );
	DEBUGln();

	pinMode( this->sensor_pin, INPUT );     // Does this device require a pull-up
}

FilterPressure::~FilterPressure() {
	// TODO Auto-generated destructor stub
}

/*
 * FILTER_PRESSURE_SENSOR
 */

 // measures analog (% of Vin)
 int water_pressure_ticks;
 int water_pressure_10mPSI;		// pressure in 0.01 PSI units

 // ADC characteristics
 #define PSI_0_MV 500			// millivolts at 0.0 PSI
 #define PSI_30_MV 4500			// millivolts at 30.0 PSI
 #define ADC_RANGE 1024			// range of ADC
 #define ADC_MV 5000			// range of ADC in millivolts
 #define PSI_RANGE 3000 		// 0.01 PSI units

 // Normalize readings based on ADC characteristics
 #define PSI_0_TICKS 		(int)((float)PSI_0_MV  / ADC_MV * ADC_RANGE)
 #define PSI_30_TICKS		(int)((float)PSI_30_MV / ADC_MV * ADC_RANGE)
 #define PSI_RANGE_TICKS	(PSI_30_TICKS - PSI_0_TICKS)

 unsigned FilterPressure::getPSI() {
	 int psi = 0;
	 int ticks;

	 water_pressure_ticks = analogRead( sensor_pin );	// 0-1023
	 ticks = (water_pressure_ticks - PSI_0_TICKS);						// 0 PSI offset
	 water_pressure_10mPSI = (int)(((long)ticks * PSI_RANGE + PSI_RANGE_TICKS/2) / PSI_RANGE_TICKS);
	 return water_pressure_10mPSI / 100;								// PSI from (.01 * PSI).
 }
