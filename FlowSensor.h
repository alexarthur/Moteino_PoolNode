/*
 * FlowSensor.h
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#ifndef FLOWSENSOR_H_
#define FLOWSENSOR_H_

#include <Arduino.h>
#include "Debug.h"

class FlowSensor {
private:
					uint8_t		sensor_pin;
					uint8_t		ext_interrupt;
					uint32_t	prevSampleTime;
					float		calibrationFactor;
	static			void		pulseCounter(void);
					void		enableInterrupt(void);
					void		disableInterrupt(void);

public:
					FlowSensor( uint8_t sensor_pin, uint8_t ext_interrupt );
	virtual			~FlowSensor();

					unsigned	getFlow_MLPS(void);
					uint32_t	getPulseCount(void);

};

#endif /* FLOWSENSOR_H_ */
