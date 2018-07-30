/*
 * Utility.h
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include "Arduino.h"

#ifdef __AVR_ATmega1284P__
  #define LED							15 // Moteino MEGAs have LEDs on digital pin 15
  #define FLASH_SS						23 // and FLASH SPI Slave Select on digital pin 23
#else
  #define LED							9 // Moteinos have LEDs on D9
  #define FLASH_SS						8 // and FLASH SS on D8
#endif

class Utility {
public:
	Utility();
	virtual ~Utility();
	void initialize(void);

	static void Blink( uint16_t flash_ms );
};

#endif /* UTILITY_H_ */
