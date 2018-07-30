/*
 * Utility.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#include "Utility.h"

Utility::Utility() {
	// TODO Auto-generated constructor stub

}

Utility::~Utility() {
	// TODO Auto-generated destructor stub
}

void Utility::initialize() {

}

void Utility::Blink( uint16_t flash_ms ) // flash_ms = time in milliseconds to illuminate LED
{
  pinMode( LED, OUTPUT );

  digitalWrite( LED, HIGH );

  delay( flash_ms );

  digitalWrite( LED, LOW );
}
