/*
 * Debug.h
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <Arduino.h>

#define SERIAL_EN  //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define SERIAL_BAUD   9600
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif


#endif /* DEBUG_H_ */
