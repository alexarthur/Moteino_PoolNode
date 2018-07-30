// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _AutoPool_H_
#define _AutoPool_H_
#include "Arduino.h"

//add your includes for the project AutoPool here

#include <RFM69.h>
#include <SPIFlash.h>

#include "Debug.h"

//end of add your includes here


//add your function definitions for the project AutoPool here

#ifdef __AVR_ATmega1284P__
  #define LED							15 // Moteino MEGAs have LEDs on digital pin 15
  #define FLASH_SS						23 // and FLASH SPI Slave Select on digital pin 23
#else
  #define LED							9 // Moteinos have LEDs on D9
  #define FLASH_SS						8 // and FLASH SS on D8
#endif

#define NODEID		3
#define GATEWAYID	1
#define NETWORKID	100
#define ENCRYPTKEY	"sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!

//Do not add code below this line
#endif /* _AutoPool_H_ */
