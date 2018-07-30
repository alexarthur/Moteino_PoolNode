// Do not remove the include below
#include "AutoPool.h"
#include "Radio.h"
#include "Utility.h"
#include "FilterPump.h"
#include "CleanerPump.h"
#include "Thermistor.h"
#include "FilterPressure.h"
#include "PoolLamp.h"

/*
 * Moteino PIN inventory
 *
 * Key:
 * 		~	PWM Capable via analogWrite()
 *
 * == Arduino PORT B, pin change interrupt 0 ==
 *
 * PB0: D8            		 	- [SPI SS for Flash Memory]
 * PB1: D9~  					- [LED]
 * PB2: D10~ 					- [SPI SS for Radio]
 * PB3: D11~ MOSI (SPI)			- [SPI used with SS pins for multiple devices]
 * PB4: D12  MISO (SPI)			- [SPI used with SS pins for multiple devices]
 * PB5: D13  SCK  (SPI)			- [SPI used with SS pins for multiple devices]
 *
 * == Arduino PORT C, pin change interrupt 1  ==
 *
 * PC0: A0 / D14 				- << unused >>
 * PC1: A1 / D15 				- << unused >>
 * PC2: A2 / D16				- << unused >>
 * PC3: A3 / D17 				- << unused >>
 * PC4: A4 / D18 / SDA (I2C)	- << unused >>
 * PC5: A5 / D19 / SCL (I2C) 	- << unused >>
 * PC6: A6 (Analog ONLY)		- WATER_TEMP_SENSOR_PIN (thermistor)
 * PC7: A7 (Analog ONLY)		- FILTER_PRESSURE_SENSOR_PIN
 *
 * == Arduino PORT D (digital only), pin change interrupt 2 ==
 *
 * PD0: D0   RX  				- Serial RCV
 * PD1: D1   TX 				- Serial XMIT
 * PD2: D2   INT0 (Hardware)	- [Radio]
 * PD3: D3~  INT1 (Hardware)	- CLEANER_FLOW_PULSE_PIN
 * PD4: D4   PCINT20			- FILTER_PUMP_RELAY_PIN
 * PD5: D5~  PCINT21			- CLEANER_PUMP_RELAY_PIN
 * PD6: D6~  PCINT22			- POOL_LAMP_RELAY_PIN
 * PD7: D7   PCINT23			-
 *
 */

/*
 * Interfaces (pins)
 *
 *  Water temperature thermistor Sensor - requires analog pin.
 *  Filter pump relay control - requires digital pin, normally low.
 *  Filter pump pressure sensor - requires analog pin.
 *  Pool cleaner pump relay control - requires digital pin, normally low.
 *
 */

#define WATER_TEMP_SENSOR_PIN			A6	// Water temperature thermistor on analog pin 6 (INPUT|PULLUP)
#define FILTER_PRESSURE_SENSOR_PIN		A7	// Filter pressure transducer on analog pin 7 (INPUT)

#define CLEANER_FLOW_SENSOR_PIN			3	// Water flow through cleaner ext. int 1 (INT1) count pulses on digital pin 3 (INPUT)
#define FILTER_PUMP_RELAY_PIN			4	// FILTER PUMP RELAY on digital pin 5 (OUTPUT|LOW)
#define CLEANER_PUMP_RELAY_PIN			5	// CLEANER_PUMP_RELAY on digital pin 7 (OUTPUT|LOW)
#define POOL_LAMP_RELAY_PIN				6	// POOL_LAMP_RELAY on digital pin 17 (A3) - (OUTPUT|LOW)
#define FILTER_PUMP_TOGGLE_PIN			X	// FILTER PUMP TOGGLE on digital pin 4 (INPUT|PULLUP)
#define CLEANER_PUMP_TOGGLE_PIN			X	// CLEANER_PUMP_TOGGLE on digital pin 6 (INPUT|PULLUP)

/*
 * Interrupts
 *
 * D4 - PCINT20
 * D6 - PCINT22
 */

/*
 * flash(SPI_SS, MANUFACTURER_ID)
 * SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
 * MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
 *                             0xEF30 for Winbond 4mbit flash
 */
SPIFlash flash(FLASH_SS, 0xEF30); // Winbond Flash on Moteino (0xEF30)

 Radio			*radio				= NULL;  // Radio used to communicate with gateway
 FilterPump		*filterPump			= NULL;  // Pool Filter Pump
 FilterPressure	*filterPressure		= NULL;  // Pool Filter Pressure sensor
 Thermistor		*waterTemperature	= NULL;  // Pool Water Temperature sensor
 CleanerPump	*cleanerPump		= NULL;  // Pool Cleaner Pump
 FlowSensor		*cleanerFlow		= NULL;  // Pool Cleaner Flow sensor
 PoolLamp		*poolLamp			= NULL;  // Pool Lamp

 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////     F R O M   M E T R I C S . J S    //////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////
 //
 //   ////////////////////////
 //   /////    P O O L   /////
 //   ////////////////////////
 //   //
 //   // PTE:d+      (sent in Celsius units (x10) -- divide by 10 and convert to Fahrenheit)
 //   // PPR:d+      (Filter pressure sent in XXX units)
 //   // PFL:d+      (Cleaner flow sent in XXX units)
 //   // PPH:d+      (Water Ph sent in XXX units)
 //   // PCL:d+      (Chlorine level sent in XXX units)
 //   // PFR:d+      (Filter run state, sent as 0 (stopped) or 1 (running))
 //   // PCR:d+      (Cleaner run state, sent as 0 (stopped), 1 (running) or 2 (disabled))
 //   // PLI:d+      (Pool Lamp state, sent as 0 (off) or 1 (on))
 //   //
 //   PoolPump_OFF  : { name:'Pool Pump', regexp:/PFR\:0/i, value:'OFF' },
 //   PoolPump_ON   : { name:'Pool Pump', regexp:/PFR\:1/i, value:'ON' },
 //   VacPump_OFF   : { name:'Polaris Pump', regexp:/PCR\:0/i, value:'OFF' },
 //   VacPump_ON    : { name:'Polaris Pump', regexp:/PCR\:1/i, value:'ON' },
 //   VacPump_NA    : { name:'Polaris Pump', regexp:/PCR\:2/i, value:'DISABLED' },
 //   PoolLight_OFF : { name:'Pool Light', regexp:/PLI\:0/i, value:'OFF' },
 //   PoolLight_ON  : { name:'Pool Light', regexp:/PLI\:1/i, value:'ON' },
 //   poolTemp  : { name:'Pool Temp', regexp:/PTE\:(d+)/i, value:'', valuation:function(value) {return ((value/10)*9.0/5.0)+32.0;},  unit:'°F', pin:1, graph:1 },
 //   poolPress : { name:'Filter Pressure', regexp:/PPR\:(\d+)/i, value:'', valuation:function(value) {return Math.round((value*0.01)*10)/10.0;},  unit:'psi', pin:1, graph:1 },
 //   poolFlow  : { name:'Polaris Flow', regexp:/PFL\:(\d+)/i, value:'', valuation:function(value) {return (value)}, unit:'mL/sec', pin:1, graph:1 },
 //   poolPH    : { name:'Pool pH', regexp:/PPH\:(\d+)/i, value:'', valuation:function(value) {return (value)}, unit:'', pin:1, graph:1 },
 //   poolORP   : { name:'Pool ORP', regexp:/PCL\:(\d+)/i, value:'', valuation:function(value) {return (value)}, unit:'mV', pin:1, graph:1 },
 //

 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////     P O O L  T E M P E R A T U R E     ////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////

 void sendPoolTemp(void) {

     char txBuf[50];

     // The temperature value sent starts with the original floating point value multiplied by ten and
     // and then cast to an integer to strip the fractional part off (i.e. 22.3*C is sent as 223)
     sprintf( txBuf, "PTE:%d", (int)(waterTemperature->getCelsius()*10));

     radio->send(GATEWAYID, txBuf, strlen( txBuf ));

     Utility::Blink(3);
 }


 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////   F I L T E R   P U M P   S T A T U S  ////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////

 void sendFilterPumpRunState( uint8_t destNodeID = GATEWAYID ) {

     char txBuf[10];

     sprintf( txBuf, "PFR:%d", filterPump->getRunState() == RunState::Running ? 1 : 0);

     radio->send( destNodeID, txBuf, strlen(txBuf) );

     Utility::Blink(3);

 }

 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////      F I L T E R  P R E S S U R E      ////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////

 void sendFilterPSI( uint8_t destNodeID = GATEWAYID ) {

     char txBuf[10];

     sprintf( txBuf, "PPR:%d", filterPressure->getPSI() );

     radio->send(destNodeID, txBuf, strlen(txBuf) );

     Utility::Blink(3);
 }

 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////     C L E A N E R   F L O W  S E N S O R    ///////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////

 void sendCleanerPumpFlowRate( uint8_t destNodeID = GATEWAYID ) {

     char txBuf[10];

     sprintf( txBuf, "PFL:%d", cleanerFlow->getFlow_MLPS() );

     radio->send(destNodeID, txBuf, strlen( txBuf ));

     Utility::Blink(3);
 }

 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////   C L E A N E R   P U M P   S T A T U S  //////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////

 void sendCleanerPumpRunState( uint8_t destNodeID = GATEWAYID ) {

	char txBuf[10];

	RunState curRunState = cleanerPump->getRunState();

	int iRunState = 0;

	/*
	 * The gateway can be told to indicate that the cleaner is stopped, running or disabled
	 *
	 * Send...
	 * 0 : Stopped
	 * 1 : Running
	 * 2 : Disabled (cannot be run)
	 *
	 */
	switch ( curRunState ) {
		case Running:
			iRunState = 1;
			break;

		case Stopped:
			iRunState = ( cleanerPump->canRun() ? 0 : 2 );
			break;
     }

     sprintf( txBuf, "PCR:%d", iRunState);

     radio->send( destNodeID, txBuf, strlen( txBuf ) );

     Utility::Blink(3);

 }

 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////   P O O L   L A M P   S T A T U S   ///////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////

 void sendPoolLampState( uint8_t destNodeID = GATEWAYID ) {

     char txBuf[50];

     sprintf( txBuf, "PLI:%d", (poolLamp->getLampState() == On) ? 1 : 0 );

     radio->send(destNodeID, txBuf, strlen( txBuf ));

     Utility::Blink(3);

 }

 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////

 uint32_t	statusIntervalLast		= 0;		// Keeps track of the most recent transmit intervals that was processed.
 uint16_t	statusIntervalMillis	= 15*1000;	// How often (in milliseconds) should we transmit status.

 bool sentStartMessage = false;

 bool transmitStatus( void ) {

	bool statusSent = false;

   /*
    * Status is sent only once each interval, based on statusIntervalMillis
    */
	uint32_t intervalNow = millis() / statusIntervalMillis;

	/*
	* The "START" message is only sent once.  Doing so, leaves a lingering metric on the gateway that never gets updated.
	* We use to this determine how long this node has been running.
	*/
	if ( !sentStartMessage ) {

		statusSent = radio->send(GATEWAYID, "START", 5);
		sentStartMessage = true;

	} else if ( intervalNow > statusIntervalLast ) { // Only send status is the current interval is greater than the last one

		DEBUG("--== transmitStatus ==--");

		// Report current status
		char payload[Radio::MAX_PAYLOAD_LEN+1];	// Main payload buffer, the contents of which are always sent to GATEWAYID
		char secondary[30];						// Buffer used to format metrics that are concatenated to payload only sent filterPump is Running
		char msg[100];

		int filterRunState		= filterPump->getRunState();

		/*
		 * Cleaner run state, as reported to gateway is...
		 *
		 * 0 : actual cleaner RunState is Stopped, but it could be started ( canRun() == true )
		 * 1 : cleaner RunState is Running
		 * 2 : actual cleaner RunState is Stopped AND it cannot be started ( canRun() == false )
		 */
		int cleanerRunState		= ( cleanerPump->getRunState() == Running ) ? 1 : cleanerPump->canRun() ? 0 : 2;

		int waterTemperatureCx10 = (int)(waterTemperature->getCelsius()*10);

		/*
		 * Always send filterRunState, cleanerRunState and poolLamp state
		 */
		sprintf(payload, "PFR:%d PCR:%d PLI:%d", filterRunState, cleanerRunState, poolLamp->getLampState() );

		/*
		 * Concatenate secondary metrics only when filterPump is Running
		 */
		if ( filterRunState == Running ) {

			sprintf( secondary, " PTE:%d PPR:%d PCL:%d PPH:%d PFL:%d", waterTemperatureCx10, filterPressure->getPSI(), 1, 74, cleanerPump->getFlow_MLPS());

			strncat( payload, secondary, sizeof(payload) - strlen(payload) );

		}

		uint8_t sendSize = strlen( payload );

		if (sendSize > Radio::MAX_PAYLOAD_LEN) {
			sprintf(msg, "*** Payload is %d bytes too long and has been truncated to %d bytes! ***", (sendSize-Radio::MAX_PAYLOAD_LEN), Radio::MAX_PAYLOAD_LEN);
			DEBUG(msg);
			sendSize = Radio::MAX_PAYLOAD_LEN;
		}

		DEBUG(payload);

		statusSent = radio->send( GATEWAYID, payload, sendSize );

		Utility::Blink( 3 );

		statusIntervalLast = intervalNow; // update the last interval to the current so we can avoid retransmission until the next interval

		DEBUGln();

   }

   return statusSent;
 }

 void handleReceivedPackets( void ) {

   ReceivedData rd;

   if ( radio->getReceivedData( rd ) ) {

	   DEBUG("Handle request [len=");
	   DEBUG(rd.len);
	   DEBUG("] -->");
	   DEBUG((char*)rd.buf);
	   DEBUGln();

	   char responseBuf[10];

	 /*
	  * Messages Handled
	  *
	  * PFR:0 - Stop filter pump
	  * PFR:1 - Run filter pump
	  * PFR:? - Return filter pump run state
	  * PCR:0 - Stop cleaner pump
	  * PCR:1 - Run cleaner pump
	  * PCR:? - Return cleaner pump run state
	  * PLI:0 - Extinguish pool lamp
	  * PLI:1 - Light pool lamp
	  * PLI:? - Return pool lamp illumination state
	  */

	   if ( (rd.len == 5) && (rd.buf[3] == ':') ) {  // XXXX:N

		 ////////////////////////////////////////////////////
		 //////  F I L T E R   P U M P  C O N T R O L   /////
		 ////////////////////////////////////////////////////

		 if ( rd.buf[0]=='P' && rd.buf[1]=='F' && rd.buf[2]=='R' ) { // (FILT:0, FILT:1)

			 uint8_t desiredState = rd.buf[4];

			 if ( desiredState == '0' ) {					    // Stop Filter Pump

				 if ( cleanerPump->getRunState() == Running ) { // First stop running cleaner pump

					 DEBUG("   *** Stop Cleaner Pump ***"); DEBUGln();

					 cleanerPump->setRunState( Stopped );

		     		 sendCleanerPumpRunState( rd.senderID ); delay(1000); // was 5000

				 }

				 DEBUG("   *** Stop Filter Pump ***");

				 filterPump->setRunState( Stopped );

     		 } else if ( desiredState == '1' ) {				// Start Filter Pump


				 DEBUG("   *** Run Filter Pump ***");

				 filterPump->setRunState( Running );

			 }

			 sendFilterPumpRunState( rd.senderID );

		 }

		 //////////////////////////////////////////////////////
		 //////  C L E A N E R   P U M P  C O N T R O L   /////
		 //////////////////////////////////////////////////////

     	 if ( rd.buf[0]=='P' && rd.buf[1]=='C' && rd.buf[2]=='R' ) { // (CLNR:0, CLNR:1)

     		 uint8_t desiredState = rd.buf[4];

     		 if ( desiredState == '0' ) {					// Stop Cleaner Pump

				 DEBUG("   *** Stop Cleaner Pump ***");

				 cleanerPump->setRunState( Stopped );

     		 } else if ( desiredState == '1' ) {			// Start Cleaner Pump

     			 if ( (cleanerPump->getRunState() == Stopped) ) {

     				 if ( cleanerPump->canRun() ) {

     					 DEBUG("   *** Start Cleaner Pump ***");

     					 cleanerPump->setRunState( Running );

					 } else {								// Can't run. Set to DISABLED

     					 DEBUG("   !!! Cannot run cleaner pump !!!");

					 }

     			 }

     		 }

     		 sendCleanerPumpRunState( rd.senderID );

     	 }

		 /////////////////////////////////////////////////
		 //////  P O O L   L A M P   C O N T R O L   /////
		 /////////////////////////////////////////////////

     	 if ( rd.buf[0]=='P' && rd.buf[1]=='L' && rd.buf[2]=='I' ) { // (PLI:0, PLI:1)

     		 uint8_t desiredState = rd.buf[4];

     		 if ( desiredState == '0' ) {					// Extinguish Pool Lamp

     			 DEBUG("   *** Extinguish Pool Lamp ***");

     			 poolLamp->setLampState( Off );

     		 } else if ( desiredState == '1' ) {			// Light Pool Lamp

     			 DEBUG("   *** Light Pool Lamp ***");

     			 poolLamp->setLampState( On );

     		 }

     		sendPoolLampState( rd.senderID );

     	 }

     }
	 DEBUGln();
   }

 }


 //The setup function is called once at startup of the sketch
void setup()
{

	Serial.begin( SERIAL_BAUD );
	delay(100);
	Serial.flush();

	pinMode( LED, OUTPUT );
	digitalWrite( LED, LOW );

	if (NULL != (radio = new Radio( RF69_433MHZ, NODEID, NETWORKID, ENCRYPTKEY ))) {

		if ( NULL == poolLamp ) {
			poolLamp = new PoolLamp( POOL_LAMP_RELAY_PIN );
		}

		if ( NULL != (filterPump = new FilterPump( FILTER_PUMP_RELAY_PIN ))) {

			filterPump->setRunState( Stopped );

			if ( NULL != (cleanerFlow = new FlowSensor( CLEANER_FLOW_SENSOR_PIN, INT1 ))) {

				cleanerPump	= new CleanerPump( filterPump, cleanerFlow, CLEANER_PUMP_RELAY_PIN );

				cleanerPump->setRunState( Stopped );

				waterTemperature = new Thermistor( WATER_TEMP_SENSOR_PIN );

				filterPressure = new FilterPressure( FILTER_PRESSURE_SENSOR_PIN );

//				/*
//				 * Register for pin change interrupts on port D, where our toggle switches are
//				 */
//				PCICR  |=  (1<<PCIE2);		// issue pin change interrupts on port D, for digital pins 0-7 (PCIE2)
//				PCMSK2 |=  (1<<PCINT20);	// enable PCint on digital pin 4 for the filter pump toggle (PCINT20)
//				PCMSK2 |=  (1<<PCINT22);	// enable PCint on digital pin 6 for the cleaner pump toggle (PCINT22)
//				sei(); // SREG   |=  (1<<SREG_I);		// enable interrupts
//				SREG   &= ~(1<<SREG_I);		// disable interrupts

				if (flash.initialize()) flash.sleep();

				Utility::Blink( 100 ); Utility::Blink( 100 ); Utility::Blink( 100 );

			} else {
				// Fail!
			}

		} else {
			// FAIL!
		}
	} else {
		// FAIL!
	}
	DEBUGln();
}

/*
 * updateCleanerRunState - reconcile cleaner's current RunState with current filter pump and cleaner circuit flow conditions
 */
void updateCleanerRunState() {

	RunState curRunState = cleanerPump->getRunState();
	RunState newRunState = curRunState;					// Initially, assume no change in RunState

	/*
	 * If the cleaner pump is currently running but conditions do not
	 * support that state, stop the cleaner pump
	 */
	if ( ( curRunState == Running ) && !cleanerPump->canRun() ) {

		DEBUG("   *** Stop & Disable Cleaner Pump ( cleanerPump->canRun() == false ) ***"); DEBUGln();

		newRunState = Stopped;

	}

	/*
	 * If the cleaner's RunState needs to change, change it here and notify the gateway
	 */
	if ( newRunState != curRunState ) {

		cleanerPump->setRunState( newRunState );

		// delay(100);

		sendCleanerPumpRunState();

	}

}

void handleSerialRequest() {

	//
	// If Serial.available() is true, data from the FTDI (USB) waiting.
	//
	//	if ( Serial.available() ) {
	//		char input = Serial.read();
	//
	//		switch ( input ) {
	//			case 'f':
	//				break;
	//			case 'c':
	//				break;
	//			case 't':
	//				break;
	//			case 'p':
	//				break;
	//		}
	//	}

}

// The loop function is called in an endless loop
void loop()
{
	handleSerialRequest();

	updateCleanerRunState();

	transmitStatus();

	handleReceivedPackets();
}


///*
// * Pin Change Interrupt Service Routines - only using PORTD interrupts (PCINT2) at this point
// */
//volatile uint8_t previousPins = 0;
//volatile uint8_t pins = 0;
//
//ISR(PCINT2_vect){ // Port D, PCINT16 - PCINT23
//	/*
//	 * PCINT20: PCIFILTER_PUMP_TOGGLE_PIN (digital 4) -  call FILTER_PUMP_toggleRunState()
//	 * PCINT22: CLEANER_PUMP_TOGGLE_PIN (digital 6) - call CLEANER_PUMP_toggleRunState()
//	 */
//	previousPins = pins;
//	pins = (PIND & (PCINT20 | PCINT22));
//
//}
