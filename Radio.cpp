/*
 * Radio.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#include "Radio.h"
#include "Debug.h"
#include "Utility.h"

Radio::Radio( uint8_t freqBand, uint8_t nodeID, uint8_t networkID, const char* encryptionKey ) {

	rfm69.initialize( freqBand, nodeID, networkID );

	#ifdef IS_RFM69HW // define only for RFM69HW!
	rfm69.setHighPower();
	#endif

	rfm69.encrypt( encryptionKey );

	//radio.sleep();

	char buf[128];
	sprintf(buf, "\nNode %d on network %d transmitting at %d Mhz...", nodeID, networkID, freqBand==RF69_433MHZ ? 433 : freqBand==RF69_868MHZ ? 868 : 915 );
	DEBUG(buf);
	DEBUGln();

}

Radio::~Radio() {
	// TODO Auto-generated destructor stub
}

bool Radio::send( uint8_t targetID, const void* buffer, uint8_t len ) {

	rfm69.send( targetID, buffer, len );

	return true;

}

bool Radio::sendWithRetry( uint8_t targetId, const void* buffer, uint8_t len, uint8_t retries, uint16_t retryWaitMs ) {

	bool xmitStatus = false;

	if (len > RF69_MAX_DATA_LEN) {
	  len = RF69_MAX_DATA_LEN; // Cap payload at maximum
	}

	if (rfm69.sendWithRetry( targetId, buffer, len, retries, retryWaitMs )) {
		DEBUG(" ok!");
		xmitStatus = true;
	}
	else DEBUG(" send failed");

	return xmitStatus;

}

bool Radio::getReceivedData( ReceivedData& receivedData ) {

	bool got_received_data = false;

	if ( rfm69.receiveDone() ) {

		receivedData.senderID	= rfm69.SENDERID;
		receivedData.len		= rfm69.DATALEN;
		receivedData.RSSI		= rfm69.RSSI;

		memset( receivedData.buf, 0, 						sizeof( receivedData.buf ) );
		memcpy( receivedData.buf, (uint8_t *)rfm69.DATA,	sizeof( receivedData.buf ) );

		if ( rfm69.ACKRequested() )
		{
		  rfm69.sendACK();
		  DEBUG(" - ACK sent");
		}

		#ifdef SERIAL_EN
			char buf[4];
			sprintf(buf, "%d", receivedData.senderID);
			DEBUG('[');DEBUG( buf );DEBUG("] ");
			for (byte i = 0; i < receivedData.len; i++)
				DEBUG((char)receivedData.buf[i]);

			DEBUG("   [RX_RSSI:"); DEBUG( receivedData.RSSI ); DEBUG("]");
		#endif

		DEBUGln();

		Utility::Blink( 3 );

		got_received_data = true;

	}
	return got_received_data;
}

uint16_t Radio::getSendDataLimit() {
	return MAX_PAYLOAD_LEN;
}
