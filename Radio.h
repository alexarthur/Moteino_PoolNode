/*
 * Radio.h
 *
 *  Created on: Jul 4, 2018
 *      Author: artal03
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <Arduino.h>
#include <RFM69.h>

typedef struct {
	uint8_t senderID;
	int16_t RSSI;
	uint8_t buf[ RF69_MAX_DATA_LEN ];
	uint8_t len;
} ReceivedData;


class Radio {
private:
	#ifdef ENABLE_ATC
	  RFM69_ATC radio;
	#else
	  RFM69 rfm69;
	#endif
	  uint8_t readBuf[ RF69_MAX_DATA_LEN + 1 ];

public:
					Radio( uint8_t freqBand, uint8_t nodeID, uint8_t networkID, const char* encryptionKey );
	virtual			~Radio();

	static	const	uint8_t		MAX_PAYLOAD_LEN = RF69_MAX_DATA_LEN;
	static			uint16_t	getSendDataLimit();

	bool		send			( uint8_t targetId, const void* buffer, uint8_t len);
	bool		sendWithRetry	( uint8_t targetId, const void* buffer, uint8_t len, uint8_t retries = 4, uint16_t retryWaitMillis = 50 );
	bool 		getReceivedData	( ReceivedData &receivedData );

};

#endif /* RADIO_H_ */
