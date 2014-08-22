/* Library for connect Arduino to the Wunderbar Bridge Module 
** Author: Daniel Mancuso
** Relayr.io
*/
//http://arduino.cc/en/Hacking/LibraryTutorial

#include "Arduino.h"

#ifndef Grove_bridge_h
#define Grove_bridge_h

#define PAYLOAD_MAX_LENGTH  19	//Max Payload Length for the Bridge UART packet.

typedef enum
{
    BRIDGE_COMM_WRITE_UP_CHANNEL  = 0x01,
    BRIDGE_COMM_READ_UP_CHANNEL   = 0x02,
    BRIDGE_COMM_READ_DOWN_CHANNEL = 0x03,
    BRIDGE_COMM_ACK               = 0x04,
    BRIDGE_COMM_NACK              = 0x05,
    BRIDGE_COMM_PING              = 0x06,
    BRIDGE_COMM_RCV_FROM_BLE      = 0x07,
    BRIDGE_COMM_NCONN             = 0x08,
}
bridge_commands_t;

typedef struct
{
  uint8_t command;
  uint8_t length;
  uint8_t payload[PAYLOAD_MAX_LENGTH];
  uint16_t  crc;
} bridge_comm_t;


class Bridge
{
	public:
		Bridge(int tx_pin=10, int rx_pin=11, int32_t baudrate=115200);
	private:
		uint16_t crc16_compute(uint8_t * p_data, int size, uint8_t * p_crc)
		void dump_packet(bridge_comm_t packet)
		bridge_comm_t create_up_packet(uint8_t * payload, int length, uint8_t * outBuffer)

		int _tx_pin;
		int _rx_pin;
		int32_t _baudrate;
};

#endif