#include "Arduino.h"
#include "Grove_bridge.h"

#include "SoftwareSerial.h"


char * inputBuffer;

boolean commandReceived = false;  
boolean commandSent = true; 

bridge_comm_t up_channel;

struct{
  bridge_comm_t channel;
  bool packet_ok;
  uint8_t rec_bytes;
  uint8_t payload_c;
} down;


Bridge::Bridge(int tx_pin=10, int rx_pin=11, int32_t baudrate=115200)
{
	Serial.begin(115200);	//this is the default baudrate for the Bridge (could be changed in the BLE Config char)

#if (BRIDGE_DEBUG)
	//initialize Soft Serial UART:
	SoftwareSerial mySerial(tx_pin, rx_pin); // RX (default 10), TX (default 11)
	mySerial.begin(baudrate);
#endif	
  	pinMode(pin, OUTPUT);
	_pin = pin;
}

bool Bridge::begin()
{
#if (BRIDGE_DEBUG)
  	mySerial.println("\n\r\n\r------------------------------- \
                          \n\r relayr - bring things to life  \
                          \n\r Arduino / WunderBar-Bridge lib \
                          \n\r-------------------------------\n\r");
#endif  	
  	Serial.write(BRIDGE_COMM_PING);  
  	delay(100);
  	if(!down.commandReceived || (down.command != BRIDGE_COMM_ACK))
  	{
#if (BRIDGE_DEBUG)
		mySerial.println("\n\r Could not find a valid Bridge module, \
			check wiring and that the Brige has the UART firmware!\n\r\
			Arduino TX ==>> Bridge RX (white wire)\n\r\
			Arduino RX ==>> Bridge TX (Yellow wire)");
#endif
		return false;
  	}
  	return true;

}

bool Bridge::sendData((char *) payload)
{


}


/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void Bridge::serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    uint8_t inChar = (uint8_t) Serial.read(); 
    // add it to the inputString:
    
    //inputBuffer+= inChar;
    down.rec_bytes++;
    
    if (down.rec_bytes == 1)
    {

      down.channel.command = inChar;

      switch (inChar)
      {
          case BRIDGE_COMM_ACK:
            down.rec_bytes = 0;
            commandReceived = true;
            break;
          case BRIDGE_COMM_NACK:
            down.rec_bytes = 0;
            commandReceived = true;
            break;
          case BRIDGE_COMM_NCONN:
            down.rec_bytes = 0;
            commandReceived = true;
            break;  
          case BRIDGE_COMM_PING:
            Serial.write(BRIDGE_COMM_ACK);  
            down.rec_bytes = 0;
            commandReceived = true;
            break;  
          case BRIDGE_COMM_RCV_FROM_BLE:
            break;  
          default:
            down.rec_bytes = 0;
            down.packet_ok = false;
      }
      //mySerial.println(down.channel.command,HEX);
    }
    
    if (down.rec_bytes == 2)
    {
      down.channel.length = inChar;
      //mySerial.println(down.channel.length,HEX);
    }
    
    if ( (down.rec_bytes > 2) && (down.rec_bytes <= (down.channel.length + 2)))
    {
      down.channel.payload[down.payload_c] = inChar;
      //mySerial.print(down.channel.payload[down.payload_c]);
      down.payload_c++;
    }
    
    if (down.rec_bytes == down.channel.length + 3)
    {
      //mySerial.println("\n");
      down.channel.crc = inChar;
    }
    
    if (down.rec_bytes == down.channel.length + 4)
    {
      down.channel.crc += (inChar << 8);
      //mySerial.println(down.channel.crc,HEX);
      
      uint16_t c_crc = crc16_compute((uint8_t *) &down.channel, down.channel.length+2, NULL);
      //mySerial.println(c_crc,HEX);
      
      if (c_crc == down.channel.crc)
      {
        down.packet_ok = true;
        digitalWrite(led, LOW); 
      } else down.packet_ok = false;
    
      commandReceived = true;

      //reset counters
      down.rec_bytes = 0;
      down.payload_c = 0;
      /*
      mySerial.println("Packet received:");
      if (down.packet_ok) mySerial.print("CRC OK");
      dump_packet(down.channel);     */ 
    }
  }
}

uint16_t Bridge::crc16_compute(uint8_t * p_data, int size, uint8_t * p_crc)
{
    uint32_t i;
    uint16_t crc = (p_crc == NULL) ? 0xffff : *p_crc;

    for (i = 0; i < size; i++)
    {
        crc = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= p_data[i];
        crc ^= (uint8_t)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }    
    return crc;
}

/*
*params: int length (payload length)
*/
bridge_comm_t Bridge::create_up_packet(uint8_t * payload, int length, uint8_t * outBuffer)
{
  bridge_comm_t packet;

  packet.command = BRIDGE_COMM_WRITE_UP_CHANNEL;
  packet.length = length;

  memcpy(packet.payload, payload, length);

  outBuffer[0] = packet.command;
  outBuffer[1] = packet.length;
  
  for (char i = 2; i < length + 2; i++)
  {
    outBuffer[i] = packet.payload[i-2];
  }

  packet.crc = crc16_compute(outBuffer, length + 2, NULL);

  outBuffer[length + 2] = (packet.crc & 0xff);
  outBuffer[length + 3] = (packet.crc >> 8);

  /*mySerial.print("\n\rCRC H/L: ");
  mySerial.print(outBuffer[length + 2], HEX);
  mySerial.print(outBuffer[length + 3], HEX);*/

  return packet;
}

void Bridge::dump_packet(bridge_comm_t packet)
{
    mySerial.print("\n\r-----------\n\r");
    mySerial.print("Packet size: ");
    mySerial.println(packet.length+4);
    mySerial.print("CMD: ");
    mySerial.println(packet.command, HEX);
    mySerial.print("Payload length: ");
    mySerial.println(packet.length, DEC);
    mySerial.print("Payload: ");

    for (char i = 2; i < packet.length+2; i++)
    {
      mySerial.print(packet.payload[i-2], HEX);
      mySerial.print(", ");
    }  

    mySerial.print("\n\rCRC: ");
    mySerial.print(packet.crc, HEX);
    mySerial.print("\n\r-----------\n\r");
}
