#include "SoftwareSerial.h"

#define PAYLOAD_MAX_LENGTH  19

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

char * inputBuffer;

boolean commandReceived = false;  
boolean commandSent = true; 

typedef struct
{
  uint8_t command;
  uint8_t length;
  uint8_t payload[PAYLOAD_MAX_LENGTH];
  uint16_t  crc;
} bridge_comm_t;

bridge_comm_t up_channel;

struct{
  bridge_comm_t channel;
  bool packet_ok;
  uint8_t rec_bytes;
  uint8_t payload_c;
} down;

int led = 7;
int button = 2;


SoftwareSerial mySerial(10, 11); // RX, TX

void setup()
{
  // initialize serial:
  Serial.begin(115200);

  pinMode(led, OUTPUT);
  pinMode(button, INPUT);
  
  mySerial.begin(115200);
  mySerial.println("\n\r-------------------------------\
                    \n\rArduino / WunderBar-Bridge app \
                    \n\r-------------------------------\n\r");
}

uint8_t myData[2];

void loop()
{
  if (commandReceived)
  {
    commandReceived = false;

    mySerial.println("\n\n\r<<<=== Packet received:");
    if (down.packet_ok) mySerial.print("CRC OK");
    dump_packet(down.channel); 

    if(down.channel.command == BRIDGE_COMM_NCONN)
    {
      mySerial.println("\n\n\rError: Bridge Module not connected to the Wunderbar");
    }
  }
  
  //on button press send test packet 
  //if(digitalRead(button) == 0 && commandSent){
    digitalWrite(led, HIGH);

    uint8_t outputBuffer[PAYLOAD_MAX_LENGTH];

    myData[0]++;
    myData[1]++;
    bridge_comm_t outPacket = create_up_packet(myData, sizeof(myData), outputBuffer);

   // memcpy((char *) &outPacket, outputBuffer, sizeof(outPacket));//outPacket.length + 4);

    mySerial.println("\n\r===>>> OutPacket:");
    dump_packet(outPacket);

    mySerial.print("output Buffer: "); 
    for (char i=0; i < outPacket.length+4; i++)
    {
      mySerial.print(outputBuffer[i], HEX);
      mySerial.print(",");
    }

    Serial.write((uint8_t *)outputBuffer, outPacket.length + 4);  

    commandSent  = false;

    delay(3000);
  //}

  if(digitalRead(button) == 1)
  {
    commandSent  = true;
    digitalWrite(led, LOW);
    delay(500);
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent()
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
      mySerial.println("\n");
      down.channel.crc = inChar;
    }
    
    if (down.rec_bytes == down.channel.length + 4)
    {
      down.channel.crc += (inChar << 8);
      //mySerial.println(down.channel.crc,HEX);
      
      uint16_t c_crc = crc16_compute((uint8_t *) &down.channel,down.channel.length+2,NULL);
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

uint16_t crc16_compute(uint8_t * p_data, int size, uint8_t * p_crc)
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
bridge_comm_t create_up_packet(uint8_t * payload, int length, uint8_t * outBuffer)
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

  mySerial.print("\n\rCRC H/L: ");
  mySerial.print(outBuffer[length + 2], HEX);
  mySerial.print(outBuffer[length + 3], HEX);

  return packet;
}

void dump_packet(bridge_comm_t packet)
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
