#include "antdefines.h"
#include "antmessage.h"


//ANT+ Constants for power meter channel
#define CHANNEL_TYPE_SLAVE (0x00)
#define ANTPLUS_NETWORK_KEY {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //NOTE: This is a private key that must be obtained from the ANT+ website. It is free to sign up.
#define DEVICE_TYPE_POWER_METER (11)
#define DEVICE_NUMBER_WILDCARD (0x00)
#define SENSOR_CHANNEL_FREQ (57)
#define CHANNEL_PERIOD_POWER_METER (8182)
#define TIMEOUT (30) //SECONDS

#define CHANNEL_TYPE_BIDIRECTIONAL_SLAVE (0X00)

#include <SoftwareSerial.h>

//Pins cnnections from NRF24AP2 to Arduino Pro Mini
#define TX_PIN (8) 
#define RX_PIN (9)
#define SUSPEND_PIN (3)
#define SLEEP_PIN (4)
#define RESET_PIN (5)
#define RTS_PIN (2)



SoftwareSerial ANT_serial(TX_PIN, RX_PIN); // RX, TX pins on arduino


typedef struct ANT_packet {
  byte sync;
  byte length;
  byte mesgID;
  byte data[];
} ANT_packet;

typedef struct ANT_channel {
  int channelNumber;
  byte channelType;
  byte ANT_networkKey[8];
  int networkNumber;
  int RF_frequency;
  int transmissionType; //can be zero on slave device
  int deviceType;
  int deviceNumber;
  int channelPeriod;
  int timeout;
} ANT_channel;

ANT_channel powerChannel =
{
  0, //channel no.
  CHANNEL_TYPE_BIDIRECTIONAL_SLAVE,
  ANTPLUS_NETWORK_KEY,
  0,
  SENSOR_CHANNEL_FREQ,
  0,
  DEVICE_TYPE_POWER_METER,
  DEVICE_NUMBER_WILDCARD,
  CHANNEL_PERIOD_POWER_METER,
  TIMEOUT
};

//Global data variables
int instPwr = 0; //instantaneous power

//Computes checksum for ANT+ Messsage. Bitwise XOR with all bytes (including SYNC)
byte checksum(byte* data, int size) {
  byte chksum = 0;
  for (int i = 0; i < size; i++) {
    chksum ^= data[i];
  }
  return chksum;
}

void ANT_send(byte* data, int size) {
//  Serial.println("Sending: ");

  for (int i = 0; i < size; i++) {
    ANT_serial.write(data[i]);
//    Serial.print(data[i], HEX);
//    Serial.println();
  }
}

//prints array for debugging
void printArray(byte arr[], int size) {
  for (int i = 0; i < size; i++) {
    Serial.println(arr[i], HEX);
  }
}

//Sends calibration request. Message structure is as outlined in the ANT+ docs
void requestCalibration(ANT_channel* channel) {
  Serial.println("Send Calibration Request");
  byte message[13];
  message[0] = MESG_TX_SYNC;
  message[1] = 9; //data msg length
  message[2] = MESG_ACKNOWLEDGED_DATA_ID; //msgID
  message[3] = channel->channelNumber;
  message[4] = 0x01; //calibration message
  message[5] = 0xAA;
  message[6] = 0xFF;
  message[7] = 0xFF;
  message[8] = 0xFF;
  message[9] = 0xFF;
  message[10] = 0xFF;
  message[11] = 0xFF;
  message[12] = checksum(message, 12);
  ANT_send(message, 13);
}


void ANT_setNetworkKey(ANT_channel* channel) {
  Serial.println("Setting network key ");

  byte message[13];
  message[0] = MESG_TX_SYNC;
  message[1] = 9; //data msg length
  message[2] = 0x46; //msgID
  message[3] = channel->networkNumber;
  memcpy((byte*)(message + 4 * sizeof(byte)), channel->ANT_networkKey, 8);
  message[12] = checksum(message, 12);
  ANT_send(message, 13);
}


void ANT_setChannelID(ANT_channel* channel) {
  Serial.println("Setting channel id ");
  byte message[9];
  message[0] = MESG_TX_SYNC;
  message[1] = 5; //data msg length
  message[2] = 0x51; //msgID
  message[3] = channel->channelNumber;
  message[4] = channel->deviceNumber & 0x00FF; //LSB
  message[5] = channel->deviceNumber & 0xFF00; //MSB
  message[6] = channel->deviceType;
  message[7] = channel->transmissionType;
  message[8] = checksum(message, 8);
  ANT_send(message, 9);
}

void ANT_setChannelPeriod(ANT_channel* channel) {
  Serial.println("Setting channel period ");
  byte message[7];
  message[0] = MESG_TX_SYNC;
  message[1] = 3; //data msg length
  message[2] = 0x43; //msgID
  message[3] = channel->channelNumber;
  message[4] = channel->channelPeriod & 0x00FF; //LSB
  message[5] = channel->channelPeriod & 0xFF00; //MSB
  message[6] = checksum(message, 6);
  ANT_send(message, 7);
}

void ANT_assignChannel(ANT_channel* channel) {
  Serial.println("Assigning channel ");
  byte message[7];
  message[0] = MESG_TX_SYNC;
  message[1] = 3; //data msg length
  message[2] = 0x42; //msgID
  message[3] = channel->channelNumber;
  message[4] = channel->channelType;
  message[5] = channel->networkNumber;
  message[6] = checksum(message, 6);
  ANT_send(message, 7);
}

void ANT_openChannel(ANT_channel* channel) {
  Serial.print("Openning channel ");
  byte message[5];
  message[0] = MESG_TX_SYNC;
  message[1] = 1; //data msg length
  message[2] = 0x4B; //msgID
  message[3] = channel->channelNumber;
  message[4] = checksum(message, 4);
  ANT_send(message, 5);
}

void ANT_SetChannelRFFreq(ANT_channel* channel) {
  Serial.println("Setting Frequency");
  byte message[6];
  message[0] = MESG_TX_SYNC;
  message[1] = 2; //data msg length
  message[2] = 0x45; //msgID
  message[3] = channel->channelNumber;
  message[4] = channel->RF_frequency;
  message[5] = checksum(message, 5);
  ANT_send(message, 6);
}

void ANT_SetChannelSearchTimeout(ANT_channel* channel) {
  Serial.println("Setting channel timeout ");
  byte message[6];
  message[0] = MESG_TX_SYNC;
  message[1] = 2; //data msg length
  message[2] = 0x44; //msgID
  message[3] = channel->channelNumber;
  message[4] = channel->timeout;
  message[5] = checksum(message, 5);
  ANT_send(message, 6);
}

void ANT_requestChannelStatus(ANT_channel* channel) {
  byte message[6];
  message[0] = MESG_TX_SYNC;
  message[1] = 2; //data msg length
  message[2] = 0x4D; //msgID
  message[3] = channel->channelNumber;
  message[4] = 0x52;
  message[5] = checksum(message, 5);
  ANT_send(message, 6);
}

void establishChannel(ANT_channel* channel) {
  ANT_setNetworkKey(channel);
  delay(1000);
  ANT_assignChannel(channel);
  delay(1000);
  ANT_setChannelID(channel);
  delay(1000);
  ANT_SetChannelRFFreq(channel);
  delay(1000);
  ANT_SetChannelSearchTimeout(channel);
  delay(1000);
  ANT_openChannel(channel);
  delay(1000);
}

void updateInstPwr(byte* data) {
  instPwr = data[6] | data[7] << 8;
  //Serial.println(instPwr);

}

//Interprets the calibration response packet
void updateCalibration(byte* data) {
  int calibData = data[6] | data[7] << 8;
  Serial.print("Zero Offset Value: ");
  Serial.println(calibData);
  if (data[1] == 0xAC) {
    Serial.print("Calibration Successful,  ");
    Serial.print("Zero Offset Value: ");
    Serial.println(calibData);
  } else if (data[1] == 0xAF)  {
    Serial.print("Calibration Unsuccessful ");
    Serial.print("Zero Offset Value: ");
    Serial.println(calibData);
  } else if (data[1] == 0x12){
    //this is a auto zero support packet
  } else {
    Serial.println("Other Calibration Page Received");
    Serial.print("Calibration ID: ");
    Serial.println(data[1], HEX);
    printArray(data, MESG_MAX_DATA_BYTES);
  }
  
}

//Constructs whole packet from incoming bytes on UART. Returns true if full packet is received successfully
boolean constructPacket(ANT_packet* packet) {
  byte rxbuf[ANT_MAX_SIZE];
  int bufcnt = 0;
  while (ANT_serial.available()) {
    byte byteIn = ANT_serial.read();
    delay(10); //the delay is needed so that we do not read the incoming bytes too quickly, leading ANT_serial.available() to return false in the middle of a packet
               //TODO encapsulate whole section with while statement (with timeout condition) and use if(ANT_serial.available()) instead.
    //      Serial.print("byte received ");
    //      Serial.print(byteIn, HEX);
    //      Serial.print("    buffer count ");
    //      Serial.println(bufcnt);
    if (bufcnt == 0) {
      if (byteIn == MESG_TX_SYNC) { //first byte - SYNC byte received
        rxbuf[bufcnt] = byteIn;
        bufcnt++;
      } else { //no sync byte
        return false;
      }
    } else if (bufcnt == 1) {
      rxbuf[bufcnt] = byteIn;		//Message length
      bufcnt++;
    } else if (bufcnt == 2) {
      rxbuf[bufcnt] = byteIn;    //Message ID
      bufcnt++;
    } else if ((bufcnt < (rxbuf[1] + MESG_HEADER_SIZE))) {
      rxbuf[bufcnt] = byteIn;    //Data bytes
      bufcnt++;
    } else if (bufcnt == (rxbuf[1] + MESG_HEADER_SIZE)) {
      rxbuf[bufcnt] = byteIn;   //checksum
      if (byteIn == checksum(rxbuf, rxbuf[1] + MESG_HEADER_SIZE)) { //if checksum is correct, packet is received successfully
        memcpy(packet, rxbuf, rxbuf[1] + 4);
        //printArray(rxbuf, rxbuf[1] + 4);
        return true;
      } else {
        return false;
      }
    }


  }
  return false;
}

//Interprets data based on packet
void readPacket(ANT_packet* packet) {
  int dataLength = packet->length;
  byte mesgID = packet->mesgID;
  byte dataPageNum = packet->data[1];

  switch (mesgID) {
    case MESG_BROADCAST_DATA_ID:
      Serial.print("data page       ");
      Serial.println(dataPageNum, HEX);
      switch (dataPageNum) {
        case 0x10: //basic power data page
          updateInstPwr(&(packet->data[1]));
          break;
        case 0x01: //calibration response
          updateCalibration(&(packet->data[1]));
//          printPacket(packet);
          break;
        default:
          Serial.println("Other data page (not handled)");
      }
      break;

    case MESG_CHANNEL_STATUS_ID:
      Serial.println("Channel status packet received.");

    default:
      Serial.print(mesgID, HEX);
      Serial.println(" msgID (not handled)");
  }
}

void setup() {
  ANT_serial.begin(9600); //the baud rate for the NRF24AP2 module I am using is 9600
  Serial.begin(115200); //Needs to be faster than ANT_serial baud rate if you want to debug by printing bytes continuously

  pinMode(SUSPEND_PIN, OUTPUT);
  pinMode(SLEEP_PIN,   OUTPUT);
  pinMode(RESET_PIN,   OUTPUT);
  pinMode(RTS_PIN,     INPUT);

  //As per nrf24ap2 docs for normal operation
  digitalWrite(RESET_PIN,   HIGH);
  digitalWrite(SUSPEND_PIN, HIGH);
  digitalWrite(SLEEP_PIN,   LOW);

  //printChannelParam(&powerChannel);

  //Resets MRF24AP2 Module
  digitalWrite(RESET_PIN, LOW);
  delay(5);
  digitalWrite(RESET_PIN, HIGH);
  delay(1000);

  establishChannel(&powerChannel);
  Serial.println("press any key to calibrate power meter");

}



void printPacket(ANT_packet* packet) {
  Serial.println("PACKET DATA");
  Serial.print("Sync : ");
  Serial.println(packet->sync, HEX);
  Serial.print("Length: ");
  Serial.println(packet->length, HEX);
  Serial.print("Channel num: ");
  Serial.println(packet->data[0]);
  printArray(&packet->data[1], packet->length);
  Serial.print("computed checksum");
  Serial.println(checksum((byte*)packet, packet->length + 3));
  Serial.print("Checksum in packet");
  Serial.println(packet->data[packet->length]);

}


void printChannelParam(ANT_channel* channel) {
  Serial.print("Channel Number: ");
  Serial.println(channel->channelNumber);
  Serial.print("Channel Type: ");
  Serial.println(channel->channelType);
  Serial.print("Network Key: ");
  printArray(channel->ANT_networkKey, 8);
  Serial.println();
  Serial.print("Network number: ");
  Serial.println(channel->channelNumber);
  Serial.print("Frequency: ");
  Serial.println(channel->RF_frequency);
  Serial.print("Transmission type: ");
  Serial.println(channel->transmissionType);
  Serial.print("device type: ");
  Serial.println(channel->deviceType);
  Serial.print("device number: ");
  Serial.println(channel->deviceNumber);
  Serial.print("Period: ");
  Serial.println(channel->channelPeriod);
  Serial.print("timeout: ");
  Serial.println(channel->timeout);
}

void loop() {
  byte packet[ANT_MAX_SIZE];
  ANT_packet* ant_packet = (ANT_packet*)packet;
  if (constructPacket(ant_packet)) {
    //Serial.println("packet constructed success");
    //printPacket(ant_packet);
    readPacket(ant_packet);
  }
    while(Serial.available()){
      Serial.read();
      requestCalibration(&powerChannel);
    }



}
