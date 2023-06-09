#include <SPI.h>            
#include <LoRa_STM32.h>
#include <TinyGPSPlus.h>
#include "numeric_lib.h"

#define RX_SIZE   1
#define gpsModule Serial1

static TinyGPSPlus gps; 

const int csPin = PA4;          
const int resetPin = PB0;       
const int irqPin = PA1;         
const int LM35Pin = PA2;

typedef struct
{
  float temp;
  float lng;
  float lat;
}SensorData_t;

typedef struct
{
  char temp[10];
  char lng[10];
  char lat[10];
}StrData_t;

String outgoing;              
byte localAddress = 0xFF;    
byte destination = 0xBB;      // Master address
long lastSendTime = 0;       
int interval = 2000;         
SensorData_t sensorData;
StrData_t txData;
char dataToSend[100] = {0};

void setup() {
  Serial.begin(115200);            
  gpsModule.begin(9600); //gpsTx: PA10, gpsRx: PA9 
  while (!Serial);

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {            
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                 
  }
  Serial.println("LoRa init succeeded.");
}

void loop() {
  // parse for a packet, and call onReceive with the result:
  //onReceive(LoRa.parsePacket());
  Get_SensorData(&sensorData);
  FloatToString(sensorData.temp,txData.temp,2);
  FloatToString(sensorData.lng,txData.lng,2);
  FloatToString(sensorData.lat,txData.lat,2);

  strcat(dataToSend,txData.temp);
  strcat(dataToSend,",");
  strcat(dataToSend,txData.lng);
  strcat(dataToSend,",");
  strcat(dataToSend,txData.lat);
  Serial.println(dataToSend);
  
  //sendMessage(txData.temp);
  memset(txData.temp,'\0',strlen(txData.temp));
  memset(txData.lng,'\0',strlen(txData.lng));
  memset(txData.lat,'\0',strlen(txData.lat));
  memset(dataToSend,'\0',strlen(dataToSend));
}

void sendMessage(char* outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(strlen(outgoing));        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingLength = LoRa.read();    // incoming msg length

  uint8_t incoming = LoRa.read();

  if (incomingLength != RX_SIZE) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                   
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message length: " + String(incomingLength));
  Serial.print("Incoming message: ");
  Serial.println(incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();

  delay(100);
  Get_SensorData(&sensorData);
  FloatToString(sensorData.temp,txData.temp,2);
  FloatToString(sensorData.lng,txData.lng,2);
  FloatToString(sensorData.lat,txData.lat,2);

  strcat(dataToSend,txData.temp);
  strcat(dataToSend,",");
  strcat(dataToSend,txData.lng);
  strcat(dataToSend,",");
  strcat(dataToSend,txData.lat);
  
  sendMessage(dataToSend);
  memset(txData.temp,'\0',strlen(txData.temp));
  memset(txData.lng,'\0',strlen(txData.lng));
  memset(txData.lat,'\0',strlen(txData.lat));
  memset(dataToSend,'\0',strlen(dataToSend));
}

void Get_SensorData(SensorData_t* data)
{
  int rawVal = analogRead(LM35Pin);
  float volt = rawVal * (3.0/4096.0);
  float temp = volt * 100;
  data->temp = temp;
  if(gpsModule.available() && gps.encode(gpsModule.read()))
  {
    data->lng = gps.location.lng();
    data->lat = gps.location.lat();
  }
}
