#include <ESP8266WiFi.h>
#include <SPI.h>             
#include <LoRa.h>
#include <ThingSpeak.h>
#include "numeric_lib.h"

#define TX_SIZE         1
#define API_KEY       "AAA"
#define CHANNEL_ID    2333 
#define WIFI_SSID     "aaa"
#define WIFI_PWORD    "XXX"



const int csPin = 15;          
const int resetPin = 16;     
const int irqPin = 2;         

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
         
byte localAddress = 0xBB;     
byte node1Addr = 0xFF;       
byte node2Addr = 0xEE;
byte query = 0xAA;
uint32_t lastSendTime; 
uint32_t prevUploadTime;      
int interval = 2000; // interval between sends
SensorData_t node1Data;
SensorData_t node2Data;
StrData_t rxData1;
StrData_t rxData2;

byte Node[2] = {node1Addr,node2Addr};
uint8_t node = 0;

static WiFiClient wifiClient;

void getPosition(char* str, int* pos1, int* pos2)
{
  for(int i = 0; i < strlen(str); i++)
  {
    if(str[i] == ',')
    {
        *pos1 = i;
        break;
    }
  }
  for(int j = *pos1 + 1; j < strlen(str); j++)
  {
    if(str[j] == ',')
    {
        *pos2 = j;
        break;
    }
  }
}

void storeValues(char* str, char* temp, char* lng, char* lat,int pos1,int pos2)
{
  int len = strlen(str);
  for(int i = 0; i < pos1; i++)
  {
    temp[i] = str[i];
  }
  for(int j = pos1 + 1; j < pos2; j++)
  {
    lng[j-(pos1 + 1)] = str[j];
  }
  for(int k = pos2 + 1; k < len; k++)
  {
    lat[k-(pos2 + 1)] = str[k];
  }
}

void setup() {
  Serial.begin(9600);                 
  Serial.println("LoRa Duplex");
  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(433E6))
  {  
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                    
  }
  Serial.println("LoRa init succeeded."); 
  lastSendTime = 0;
  prevUploadTime = 0;
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(wifiClient);
}

void loop() {
  if(WiFi.status() != WL_CONNECTED)
  {
    while(WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(WIFI_SSID,WIFI_PWORD);
      delay(500);
    }
    Serial.println("Connected");
  }
  if (millis() - lastSendTime > interval) {
    sendMessage(query,Node[node]);
    node = (node + 1) % 2;
    lastSendTime = millis();          
  }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

void sendMessage(uint8_t data, uint8_t addr) {
  LoRa.beginPacket(); 
  LoRa.write(addr); //destination address
  LoRa.write(localAddress);
  LoRa.write(TX_SIZE); //Message length
  LoRa.write(data); 
  LoRa.endPacket();        
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;        

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  char rxData[100] = {0};
  int pos1;
  int pos2;
  int i = 0;

  while (LoRa.available()) 
  {
    rxData[i++] = (char)LoRa.read();
  }

  if (incomingLength != strlen(rxData))
  {  
    Serial.println("error: message length does not match length");
    return;                        
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                          
  }
//  // if message is for this device, or broadcast, print details:
//  Serial.println("Received from: 0x" + String(sender, HEX));
//  Serial.println("Sent to: 0x" + String(recipient, HEX));
//  Serial.println("Message ID: " + String(incomingMsgId));
//  Serial.println("Message length: " + String(incomingLength));
//  Serial.println("Message: " + incoming);
//  Serial.println("RSSI: " + String(LoRa.packetRssi()));
//  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();

  if(sender == node1Addr)
  {
    
    Serial.println("--Data Received from node 1");
    Serial.println(rxData);
    getPosition(rxData,&pos1,&pos2);
    storeValues(rxData,rxData1.temp,rxData1.lng,rxData1.lat,pos1,pos2);
    StringToFloat(rxData1.temp,&node1Data.temp);
    StringToFloat(rxData1.lng,&node1Data.lng);
    StringToFloat(rxData1.lat,&node1Data.lat);
    Serial.print("Temp:");
    Serial.println(node1Data.temp);
    Serial.print("Lng:");
    Serial.println(node1Data.lng);
    Serial.print("Lat:");
    Serial.println(node1Data.lat);  
  }
  else if(sender == node2Addr)
  {
    Serial.println("--Data received from node 2");
    Serial.println(rxData);
    getPosition(rxData,&pos1,&pos2);
    storeValues(rxData,rxData2.temp,rxData2.lng,rxData2.lat,pos1,pos2);
    StringToFloat(rxData2.temp,&node2Data.temp);
    StringToFloat(rxData2.lng,&node2Data.lng);
    StringToFloat(rxData2.lat,&node2Data.lat);
    Serial.print("Temp:");
    Serial.println(node2Data.temp);
    Serial.print("Lng:");
    Serial.println(node2Data.lng);
    Serial.print("Lat:");
    Serial.println(node2Data.lat); 
  }

  if(millis() - prevUploadTime >= 25000)
  {
    //Upload to thingspeak
    ThingSpeak.setField(1,node1Data.temp);
    ThingSpeak.setField(2,node1Data.lng);
    ThingSpeak.setField(3,node1Data.lat);
    //ThingSpeak.setField(4,node1Data.temp);
    ThingSpeak.setField(5,node2Data.temp);
    ThingSpeak.setField(6,node2Data.lng);
    ThingSpeak.setField(7,node2Data.lat);
    //ThingSpeak.setField(8,node2Data.temp);
    if(ThingSpeak.writeFields(CHANNEL_ID,API_KEY) == 200)
    {
      Serial.println("SUCCESS: Data sent to ThingSpeak successfully");
    }
    else
    {
      Serial.println("ERROR: Sending to ThingSpeak failed");
    }
    prevUploadTime = millis();
  }
}
