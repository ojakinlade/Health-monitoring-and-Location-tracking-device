#include <ESP8266WiFi.h>
#include <SPI.h>             
#include <LoRa.h>
#include <ThingSpeak.h>
#include <LiquidCrystal_I2C.h>
#include "numeric_lib.h"

#define TX_SIZE         1
#define API_KEY       "EEX0AFZ56X4T1XUD"
#define CHANNEL_ID    2146562 

const int csPin = 15;          
const int resetPin = 16;     
const int irqPin = 2;  

const char* ssid = "A9 Pro";
const char* pword = "weslena99#"; 

typedef struct
{
  float temp;
  float lng;
  float lat;
  float bpm;
}SensorData_t;

typedef struct
{
  char temp[10];
  char lng[10];
  char lat[10];
  char bpm[10];
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
LiquidCrystal_I2C lcd(0x27,16,2);

// Function to find the positions of commas in a string
void FindCommaPositions(char* str, int* pos1, int* pos2, int* pos3)
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
  for(int k = *pos2 + 1; k < strlen(str); k++)
  {
    if(str[k] == ',')
    {
        *pos3 = k;
        break;
    }
  }
}

/**
* @brief Function to extract and store values between commas in separate arrays
* @param str The input string
* @param temp The array to store the value before the first comma
* @param lng The array to store the value between the first and second commas
* @param lat The array to store the value after the second comma
* @param pos1 The position of the first comma
* @param pos2 The position of the second comma
*/
void ParseData(char* str, char* temp, char* lng, char* lat,char* bpm,int pos1,int pos2,int pos3)
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
  for(int k = pos2 + 1; k < pos3; k++)
  {
    lat[k-(pos2 + 1)] = str[k];
  }
  for(int l = pos3 + 1; l < len; l++)
  {
    bpm[l-(pos3 + 1)] = str[l];
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
  //Start-up Message
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Health Monitor");
  delay(2000);
  lcd.clear();
  lcd.print("1: OKAY ");
  lcd.setCursor(0,1);
  lcd.print("2: OKAY ");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  ThingSpeak.begin(wifiClient);
}

void loop() {
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

  Serial.println("Query sent");
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;        

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingLength = LoRa.read();    // incoming msg length

  char rxData[100] = {0};
  int pos1;
  int pos2;
  int pos3;
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
  Serial.println();

  if(sender == node1Addr)
  {
    Serial.println("--Data Received from node 1");
    Serial.println(rxData);
    FindCommaPositions(rxData,&pos1,&pos2,&pos3);
    ParseData(rxData,rxData1.temp,rxData1.lng,rxData1.lat,rxData1.bpm,pos1,pos2,pos3);
    
    StringToFloat(rxData1.temp,&node1Data.temp);
    StringToFloat(rxData1.lng,&node1Data.lng);
    StringToFloat(rxData1.lat,&node1Data.lat);
    StringToFloat(rxData1.bpm,&node1Data.bpm);
    node1Data.temp = node1Data.temp * 1.28;
    Serial.print("Temp:");
    Serial.println(node1Data.temp);
    Serial.print("Lng:");
    Serial.println(node1Data.lng);
    Serial.print("Lat:");
    Serial.println(node1Data.lat); 
    Serial.print("BPM:");
    Serial.println(node1Data.bpm); 
    if(node1Data.temp < 35.0 || node1Data.bpm < 20)
    {
      lcd.setCursor(3,0);
      lcd.print("CRITICAL");
    }
    else if(node1Data.temp > 38.0 || node1Data.bpm < 20)
    {
      lcd.setCursor(3,0);
      lcd.print("CRITICAL");
    }
    else
    {
      lcd.setCursor(3,0);
      lcd.print("OKAY     ");
    } 
  }
  else if(sender == node2Addr)
  {
    Serial.println("--Data received from node 2");
    Serial.println(rxData);
    FindCommaPositions(rxData,&pos1,&pos2,&pos3);
    ParseData(rxData,rxData2.temp,rxData2.lng,rxData2.lat,rxData2.bpm,pos1,pos2,pos3);
    
    StringToFloat(rxData2.temp,&node2Data.temp);
    StringToFloat(rxData2.lng,&node2Data.lng);
    StringToFloat(rxData2.lat,&node2Data.lat);
    StringToFloat(rxData2.bpm,&node2Data.bpm);
    node2Data.temp = node2Data.temp * 1.28;
    Serial.print("Temp:");
    Serial.println(node2Data.temp);
    Serial.print("Lng:");
    Serial.println(node2Data.lng);
    Serial.print("Lat:");
    Serial.println(node2Data.lat); 
    Serial.print("BPM:");
    Serial.println(node2Data.bpm);
    if(node2Data.temp < 35.0 || node2Data.bpm < 20)
    {
      lcd.setCursor(3,1);
      lcd.print("CRITICAL");
    }
    else if(node2Data.temp > 38.0 || node2Data.bpm < 20)
    {
      lcd.setCursor(3,1);
      lcd.print("CRITICAL");
    }
    else
    {
      lcd.setCursor(3,1);
      lcd.print("OKAY     ");
    } 
  }

  if(millis() - prevUploadTime >= 25000)
  {
    //Upload to thingspeak
    ThingSpeak.setField(1,node1Data.temp);
    ThingSpeak.setField(2,node1Data.bpm);
    ThingSpeak.setField(3,node1Data.lat);
    ThingSpeak.setField(4,node1Data.lng);
    ThingSpeak.setField(5,node2Data.temp);
    ThingSpeak.setField(6,node2Data.bpm);
    ThingSpeak.setField(7,node2Data.lat);
    ThingSpeak.setField(8,node2Data.lng);
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
