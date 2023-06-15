#include <MapleFreeRTOS900.h>
#include <SPI.h>  
#include <Wire.h>          
#include <LoRa_STM32.h>
#include <TinyGPSPlus.h>
#include "numeric_lib.h"
#include "MAX30105.h"
#include "heartRate.h"

#define RX_SIZE   1
#define gpsModule Serial1

static TinyGPSPlus gps; 
static MAX30105 particleSensor;

const int csPin = PA4;          
const int resetPin = PB0;       
const int irqPin = PA1;         
const int LM35Pin = PA2;
const byte RATE_SIZE = 4;
const unsigned long AVERAGE_INTERVAL = 5000; // Calculate average every 5 seconds

byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
uint32_t lastAverageTime = 0;
bool queryReceived = false;


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

String outgoing;              
byte localAddress = 0xFF;    
byte destination = 0xBB;      // Master address
long lastSendTime = 0;       
int interval = 2000; 
SensorData_t sensorData;        
StrData_t txData;
char dataToSend[100] = {0};


void sendMessage(char* outgoing)
{
  LoRa.beginPacket();                   
  LoRa.write(destination);             
  LoRa.write(localAddress);             
  LoRa.write(strlen(outgoing));       
  LoRa.print(outgoing);                 
  LoRa.endPacket();                   
}

void Get_SensorData(SensorData_t* data)
{
  int rawVal = analogRead(LM35Pin);
  float volt = rawVal * (3.0/4096.0);
  float temp = volt * 100;
  data->temp = temp;  
  
  long irValue = particleSensor.getIR();
  Serial.print("IR:");
  Serial.println(irValue);

  //We sensed a beat!
  long delta = millis() - lastBeat;
  lastBeat = millis();

  beatsPerMinute = 60 / (delta / 1000.0);
  Serial.print("bpm:");
  Serial.println(beatsPerMinute);

  rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
  rateSpot %= RATE_SIZE; //Wrap variable

  //Take average of readings
  beatAvg = 0;
  for (byte x = 0 ; x < RATE_SIZE ; x++)
  {
    beatAvg += rates[x];
  }
  beatAvg /= RATE_SIZE;
  data->bpm = beatAvg * 3.0; //Multiplied by 3.0 to make the in the range of a human BPM
  if (irValue < 50000)
  {
    beatAvg = 0;
    data->bpm = 0;
  }
}

static void vGPS_Task(void* pvParameters)
{
  gpsModule.begin(9600); //gpsTx: PA10, gpsRx: PA9 
  for(;;)
  {
    if(gpsModule.available() && gps.encode(gpsModule.read()))
    {
      sensorData.lng = gps.location.lng();
      sensorData.lat = gps.location.lat();
    }   
  }
}

static void vNodeRx_Task(void* pvParameters)
{
  for(;;)
  {
    onReceive(LoRa.parsePacket());
  }
}

void onReceive(int packetSize)
{
  if (packetSize == 0) return;         
  // read packet header bytes:
  int recipient = LoRa.read();         
  byte sender = LoRa.read();           
  byte incomingLength = LoRa.read();   
  uint8_t incoming = LoRa.read();

  if (incomingLength != RX_SIZE) {   
    Serial.println("error: message length does not match length");
    return;                   
  }
  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress) {
    Serial.println("This message is not for me.");
    return;                         
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

  Get_SensorData(&sensorData);  
    
  FloatToString(sensorData.temp,txData.temp,2);
  FloatToString(sensorData.lng,txData.lng,2);
  FloatToString(sensorData.lat,txData.lat,2);
  FloatToString(sensorData.bpm,txData.bpm,2);

  strcat(dataToSend,txData.temp);
  strcat(dataToSend,",");
  strcat(dataToSend,txData.lng);
  strcat(dataToSend,",");
  strcat(dataToSend,txData.lat);
  strcat(dataToSend,",");
  strcat(dataToSend,txData.bpm);
  Serial.println(dataToSend);
  sendMessage(dataToSend);
  memset(txData.temp,'\0',strlen(txData.temp));
  memset(txData.lng,'\0',strlen(txData.lng));
  memset(txData.lat,'\0',strlen(txData.lat));
  memset(txData.bpm,'\0',strlen(txData.bpm));
  memset(dataToSend,'\0',strlen(dataToSend));   
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  if (!LoRa.begin(433E6)) {            
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                 
  }
  Serial.println("LoRa init succeeded.");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  xTaskCreate(vGPS_Task,
                "Task1",
                200,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
  xTaskCreate(vNodeRx_Task,
              "Task2",
              200,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  

}
