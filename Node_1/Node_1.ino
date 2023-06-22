#include <MapleFreeRTOS900.h>
#include <SPI.h>  
#include <Wire.h>          
#include <LoRa_STM32.h>
#include <TinyGPSPlus.h>
#include "numeric_lib.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "button.h"

#define RX_SIZE   1
#define gpsModule Serial1

static TinyGPSPlus gps; 
static MAX30105 particleSensor;

const int csPin = PA4;          
const int resetPin = PB0;       
const int irqPin = PA1;         
const int LM35Pin = PA2;
const byte RATE_SIZE = 4;
const int BTN0 = PB3;
const int BTN1 = PB4;
const int BTN2 = PB5;
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
  char code[5];
}StrData_t;

String outgoing;              
byte localAddress = 0xFF;    
byte destination = 0xBB;      // Master address
long lastSendTime = 0;       
int interval = 2000; 
SensorData_t sensorData;        
StrData_t txData = {0};
char dataToSend[100] = {0};

Button_t btn0 = {BTN0,BUTTON_NOT_PRESSED,false}; 
Button_t btn1 = {BTN1,BUTTON_NOT_PRESSED,false}; 
Button_t btn2 = {BTN2,BUTTON_NOT_PRESSED,false}; 
BUTTON button0(&btn0);
BUTTON button1(&btn1);
BUTTON button2(&btn2);

//RTOS Handle(s)
TaskHandle_t buttonTaskHandle;
TaskHandle_t nodeTaskHandle;
QueueHandle_t buttonToNodeQueue;

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

static void vNode_Task(void* pvParameters)
{
  static char morse_code[5];
  for(;;)
  {
    if(xQueueReceive(buttonToNodeQueue,morse_code,0) == pdPASS)
    {
      Serial.println("Node task received data successfully");
      strncpy(txData.code, morse_code, sizeof(txData.code) - 1);
      txData.code[sizeof(txData.code) - 1] = '\0';
    }
    onReceive(LoRa.parsePacket());
  }
}

static void vButton_Task(void* pvParameters)
{
  int len = 0;
  static char morse_code[5] = "----";
  for(;;)
  {
    button0.Poll(&btn0);
    button1.Poll(&btn1);
    button2.Poll(&btn2);

    if(btn0.isDebounced && !btn0.prevPressed)
    {
      btn0.prevPressed = true;
      Serial.println("btn 0 pressed");
      if(len < 4)
      {
        morse_code[len++] = '0';
      } 
    }
    else if(!btn0.isDebounced && btn0.prevPressed)
    {
      btn0.prevPressed = false;
    }
    if(btn1.isDebounced && !btn1.prevPressed)
    {
      btn1.prevPressed = true;
      Serial.println("btn 1 pressed");
      if(len < 4)
      {
        morse_code[len++] = '1';
      }
    }
    else if(!btn1.isDebounced && btn1.prevPressed)
    {
      btn1.prevPressed = false;
    }
    if(btn2.isDebounced && !btn2.prevPressed)
    {
      btn2.prevPressed = true;
      Serial.println(txData.code);
      if(xQueueSend(buttonToNodeQueue,morse_code,0) == pdPASS)
      {
        Serial.println("[SUCCESS]Data sent to node task successfully");
      }
      else
      {
        Serial.println("[FAILURE]Data failed to send to node task");
      }
      len = 0;
      for(int i = 0; i < 4; i++)
      {
        morse_code[i] = '-';
      }
    }
    else if(!btn2.isDebounced && btn2.prevPressed)
    {
      btn2.prevPressed = false;
    }
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
  Serial.print("Incoming message: ");
  Serial.println(incoming);

  Serial.print("Morse code:");
  Serial.print(txData.code);

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
  strcat(dataToSend,",");
  strcat(dataToSend,txData.code);
  Serial.println(dataToSend);
  sendMessage(dataToSend);
  memset(txData.temp,'\0',sizeof(txData.temp));
  memset(txData.lng,'\0',sizeof(txData.lng));
  memset(txData.lat,'\0',sizeof(txData.lat));
  memset(txData.bpm,'\0',sizeof(txData.bpm));
  memset(dataToSend,'\0',sizeof(dataToSend));   
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  txData.code[0] = '-';
  txData.code[1] = '-';
  txData.code[2] = '-';
  txData.code[3] = '-';
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
  
  buttonToNodeQueue = xQueueCreate(1,sizeof(txData.code));
  xTaskCreate(vGPS_Task,
                "Task1",
                200,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
  xTaskCreate(vNode_Task,
              "Task2",
              200,
              NULL,
              tskIDLE_PRIORITY + 1,
              &nodeTaskHandle);
  xTaskCreate(vButton_Task,
              "Task3",
              200,
              NULL,
              tskIDLE_PRIORITY + 1,
              &buttonTaskHandle);
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  

}
