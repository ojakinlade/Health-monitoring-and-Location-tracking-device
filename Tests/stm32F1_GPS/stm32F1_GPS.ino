#include <TinyGPSPlus.h>

#define gpsModule Serial1

TinyGPSPlus gps; 
 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("GPS TEST");  
  gpsModule.begin(9600); //gpsTx: PA10, gpsRx: PA9 
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(gpsModule.available() && gps.encode(gpsModule.read()))
  {
    Serial.print("Longitude: ");
    Serial.println((int)(gps.location.lng() * 100));
    Serial.print("Latitude: ");
    Serial.println(int(gps.location.lat() * 100));
  }

}
