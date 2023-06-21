# Health-monitoring-and-Location-tracking-device
This repository contains the source code and documentation for a soldier's health and location monitoring device. The device uses LoRa communication to send health and location information to a central device, which then forwards the data to ThingSpeak for remote monitoring. The software running on the device is built on FreeRTOS to ensure uninterrupted data retrieval from the GPS module.

## Features

- Real-time monitoring of soldier health and location
- LoRa communication for long-range wireless data transmission
- Integration with ThingSpeak for remote monitoring and data visualization
- FreeRTOS-based software for reliable GPS data retrieval

## Hardware Requirements

- Soldier device:
  - STM32F103C8 Microcontroller
  - LoRa module (SX1278)
  - GPS module (NEO-6M)
  - Health sensors (MAX30100 sensor)
  - LM35 temperature sensor
  - Power supply (e.g., battery)

- Central device:
  - ESP8266 Development Board
  - LoRa module (SX1278)

## Software Requirements

- FreeRTOS
- LoRa Arduino Library
- GPS library (TinyGPSPlus)
- ThingSpeak library (ThingSpeak Arduino)

## Credits

- https://how2electronics.com/interfacing-lora-sx1278-stm32-sender-receiver/
- https://lastminuteengineers.com/max30100-pulse-oximeter-heart-rate-sensor-arduino-tutorial/
- https://how2electronics.com/interfacing-max30100-pulse-oximeter-sensor-arduino/
- https://randomnerdtutorials.com/esp8266-nodemcu-thingspeak-publish-arduino/
