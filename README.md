# ESP32SingleChannelGateway
A Fork of the popular ESP8266 single channel gateway adapted to ESP32 on the 915Mhz band.

Include correct pinout to use the Heltec ESP32 Wifi/LoRa V2 boards and to get data GPS data from a NEO 6M GPS module using Tiny GPS ++.

# Instructions
- Copy the libraries to your ~/Documents/Arduino/libraries (staging folder)
- For the lora node, check the lmic config.h file make sure to enable 915Mhz band only.
