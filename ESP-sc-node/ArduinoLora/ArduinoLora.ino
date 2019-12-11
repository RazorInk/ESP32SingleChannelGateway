#include <CayenneLPP.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "SSD1306.h"
TinyGPSPlus gps;


float latitude;
float longitude;
float latitudeTwo;
float longitudeTwo;
float altitudeOne;
float altitudeTwo;
// Enter your LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { };
static const u1_t PROGMEM APPSKEY[16] = { };
// Enter your LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = {  };
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
// Schedule data trasmission in every this many seconds (might become longer due to duty
// cycle limitations).
// we set 10 seconds interval
const unsigned TX_INTERVAL = 5; // Fair Use policy of TTN requires update interval of at least several min. We set update interval here of 1 min for testing

// Pin mapping for Heltec ESP32 Wifi/LoRa V2 NOT V1
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 35, 34}, // maybe change 32 with LMIC_UNUSED_PIN
};

//TinyGPS++
#define GPS_RX 22
#define GPS_TX 23
HardwareSerial GPSSerial(2); 

CayenneLPP lpp(15);
SSD1306  display(0x3c, 4, 15);

void get_coords () {
  while (GPSSerial.available())
    gps.encode(GPSSerial.read());
  latitude  = gps.location.lat();
  Serial.print("test Latitude: ");
  Serial.println(latitude);
  longitude = gps.location.lng();
  Serial.print("test Longitude:");
  Serial.println(longitude);
  altitudeOne = gps.altitude.meters();
  // Only update if location it is valid and has changed
  if ((latitude && longitude) && latitude != latitudeTwo
      && longitude != longitudeTwo) {
    latitudeTwo = latitude;
    longitudeTwo = longitude;
    altitudeTwo = altitudeOne; 
    Serial.println("new lat and long found");
  }

}
void onEvent (ev_t ev) 
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) 
  {
    case EV_TXCOMPLETE:
      Serial.printf("EV_TXCOMPLETE (includes waiting for RX windows)\r\n");
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;  
    case EV_RXCOMPLETE:
      if (LMIC.dataLen)
      {
        Serial.printf("Received %d bytes\n", LMIC.dataLen);
      }
      break;
    default:
      Serial.printf("Unknown event\r\n");
      break;
  }
}

void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) 
  {
    Serial.printf("OP_TXRXPEND, not sending\r\n");
  } 
  else
  if (!(LMIC.opmode & OP_TXRXPEND)) 
  {
    get_coords();
    
    lpp.reset();
    lpp.addGPS(1, latitudeTwo, longitudeTwo, altitudeTwo);

    Serial.printf("GPS lat = %.2f, lon = %.2f, alt = %.2f\n", latitudeTwo, longitudeTwo, 300.0);

    display.drawString(0, 0, "Latitude: ");
    display.drawString(90, 0, String(latitudeTwo));
    display.drawString(0, 20, "Longitude  : ");
    display.drawString(90,20, String(longitudeTwo));
    display.display();    
    Serial.printf("GPS lat = %.2f, lon = %.2f, alt = %.2f\n", latitudeTwo, longitudeTwo, 300.0);

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
         
    Serial.printf("Packet queued\r\n");
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() 
{
  Serial.begin(115200);

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  GPSSerial.setTimeout(2);
  
  Serial.printf("Starting...\r\n");


  pinMode(16,OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH);
  display.init();
  display.setFont(ArialMT_Plain_10);
    
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  // Select frequencies range
  LMIC_selectSubBand(1); // original was 0
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  Serial.printf("LMIC setup done!\r\n");
  // Start job
  do_send(&sendjob);
}

void loop() 
{

  os_runloop_once();
}
