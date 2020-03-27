/***************************************************
  This is a Coursework project for COMP3210 Advanced Networks.

  It uses the FONA 3G along with a Things Network SIM to get GPS data and
  transmit it back via MQTT periodically.
  The Things Network SIM allows this to work in practically any country, making it even track
  cross-border.

  Some sections of code were taken from the FONAtest example and an existing simialar project:
  https://github.com/lectroidmarc/gsm-tracker/blob/master/gsm_tracker/gsm_tracker.ino
  Some sections of code were taken from the Adafruit MQTT Library examples: 
  https://github.com/adafruit/Adafruit_MQTT_Library/tree/master/examples/mqtt_fona

  Written specifically to work with the Adafruit FONA 3G + GPS Breakout
  ----> https://www.adafruit.com/product/2687
 ****************************************************/

#include "Adafruit_SleepyDog.h"
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include "MQTT_Secrets.h"
#include <SoftwareSerial.h>



// FONA Pins
#define FONA_RX 6
#define FONA_TX 5
#define FONA_RST 7
#define FONA_PS 2
#define FONA_KEY 3

// MQTT
#define MQTT_TOPIC = "/bike-tracker"

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Setup FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

// Setup FONA MQTT class
Adafruit_MQTT_FONA mqtt(&fona, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS);

uint8_t type;


void setup() {
  
  /// FONA 3G Documentation: https://learn.adafruit.com/adafruit-fona-3g-cellular-gps-breakout/pinouts
  Serial.begin(115200);
  Serial.println(MQTT_BROKER);
  Serial.println(F("Arduino started"));

  // Power up FONA if it's off
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, HIGH);
  pinMode(FONA_PS, INPUT);
  if (digitalRead(FONA_PS) == LOW) {
    Serial.println(F("Powering FONA on"));
    while (digitalRead(FONA_PS) == LOW) {
      digitalWrite(FONA_KEY, LOW);
      delay(500);
    }
    digitalWrite(FONA_KEY, HIGH);
    Serial.println(F("FONA Started"));
  }
  
  
  Serial.println(F("Initialising FONA Serial..."));
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

  // Check correct FONA connected
  type = fona.type();
  if (type == FONA3G_E) {
    Serial.println(F("FONA 3G (European) in use"));
  } else {
    Serial.println(F("Incorrect FONA in use"));
  }

  // Wait for GSM connection
  Serial.print(F("Waiting for GSM network..."));
  while (1) {
    uint8_t network_status = fona.getNetworkStatus();
    if (network_status == 1 || network_status == 5) break;
    delay(250);
  }

  
  while (1) {
    if (!fona.enableGPS(true)) {
      Serial.println(F("Failed to turn on GPS"));
      delay(300);
    } else {
      Serial.println(F("GPS Enabled"));
      int8_t stat;
      stat = fona.GPSstatus();
      if (stat < 0) Serial.println(F("Failed to query"));
      if (stat == 0) Serial.println(F("GPS Status: GPS off"));
      if (stat == 1) Serial.println(F("GPS Status: No fix"));
      if (stat == 2) Serial.println(F("GPS Status: 2D fix"));
      if (stat == 3) Serial.println(F("GPS Status: 3D fix"));
  
      // check for GPS location
      char gpsdata[120];
      fona.getGPS(0, gpsdata, 120);
      Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
      Serial.println(gpsdata);
      break;
    }

  }
}

void loop() {
  // put your main code here, to run repeatedly:

  int8_t stat;
  stat = fona.GPSstatus();
  
  if (stat == 0) Serial.println(F("GPS Status: GPS off"));
  if (stat == 1) Serial.println(F("GPS Status: No fix"));
  if (stat == 2) Serial.println(F("GPS Status: 2D fix"));
  if (stat == 3) Serial.println(F("GPS Status: 3D fix"));

//  if (stat >= 2) {
//    // A 2D or 3D fix means we can report, otherwise don't report anything this time.
//    boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)
//    if (gps_success) {
//      Serial.println
//    }
//    
//    
//
//    // Todo: Take a GPS lat/long reading, report it via MQTT, then sleep FONA and then this Arduino.
//  }
  

  delay(1000);

}
