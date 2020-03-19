/***************************************************
  This is a Coursework project for COMP3210 Advanced Networks.

  It uses the FONA 3G along with a Things Network SIM to get GPS data and
  transmit it back via MQTT periodically.
  The Things Network SIM allows this to work in practically any country, making it even track
  cross-border.

  Some sections of code were taken from the FONAtest example and an existing simialr project:
  https://github.com/lectroidmarc/gsm-tracker/blob/master/gsm_tracker/gsm_tracker.ino

  Written specifically to work with the Adafruit FONA 3G + GPS Breakout
  ----> https://www.adafruit.com/product/2687
 ****************************************************/


#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#define FONA_RX 6
#define FONA_TX 5
#define FONA_RST 7
#define FONA_PS 2
#define FONA_KEY 3

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t type;


void setup() {
  /// FONA 3G Documentation: https://learn.adafruit.com/adafruit-fona-3g-cellular-gps-breakout/pinouts
  Serial.begin(115200);
  Serial.println(F("Arduino started"));

  // Power up FONA if it's off
  pinMode(FONA_KEY OUTPUT);
  digitalWrite(FONA_KEY, HIGH);
  pinMode(FONA_PS, INPUT)
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

  whi


  
  


}

void loop() {
  // put your main code here, to run repeatedly:

}
