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
#include "protobuf/BikeTrackerPayload.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include <SoftwareSerial.h>

// FONA Pins
#define FONA_RX 6
#define FONA_TX 5
#define FONA_RST 7
#define FONA_PS 2
#define FONA_KEY 3

/****************************** FONA ***************************************/

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Setup FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

/****************************** MQTT ***************************************/

// Setup FONA MQTT class
Adafruit_MQTT_FONA mqtt(&fona, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS);
// Publish to MQTT topic 'biketracker/payload' for sending location updates.
Adafruit_MQTT_Publish payload_pub = Adafruit_MQTT_Publish(&mqtt, "biketracker/payload");
// Subscribe to MQTT topic 'biketracker/sleep' for updating device sleep interval
Adafruit_MQTT_Subscribe sleep_sub = Adafruit_MQTT_Subscribe(&mqtt, "biketracker/sleep");
// Subscribe to MQTT topic 'biketracker/reboot' for triggering reboot of Arduino + FONA
Adafruit_MQTT_Subscribe reboot_sub = Adafruit_MQTT_Subscribe(&mqtt, "biketracker/reboot");

// How long to sleep device between payloads in milliseconds
long sleep_duration = 60000


void setup() {
  // Watchdog for if microcontroller crashes/freezes
  Watchdog.enable(8000);
  
  /// FONA 3G Documentation: https://learn.adafruit.com/adafruit-fona-3g-cellular-gps-breakout/pinouts
  Serial.begin(115200);
  Serial.println(MQTT_BROKER);
  Serial.println(F("Arduino started"));

  
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, HIGH);
  pinMode(FONA_PS, INPUT);

  // Power up FONA if it's off
  FONA_power_on()

  Watchdog.reset();
  
  Serial.println(F("Initialising FONA Serial..."));
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

  Watchdog.reset();

  // Check correct FONA connected
  uint8_t type = fona.type();
  if (type == FONA3G_E) {
    Serial.println(F("FONA 3G (European) in use"));
  } else {
    Serial.println(F("Incorrect FONA in use"));
  }

  mqtt.subscribe(&sleep_sub);
  mqtt.subscribe(&reboot_sub);

  // Wait for GSM connection
  Serial.print(F("Waiting for GSM network..."));
  while (1) {
    uint8_t network_status = fona.getNetworkStatus();
    if (network_status == 1 || network_status == 5) break;
    delay(250);
  }

  Watchdog.reset();
  
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
  Watchdog.reset();
  
  int8_t stat = fona.GPSstatus();
  
  if (stat == 0) Serial.println(F("GPS Status: GPS off"));
  if (stat == 1) Serial.println(F("GPS Status: No fix"));
  if (stat == 2) Serial.println(F("GPS Status: 2D fix"));
  if (stat == 3) Serial.println(F("GPS Status: 3D fix"));

  int failed_gps_retries = 5;

  if (stat >= 2) {
    // A 2D or 3D fix means we can report, otherwise don't report anything this time.
    float latitude, longitude, speed_kph, heading, speed_mph, altitude;
    int attempt = 0;

    if (attempt < failed_gps_retries) {
      Watchdog.reset();
      boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)
      if (gps_success) {
        Serial.print(F("GPS lat:"));
        Serial.println(latitude, 6);
        Serial.print(F("GPS long:"));
        Serial.println(longitude, 6);
        Serial.print(F("GPS speed KPH:"));
        Serial.println(speed_kph);
        Serial.print(F("GPS heading:"));
        Serial.println(heading);
        Serial.print(F("GPS altitude:"));
        Serial.println(altitude);

        Serial.print(F("Publishing GPS payload"));
        uint8_t buffer[128];
        BikeTrackerPayload payload = BikeTrackerPayload_init_zero;
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        payload.latitude = latitude;
        payload.longitude = longitude;
        // Todo:
        payload.epoch = 0;
        payload.battery = fona.getBattPercent();
        payload.speed_kph = speed_kph;
        pyload.heading = heading;
        payload.altitude = altitude;
        
        payload_pub.publish("Todo");
        
      } else {
        sleep (300);
        attempt += 1
      }
    } else {
      Serial.println("Failed to get GPS position 5 times in a row, skipping this GPS reading");
    }

    
    

    // Todo: Take a GPS lat/long reading, report it via MQTT, then sleep FONA and then this Arduino.
  }

  Watchdog.reset();  
  // Wait for incoming subscriptions
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &sleep_sub) {
      Serial.print(F("Updating Sleep Duration to: "));
      Serial.println(sleep_sub.lastread);
    } else if (subscription == &reboot_sub) {
      Serial.print(F("Triggering device reboot..."));
      FONA_power_off()
      Watchdog.enable(100);
      delay(500);
    }
  }
  

  delay(1000);

}

// Function to switch FONA on
void FONA_power_on() {
  // Check FONA is actually off by reading Power State
  if (digitalRead(FONA_PS) == LOW) {
    Serial.println(F("Powering FONA on"));
    while (digitalRead(FONA_PS) == LOW) {
      digitalWrite(FONA_KEY, LOW);
      delay(500);
    }
    digitalWrite(FONA_KEY, HIGH);
    Serial.println(F("FONA Started"));
  } else {
    Serial.println(F("FONA is already on"));
  }
}

// Function to switch FONA off
void FONA_power_off() {
  // Check FONA is actually on by reading Power State
  if (digitalRead(FONA_PS) == HIGH) {
    Serial.println(F("Powering FONA off"));
    while (digitalRead(FONA_PS) == HIGH) {
      digitalWrite(FONA_KEY, LOW);
      delay(500);
    }
    digitalWrite(FONA_KEY, HIGH);
    Serial.println(F("FONA Powered Off"));
  } else {
    Serial.println(F("FONA is already off"));
  }
}
