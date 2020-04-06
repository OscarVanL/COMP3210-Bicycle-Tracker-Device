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
  I followed this Protocol Buffer guide:
  https://www.dfrobot.com/blog-1161.html

  Written specifically to work with the Adafruit FONA 3G + GPS Breakout
  ----> https://www.adafruit.com/product/2687

  I modified the Adafruit FONA Library to add Network Time Sync support for the FONA 3G, which
  was not previously supported. See https://github.com/adafruit/Adafruit_FONA/pull/115
  I also merged this RTC helper function https://github.com/adafruit/Adafruit_FONA/pull/76
 ****************************************************/

#include "Adafruit_SleepyDog.h"
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include "MQTT_Secrets.h"
#include "proto_files/BikeTrackerPayload.pb.c"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include <TimeLib.h>
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
unsigned long sleep_duration = 60000;
// How long the device can be unresponsive before the watchdog resets the Arduino.
unsigned long watchdog_duration = 8000;


void setup() {
  // Watchdog for if microcontroller crashes/freezes
  Watchdog.enable(watchdog_duration);
  
  /// FONA 3G Documentation: https://learn.adafruit.com/adafruit-fona-3g-cellular-gps-breakout/pinouts
  Serial.begin(9600);
  Serial.println(F("Arduino started"));
  
  // Initialise Arduino pins
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, HIGH);
  pinMode(FONA_PS, INPUT);

  // Power up FONA if it's off
  FONA_power_on();

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

  // Subscribe to relevant topics
  mqtt.subscribe(&sleep_sub);
  mqtt.subscribe(&reboot_sub);

  // Wait for GSM connection
  Serial.print(F("Waiting for GSM network..."));
  while (1) {
    uint8_t network_status = fona.getNetworkStatus();
    if (network_status == 1 || network_status == 5) break;
    delay(1000);
  }

  Watchdog.reset();

  // Wait until the GPS module enables, if it never does something is wrong.
  while (1) {
    if (!fona.enableGPS(true)) {
      Serial.println(F("Failed to turn on GPS"));
      delay(1000);
    } else {
      break;
    }
  }
}

int failed_gps_retries = 5;
bool gps_captured = false;
int attempt = 0;

void loop() {
  Watchdog.reset();
  int8_t stat = fona.GPSstatus();
  
//  if (stat == 0) Serial.println(F("GPS Status: GPS off"));
//  if (stat == 1) Serial.println(F("GPS Status: No fix"));
//  if (stat == 2) Serial.println(F("GPS Status: 2D fix"));
//  if (stat == 3) Serial.println(F("GPS Status: 3D fix"));

  if (attempt < failed_gps_retries && gps_captured == false) {
    Serial.print(F("Attempt no. "));
    Serial.println(attempt);
    
    if (stat >= 2) {
      // A 2D or 3D fix means we can report, otherwise don't report anything this time.
      float latitude, longitude, speed_kph, heading, speed_mph, altitude;
      Watchdog.reset();
      boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
      if (gps_success) {
        gps_captured = true;
        // FONA Library's readRTC function has been replaced with the unmerged one from https://github.com/adafruit/Adafruit_FONA/pull/76
        uint8_t year, month, day, hr, mins, sec, tz;
        fona.readRTC(&year, &month, &day, &hr, &mins, &sec, &tz);
        // Convert time to epoch for more efficient transmission
        setTime(hr, mins, sec, day, month, year);
        
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
        Serial.print(F("Time:"));
        Serial.print(year);
        Serial.print(F("/"));
        Serial.print(month);
        Serial.print(F("/"));
        Serial.print(day);
        Serial.print(F("-"));
        Serial.print(hr);
        Serial.print(F(":"));
        Serial.print(mins);
        Serial.print(F(":"));
        Serial.print(sec);
        Serial.print(F("+"));
        Serial.println(tz);
        Serial.print(F("Epoch time:"));
        Serial.println(now());
        
        Watchdog.reset();
        
        Serial.print(F("Creating GPS payload"));
        uint8_t buffer[128];
        GpsPayload payload = GpsPayload_init_zero;
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        payload.gps_fix = true;
        payload.latitude = latitude;
        payload.longitude = longitude;
        payload.epoch = now();
        uint16_t batt_percent;
        if (fona.getBattPercent(&batt_percent)) {
          payload.battery = batt_percent;
        }
        payload.speed_kph = speed_kph;
        payload.heading = heading;
        payload.altitude = altitude;

        // Serialise Protocol Buffer
        bool status = pb_encode(&stream, GpsPayload_fields, &payload);
        if (!status) {
          Serial.println(F("ERROR: Failed to encode protocol buffer"));
        } else {
          Serial.print(F("Message Length: "));
          Serial.println(stream.bytes_written);

          Serial.print(F("Message: "));
          for(int i = 0; i<stream.bytes_written; i++){
            char tmp[16];
            sprintf(tmp, "%02X", buffer[i]);
            Serial.print(tmp);
          }
        }
        
        Watchdog.reset();
        //payload_pub.publish("Todo");
        
      }
    } else {
      Serial.println("No GPS");
      delay(700);
      attempt += 1;
    }

    Watchdog.reset();
  } else {
    if (attempt >= failed_gps_retries) {
      Serial.println(F("Failed to get GPS position 5 times in a row, skipping this GPS reading"));
    } else {
      Serial.println(F("Sleeping until next reading"));
    }

    attempt = 0;
    int sleepMs = sleep_device(sleep_duration);
  }

}

/**
 * @brief Sleep the arduino and FONA, try to sleep for the amount of ms specified, and return the time actually
 * slept for.
 * FONA will be turned back on once sleep is over
 *
 * @param ms Milliseconds to aim to sleep for 
 * @return int Time actually slept for in milliseconds
 */
int sleep_device(unsigned long ms) {
  Serial.print(F("Sleeping for: "));
  Serial.println(ms);
  Serial.flush();
  // Set the Watchdog to allow the sleep duration.
  Watchdog.reset();
  Serial.println(F("Turning FONA off"));
  FONA_power_off();
  Serial.println(F("Sleeping Arduino"));
  Serial.flush();

  // 4000ms is the most Watchdog can sleep in one go
  unsigned long sleepMS = 0;
  if (ms > 4000) {
    int sleep_num = ms / 4000;
    Watchdog.reset();
    for(int i=0;i<sleep_num;i++) {
      Watchdog.reset();
      sleepMS += Watchdog.sleep(4000);
    }
    unsigned long extra = ms % 4000;
    sleepMS += Watchdog.sleep(extra);
  } else {
    sleepMS = Watchdog.sleep(ms);
  }
  
  Watchdog.reset();
  FONA_power_on();

  Serial.print(F("Device slept for: "));
  Serial.println(sleepMS);
  Serial.flush();
  Serial.println(F("Arduino + FONA awake"));
  Serial.flush();
  return sleepMS;
}

/**
 * @brief Check the subscribed topics
 * Note: Not yet implemented/working
 *
 * @return bool true on success, false if a connection cannot be made
 */
bool check_subscriptions() {
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &sleep_sub) {
      Serial.print(F("Updating Sleep Duration to: "));
      //Todo: How do I read from the sleep_sub.lastread, it has type uint8_t [20]
      Serial.println(sleep_sub.lastread[0]);
    } else if (subscription == &reboot_sub) {
      Serial.print(F("Triggering device reboot..."));
      FONA_power_off();
      Watchdog.enable(100);
      delay(500);
    }
  }
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
    digitalWrite(FONA_KEY, LOW);
    delay(1000);
    digitalWrite(FONA_KEY, HIGH);
    Serial.println(F("FONA Powered Off"));
  } else {
    Serial.println(F("FONA is already off"));
  }
}
