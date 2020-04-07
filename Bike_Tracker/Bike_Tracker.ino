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
  Some sections of code were taken from botletics MQTT example:
  https://github.com/botletics/SIM7000-LTE-Shield/blob/master/Code/examples/SIM7000_MQTT_Demo/SIM7000_MQTT_Demo.ino
  I followed this Protocol Buffer guide:
  https://www.dfrobot.com/blog-1161.html

  Written specifically to work with the Adafruit FONA 3G + GPS Breakout
  ----> https://www.adafruit.com/product/2687

  I forked the fork of Adafruit's FONA library by 'botletics' and added Network Time Sync suport.
  See https://github.com/OscarVanL/SIM7000-LTE-Shield
  I also merged this RTC helper function https://github.com/adafruit/Adafruit_FONA/pull/76
 ****************************************************/

#include "Adafruit_SleepyDog.h"
#include "Adafruit_FONA.h" // https://github.com/OscarVanL/SIM7000-LTE-Shield/tree/master/Code
//#include "Adafruit_MQTT.h"
//#include "Adafruit_MQTT_FONA.h"
#include "MQTT_Secrets.h" // Define MQTT Broker configuration
#include "proto_files/BikeTrackerPayload.pb.c" // Protocol buffer file
#include "pb_common.h" // nanopb protocol buffer implementation https://github.com/nanopb/nanopb
#include "pb.h"
#include "pb_encode.h"
#include <TimeLib.h> // Used to get epoch time
#include <SoftwareSerial.h>

// FONA Pins
#define FONA_RX 6
#define FONA_TX 5
#define FONA_RST 7
#define FONA_PS 2
#define FONA_KEY 3

// Provider APN Settings
#define FONA_APN      "TM" // Things Mobile APN

#define MAX_FAILURES 5

/****************************** FONA ***************************************/

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Setup FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
//Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

/****************************** MQTT ***************************************/

//// Setup FONA MQTT class
//Adafruit_MQTT_FONA mqtt(&fona, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS);
//// Publish to MQTT topic 'biketracker/payload' for sending location updates.
//Adafruit_MQTT_Publish payload_pub = Adafruit_MQTT_Publish(&mqtt, "biketracker/payload");
//// Subscribe to MQTT topic 'biketracker/sleep' for updating device sleep interval
//Adafruit_MQTT_Subscribe sleep_sub = Adafruit_MQTT_Subscribe(&mqtt, "biketracker/sleep");
//// Subscribe to MQTT topic 'biketracker/reboot' for triggering reboot of Arduino + FONA
//Adafruit_MQTT_Subscribe reboot_sub = Adafruit_MQTT_Subscribe(&mqtt, "biketracker/reboot");

// How long to sleep device between payloads in milliseconds
unsigned long sleep_duration = 30000;
// How long the device can be unresponsive before the watchdog resets the Arduino.
unsigned long watchdog_duration = 8000;


void setup() {

  // Watchdog for if microcontroller crashes/freezes
  Watchdog.enable(watchdog_duration);
  
  /// FONA 3G Documentation: https://learn.adafruit.com/adafruit-fona-3g-cellular-gps-breakout/pinouts
  Serial.begin(9600);
  
  // Initialise Arduino pins
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, HIGH);
  pinMode(FONA_PS, INPUT);

  FONA_power_off();
  delay(2000);
  Watchdog.reset();

  // Reboot FONA
  FONA_power_on();
  delay(2000);

  Watchdog.reset();
  
  Serial.println(F("Starting FONA Serial"));
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

  Watchdog.reset();

  // Check correct FONA connected
  uint8_t type = fona.type();
  if (type == SIM5320E) {
    Serial.println(F("SIM5320E FONA 3G in use"));
  } else {
    Serial.println(F("Incorrect FONA"));
  }

  // Wait for GSM connection
  while (1) {
    uint8_t network_status = fona.getNetworkStatus();
    if (network_status == 1 || network_status == 5) break;
    delay(1000);
  }
  Watchdog.reset();

  //fona_setup_network();

  Watchdog.reset();

  // Wait until the GPS module enables, if it never does something is wrong.
  while (1) {
    if (!fona.enableGPS(true)) {
      Serial.println(F("Failed to enable GPS"));
      delay(1000);
    } else {
      break;
    }
  }
}

bool gps_captured = false;
int gpsfailures = 0;

void loop() {
  Watchdog.reset();
  int8_t stat = fona.GPSstatus();

  if (gpsfailures < MAX_FAILURES && gps_captured == false) {
    Serial.print(F("GPS attempt no."));
    Serial.println(gpsfailures);
    
    if (stat >= 2) {
      // A 2D or 3D fix means we can report, otherwise don't report anything this time.
      float latitude, longitude, speed_kph, heading, speed_mph, altitude;
      Watchdog.reset();
      boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
      if (gps_success) {
        gps_captured = true;
        
        Watchdog.reset();
        send_payload(true, latitude, longitude, speed_kph, heading, speed_mph, altitude);
        Watchdog.reset();
      }
    } else {
      Serial.println("No GPS");
      delay(700);
      gpsfailures += 1;
    }

    Watchdog.reset();
  } else {
    if (gpsfailures >= MAX_FAILURES) {
      Serial.println(F("Failed to get GPS postition"));
      send_payload(false, 0, 0, 0, 0, 0, 0);
    } else {
      // Payload was sent successfully
      Serial.println(F("Sleeping until next reading"));
    }

    gpsfailures = 0;
    int sleepMs = sleep_device(sleep_duration);
  }

}

int txfailures = 0;

/**
 * @brief Creates and sends a payload.
 * 
 * @param gps_fix, latitude, longitude, speed_kph, heading, speed_mph, altitude GPS data
 * @return bool Success code
 */
bool send_payload(bool gps_fix, float latitude, float longitude, float speed_kph, float heading, float speed_mph, float altitude) {
  update_time();

  Serial.print(F("GPS lat:"));
  Serial.println(latitude, 6);
  Serial.print(F("GPS long:"));
  Serial.println(longitude, 6);
  Serial.print(F("Speed KPH:"));
  Serial.println(speed_kph);
  Serial.print(F("Heading:"));
  Serial.println(heading);
  Serial.print(F("Altitude:"));
  Serial.println(altitude);
  
  Serial.print(F("Creating GPS payload"));
  Serial.flush();
    
  uint8_t buffer[32];
  GpsPayload payload = GpsPayload_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  // Set GPS data
  payload.gps_fix = gps_fix;
  payload.latitude = latitude;
  payload.longitude = longitude;
  payload.speed_kph = speed_kph;
  payload.heading = heading;
  payload.altitude = altitude;
  // Set metadata
  payload.epoch = now();
  uint16_t batt_percent;
  if (fona.getBattPercent(&batt_percent)) {
    payload.battery = batt_percent;
  }
  Serial.flush();
  
  Watchdog.reset();

  // Serialise Protocol Buffer
  bool status = pb_encode(&stream, GpsPayload_fields, &payload);
  if (!status) {
    Serial.println(F("ERROR: Failed to encode protobuf"));
    return false;
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
  Serial.flush();
  

  // Attempt to connect and publish the payload until MAX_FAILURES exceeded or payload publishes successfully.
  fona_setup_network();
  while (txfailures < MAX_FAILURES) {
    MQTT_connect();
    
    char testBuff[20];
    sprintf(testBuff, "Hello World");

    //if (!fona.MQTTpublish(F("biketracker/payload"), buffer, stream.bytes_written, 1, 0)) Serial.println(F("Failed to publish"));
    if (!fona.MQTTpublish("biketracker/payload", testBuff)) Serial.println(F("Failed to publish"));
    
  }
  Watchdog.reset();
}

void fona_setup_network() {
  // Disable existing GPRS connections
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS"));
  
  fonaSerial->println(F("AT+CMEE=2"));
  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

  //Set APN
  fona.setNetworkSettings(F(FONA_APN)); // maybe calls AT+CGDCONT=1,"IP","TN"
  
  if (!fona.enableGPRS(true)) Serial.println(F("Failed to enable GPRS"));
  Serial.println(F("GPRS enabled"));
  Watchdog.reset();
  // Print the IP address
  fona.printIP();

  Serial.println("DNS Lookup");
  char MQTT_IP[17]; //"RRR.XXX.YYY.ZZZ", maximum 17 characters
  fona.DNSlookup(F(MQTT_BROKER), IP);
  Serial.print("DNS Lookup result: ");
  Serial.println(IP);

  delay(4000);
  Watchdog.reset();

  Serial.println("Opening TCP connection");

  //Open TCP connection to MQTT broker
  while (!fona.TCPconnect(MQTT_IP, MQTT_PORT)) { 
    Serial.println(F("Faild to connect to TCP/IP"));
    delay(5000);
  }
  Serial.println(F("TCP connection opened"));
  
//
//  if (!fona.wirelessConnStatus()) {
//    while (!fona.openWirelessConnection(true)) {
//      Serial.println(F("Failed to turn on 3G, retrying..."));
//      delay(1000);
//    }
//    Serial.println(F("Enabled data"));
//  } else {
//    Serial.println(F("Data already enabled"));
//  }

  
}

/**
 * @brief Reads the time from FONA's RTC and updates the Arudino's Time Library
 * This is used to get Epoch time for later transmission.
 * Note: FONA Library's readRTC function has been replaced with the unmerged one from https://github.com/adafruit/Adafruit_FONA/pull/76
 */
void update_time() {
  uint8_t year, month, day, hr, mins, sec, tz;
  fona.readRTC(&year, &month, &day, &hr, &mins, &sec, &tz);
  // Convert time to epoch for more efficient transmission
  setTime(hr, mins, sec, day, month, year);
  Serial.print(F("Epoch time:"));
  Serial.println(now());
}

/**
 * @brief Check the subscribed topics
 * Note: Not yet implemented/working
 *
 * @return bool true on success, false if a connection cannot be made
 */
//bool check_subscriptions() {
//  Adafruit_MQTT_Subscribe *subscription;
//  while ((subscription = mqtt.readSubscription(5000))) {
//    if (subscription == &sleep_sub) {
//      Serial.print(F("Updating Sleep Duration to: "));
//      //Todo: How do I read from the sleep_sub.lastread, it has type uint8_t [20]
//      Serial.println(sleep_sub.lastread[0]);
//    } else if (subscription == &reboot_sub) {
//      Serial.print(F("Triggering device reboot..."));
//      FONA_power_off();
//      Watchdog.enable(100);
//      delay(500);
//    }
//  }
//  return true;
//}

/**
 * @brief Connect / Reconnect as necessary to the MQTT server.
 * Taken from mqtt_fona example from Adafruit MQTT library.
 *
 */
void MQTT_connect() {
  // If not connected, connect to MQTT
  if (!fona.MQTTconnect("MQTT", MQTT_BROKER, MQTT_USER, MQTT_PASS)) Serial.println(F("Failed to connect to broker"));
  fona.MQTTconnect("MQTT", MQTT_BROKER, MQTT_USER, MQTT_PASS);
  
//  if (!fona.MQTT_connectionStatus()) {
//    fona.MQTT_setParameter("URL", MQTT_BROKER, MQTT_PORT);
//    fona.MQTT_setParameter("USERNAME", MQTT_USER);
//    fona.MQTT_setParameter("PASSWORD", MQTT_PASS);
//    Serial.println(F("Connecting to MQTT broker..."));
//    if (!fona.MQTT_connect(true)) {
//      Serial.println(F("Failed to connect to broker"));
//    }
//  } else {
//    Serial.println(F("Already connected to broker"));
//  }
  
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
