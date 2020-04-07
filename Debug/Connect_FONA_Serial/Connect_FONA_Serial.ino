
#define FONA_RX 6
#define FONA_TX 5
#define FONA_RST 7

// Large buffer for replies
char replybuffer[255];

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

void setup() {
  while (!Serial);

  Serial.begin(9600);
  Serial.println(F("FONA Serial Passthrough"));

  fonaSerial->begin(4800);

  Serial.println(F("Creating SERIAL Passthrough to FONA"));
}


void loop() {
  while (1) {
    while (Serial.available()) {
      delay(1);
      fonaSerial->write(Serial.read());
    }
    if (fonaSerial->available()) {
      Serial.write(fonaSerial->read());
    }
  }
}
