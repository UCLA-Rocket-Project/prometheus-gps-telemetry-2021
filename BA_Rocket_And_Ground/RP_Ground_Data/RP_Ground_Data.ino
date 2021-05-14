// this code goes on the GROUND TELEMETRY SYSTEM
// don't put it on the rocket
// I'll be mad

#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>

// sd card defs
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelectSD = 4;

//radio defs
const int chipSelectRF = 5;
const int ledPin = 8;
#define RFM95_CS 5
#define RFM95_RST 6
#define RFM95_INT 3
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// where we put the received data
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = RH_RF95_MAX_MESSAGE_LEN;

//function defs
void setupSD();
void setupRadio();
void writeOutData();
void checkForRadioTransmission();

const bool VERBOSE_DEBUG = false; // set this to true to see when things fail

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  setupSD();
  setupRadio();
}

void loop() {
  // put your main code here, to run repeatedly:
  checkForRadioTransmission();
}

void setupSD() {
  digitalWrite(chipSelectSD,HIGH);
  digitalWrite(chipSelectRF,LOW);
  
  if (!SD.begin(chipSelectSD)) {
    Serial.println("Card failed, or not present");
    digitalWrite(ledPin, LOW); //notify that there's been an error
  }
}

void setupRadio() {
  digitalWrite(chipSelectSD,LOW);
  digitalWrite(chipSelectRF,HIGH);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    digitalWrite(ledPin, LOW);
  }
  digitalWrite(ledPin, HIGH);

  rf95.setTxPower(23, false);
}

void checkForRadioTransmission() {
  digitalWrite(chipSelectSD,LOW);
  digitalWrite(chipSelectRF,HIGH);
  
  if (rf95.available())
  {
    // Should be a message for us now

    if (rf95.recv(buf, &len))
    {
      //RH_RF95::printBuffer("Received: ", buf, len);
      Serial.println((char*)buf);
      writeOutData();
//       Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);

      // we could send a reply but we're not gonna
    }
    else
    {
      // fail silently, don't wanna clog serial comms
    }
  }
}

void writeOutData() {
  digitalWrite(chipSelectSD,HIGH);
  digitalWrite(chipSelectRF,LOW);
  File dataFile = SD.open("RocketDataGround.txt", FILE_WRITE);
  if (dataFile) {
    digitalWrite(ledPin, HIGH);
    dataFile.println((char*)buf);
    dataFile.close();
  } else {
    //Serial.println("error writing to SD"); / fail silently
    digitalWrite(ledPin, LOW);
  }
}
