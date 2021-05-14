// this code goes on the ROCKET
// don't put it on the ground telemetry system
// I'll be mad

#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include "string.h"

// radio:
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#define RF95_FREQ 434.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// gps/cellular:
//#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#define FONA_RX     11
#define FONA_TX     9
#define FONA_RST    10
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
//// cellular apn:
//#define FONA_APN       "hologram"
//#define FONA_USERNAME  ""
//#define FONA_PASSWORD  ""
//// connecting to adafruit io for the web view dashboard (these are my credentials)
//#define AIO_SERVER      "io.adafruit.com"
//#define AIO_SERVERPORT  1883
//#define AIO_USERNAME    "b_ank"
//#define AIO_KEY         "aio_bTFg80doeRLcLe3zDBhvLDkqDIEL"
////                      ^ this is sensitive, please don't mess with it
//// some fun internet stuff (don't touch):
//// Store the MQTT server, client ID, username, and password in flash memory.
//// This is required for using the Adafruit MQTT library.
//const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
//// Set a unique MQTT client ID using the AIO key + the date and time the sketch
//// was compiled (so this should be unique across multiple devices for a user,
//// alternatively you can manually set this to a GUID or other random value).
//const char MQTT_CLIENTID[] PROGMEM  = __TIME__ AIO_USERNAME;
//const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
//const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;
//Adafruit_MQTT_FONA mqtt(&fona, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);
//
//boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);
//
//const char GPSLOC_FEED[] PROGMEM = AIO_USERNAME "/feeds/gps/csv";
//Adafruit_MQTT_Publish gpsloc = Adafruit_MQTT_Publish(&mqtt, GPSLOC_FEED);
//char gpsbuffer[120];

#define halt(s) { Serial.println(F( s )); while(1);  }

// SD and sensors:
OpenLog myLog; //Create instance
Adafruit_LSM6DSOX sox;

void createNewFile();
void lsm6doxSetup();
void radioSetup();
void openLogSetup();
void lsm6doxLoop();
void recordDataToSD();
void sendDataViaRadio();
void getDataString();
void sendDataViaCellular();
void fonaSetup();
void gpsLoop();
void connect();

const byte OpenLogAddress = 42; //Default Qwiic OpenLog I2C address

// everything written to the log file or sent to the ground is stored in these global vars. This is so that, if need be, we can update everything at different rates.
unsigned long elapsedTime = 0;
float accelX = 1.23;
float accelY = 1.23;
float accelZ = 1.23;
float shockX = 1.23;
float shockY = 1.23;
float shockZ = 1.23;
float gyroX = 1.23;
float gyroY = 1.23;
float gyroZ = 1.23;
float altimeterHeight = 1.23;
float gpsLat = 1.23;
float gpsLong = 1.23;
float temperature = 1.23;
char* latp;
char* longp;

char outputStr[RH_RF95_MAX_MESSAGE_LEN] = ""; // as long as we can send

uint8_t tick = 0; // range: 0 to 255. Rolls over.
// makes sure we only send data when we want to.

void setup()
{
  Serial.begin(115200);
  //Serial.println(F("this happened11!"));
  openLogSetup();
  lsm6doxSetup();
  radioSetup();
  fonaSetup(); // takes a while
}

void loop()
{
  // this runs at roughly 2hz
  elapsedTime = millis();
  lsm6doxLoop();
  gpsLoop();
  recordDataToSD();
  sendDataViaRadio();
  if (tick % 5 == 0) {
    //sendDataViaCellular(); // update cellular every 2.5s
    // ^ WARNING! for memory reasons, this nukes the stored data.
    // must always run last.
  }
  tick++;
}

void radioSetup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    //Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  //Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    //Serial.println("setFrequency failed");
    while (1);
  }
  
  rf95.setTxPower(23, false); // maximum power. second arg must be false
}

void openLogSetup() {
  Wire.begin();
  myLog.begin();
}

void fonaSetup() {
//  //Serial.println(F("Adafruit FONA GPS MQTT demo"));
//
//// this section
// first, tries the baudrate you think the modem is set to (115200), then resets it to what you want (9600). 
// but you actually want 9600. I did it for you.
  //Serial.println(F("First trying 115200 baud"));
  // start at 115200 baud
  fonaSerial->begin(115200);
  fona.begin(*fonaSerial);
  
  // send the command to reset the baud rate to 9600 
  fona.setBaudrate(9600); 
  
  // restart with 9600 baud
  fonaSerial->begin(9600);
  //Serial.println(F("Initializing @ 4800 baud..."));
  
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));

//  if (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD)))
//    halt("Retrying FONA");

  //Serial.println(F("Connected to Cellular!"));

  

  

//  //Watchdog.reset();
  delay(10000);  // wait a few seconds to stabilize connection
  
  if (fona.enableGPS(true) == false) {
    Serial.println(F("gps is unwell"));
  }
//  //Watchdog.reset();
//
//  // connect to adafruit io
//  connect();
}

// don't need this. Appending to a file creates a new file if none exists.
void createNewFile() {

  //Serial.println("Run OpenLog New File"); //Goes to terminal
  //myLog.println("Run OpenLog New File"); //Goes to the default LOG#.txt file

  myLog.create(F("RocketData.txt"));
  myLog.syncFile();
}

void lsm6doxSetup() {
//  while (!Serial)
//    delay(10); // will pause Zero, Leonardo, etc until serial console opens\

  if (!sox.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  //Serial.println("LSM6DSOX Found!");

  // accelerometer data rates and such follow:
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G); //2, 4, 8, 16

  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS ); // 125, 250, 500, 1000, 2000

  sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ); // 0, 12.5, 26, 52, 104, 208, 416, 833, 1.66k, 3.33k, 6.66k Hz

  sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ); // 0, 12.5, 26, 52, 104, 208, 416, 833, 1.66k, 3.33k, 6.66k Hz
}


void lsm6doxLoop() {
  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  // store to our variables
  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
  accelZ = accel.acceleration.z;

  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;

  temperature = temp.temperature;
  
}

void sendDataViaRadio() {
  getDataString();

  rf95.send((uint8_t *)outputStr, sizeof(outputStr));
  //rf95.waitPacketSent(); // we might want to remove this, but that could be dumb...
  //Serial.println(outputStr);
  // there's a provision in the example to wait for a reply, but we aren't going to bother.
  // that would waste bandwidth we don't have.
  // better to just fire messages into the void and hope they're  
  // received than to miss new data while resending old messages.
}

void recordDataToSD() {
  myLog.append(F("RocketData.txt"));
  getDataString(); // writes current data to outputStr
  myLog.println(outputStr);
  myLog.syncFile();
  
}


// cellular bs:
void sendDataViaCellular() { // eventually this will do what it says. For now it just updates gps.
  //char sendbuffer[120];

  // Make sure to reset watchdog every loop iteration!
  //Watchdog.reset();

//  // ping adafruit io a few times to make sure we remain connected
//  if(! mqtt.ping(3)) {
//    // reconnect to adafruit io
//    if(! mqtt.connected())
//      connect();
//  }

  float latitude, longitude, speed_kph, heading, speed_mph, altitude;
  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude, NULL, NULL, NULL/*&speed_kph, &heading*/);
  altitude = 0;

  if (gps_success) {

//    Serial.print("GPS lat:");
//    Serial.println(latitude, 6);
//    Serial.print("GPS long:");
//    Serial.println(longitude);
//    Serial.print("GPS speed KPH:");
//    Serial.println(speed_kph);
//    Serial.print("GPS speed MPH:");
//    speed_mph = speed_kph * 0.621371192;
//    Serial.println(speed_mph);
//    Serial.print("GPS heading:");
//    Serial.println(heading);
//    Serial.print("GPS altitude:");
//    Serial.println(altitude);

    // snprintf(sendbuffer, 120, "%d,%f,%f,0", x, latitude, longitude);
    // but that doesnt work in arduino
//    char *p = sendbuffer;
//    // add speed value
//    dtostrf(speed_mph, 2, 6, p);
//    p += strlen(p);
//    p[0] = ','; p++;
//
//    // concat latitude
    gpsLat = latitude;
    //Serial.println(latitude);
//    dtostrf(latitude, 2, 6, p);
//    p += strlen(p);
//    p[0] = ','; p++;
//
//    // concat longitude
    gpsLong = longitude;
    //Serial.println(longitude);
//    dtostrf(longitude, 3, 6, p);
//    p += strlen(p);
//    p[0] = ','; p++;
//
//    // concat altitude
//    dtostrf(altitude, 2, 6, p);
//    p += strlen(p);
//
//    // null terminate
//    p[0] = 0;

    //Serial.print("Sending: "); Serial.println(sendbuffer);

//    if (! gpsloc.publish(sendbuffer)) {
//      Serial.println(F("Failed"));
//      //txfailures++;
//    } else {
//      //Serial.println(F("OK!"));
//      //txfailures = 0;
//    }

    //Watchdog.reset();

  }

  // wait a couple seconds before starting over
  //Watchdog.reset();
}

//void connect() {
//
//  // check if we're still connected
//  // and make sure there aren't a bunch
//  // of send failures
//  if(fona.TCPconnected() /*&& txfailures < MAXTXFAILURES*/)
//    return;
//
//  //Serial.println(F("Connecting to MQTT..."));
//
//  int8_t ret, retries = 5;
//
//  while (retries && (ret = mqtt.connect()) != 0) {
////    switch (ret) {
////      case 1: Serial.println(F("Wrong protocol")); break;
////      case 2: Serial.println(F("ID rejected")); break;
////      case 3: Serial.println(F("Server unavail")); break;
////      case 4: Serial.println(F("Bad user/pass")); break;
////      case 5: Serial.println(F("Not authed")); break;
////      case 6: Serial.println(F("Failed to subscribe")); break;
////      default: Serial.println(F("Connection failed")); break;
////    }
//
//    //Serial.println(F("Retrying MQTT connection"));
//    retries--;
//    //if (retries == 0) halt("Resetting system");
//    //delay(5000);
//
//  }
//
//  //Serial.println(F("MQTT Connected!"));
//
//}

void gpsLoop() { // this WILL update GPS without sending it anywhere.
  sendDataViaCellular();
}

void getDataString() {
  //sprintf(outputStr, "%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%lu",xx(accelX),xx(accelY),xx(accelZ),xx(shockX),xx(shockY),xx(shockZ),xx(gyroX),xx(gyroY),xx(gyroZ),xx(altimeterHeight),xxxx(gpsLat),xxxx(gpsLong),xx(temperature),elapsedTime);
  //sprintf(outputStr, "%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%lu",xx(accelX),xx(accelY),xx(accelZ),xx(shockX),xx(shockY),xx(shockZ),xx(gyroX),xx(gyroY),xx(gyroZ),xx(altimeterHeight),xxxx(gpsLat),xxxx(gpsLong),xx(temperature),elapsedTime);
  // switching up the order! most important stuff goes first.
  // time, gps lat, long, altimeter height, temp, accel x,y,z, gyro x,y,z, shock x,y,z
  
  char* p = outputStr;
  ultoa(elapsedTime, p, 10); // last change was this
  p += strlen(p);
  p[0] = ','; p++;

  dtostrf(gpsLat, 9, 4, p);
  //strcat(p, latp);
  p += strlen(p);
  p[0] = ','; p++;

  dtostrf(gpsLong, 9, 4, p);
  //strcat(p, longp);
  p += strlen(p);
  p[0] = ','; p++;

  dtostrf(altimeterHeight, 3, 1, p); //8
  p += strlen(p);
  p[0] = ','; p++;

  dtostrf(temperature, 3, 2, p); //5
  p += strlen(p);
  p[0] = ','; p++;

  dtostrf(accelX, 3, 1, p); //5
  p += strlen(p);
  p[0] = ','; p++;
  dtostrf(accelY, 3, 1, p); //etc
  p += strlen(p);
  p[0] = ','; p++;
  dtostrf(accelZ, 3, 1, p);
  p += strlen(p);
  p[0] = ','; p++;

  dtostrf(gyroX, 3, 1, p);
  p += strlen(p);
  p[0] = ','; p++;
  dtostrf(gyroY, 3, 1, p);
  p += strlen(p);
  p[0] = ','; p++;
  dtostrf(gyroZ, 3, 1, p);
  p += strlen(p);
  p[0] = ','; p++;

  dtostrf(shockX, 3, 1, p);
  p += strlen(p);
  p[0] = ','; p++;
  dtostrf(shockY, 3, 1, p);
  p += strlen(p);
  p[0] = ','; p++;
  dtostrf(shockZ, 3, 1, p);
  p += strlen(p);

  p[0] = 0;
}

int xx(const float &n) {
  return round(n*100);
}
int xxxx(const float &n) {
  return round(n*10000);
}
