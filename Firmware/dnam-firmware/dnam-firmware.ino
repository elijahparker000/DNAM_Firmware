/*
Note from Elijah:

This is a rough overall sketch for our project. It normally sends the bme_temp, bme_humidity, and atmospheric bme_pressure data, but
when it receives the bytes "01" from the gateway, it's next send will be the GPS data. I think there may be some timing issues 
which are causing some of the messages to not go through properly, but I need to do more research. Ultimately, it works.

The code is taken from an example for this library which you will need to install: https://github.com/mcci-catena/arduino-lmic

Like the comment below says, make sure you have a device set up for OTAA on TTN
Once you do that, you need to enter the APPEUI, DEVEUI, and APPKEY from your TTN device

Note that they must be in this format: 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 (with the "0x" in front and separated by commas)

Also note that the APPEUI and DEVEUI are Little Endian, but TTN provides them as Big Endian, 
so my DEVEUI "70B3D57ED0061F66" has to have the bytes reversed to "661F06D07ED5B370" before 
being formatted with the "0x" and commas

Do not forget to define the radio type correctly in arduino-lmic/project_config/lmic_project_config.h

The correct pin definitions can be found at the beginning of the code. 
*/

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <Preferences.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <math.h>

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- PIN CONFIG ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------

// Define a struct for pin configurations
struct PinConfig {
  // GPS Module
  int gpsRX;
  int gpsTX;

  // BME280 Sensor
  int bmeSDA;
  int bmeSCL;

  // LoRa Module
  int loraNSS;
  int loraDIO0;
  int loraDIO1;
  int loraDIO2;
  int loraRST;
};

// Initialize the pin configuration
const PinConfig pinConfig = {
  // GPS Module
  .gpsRX = 4,  // RX pin for GPS
  .gpsTX = 3,  // TX pin for GPS

  // BME280 Sensor
  .bmeSDA = 21,  // SDA pin for BME280 //for bme_human reference only; this definition is not used later in code
  .bmeSCL = 22,  // SCL pin for BME280 //for bme_human reference only; this definition is not used later in code

  // LoRa Module
  .loraNSS = 5,    // NSS pin for LoRa
  .loraDIO0 = 2,   // DIO0 pin for LoRa
  .loraDIO1 = 34,  // DIO1 pin for LoRa
  .loraDIO2 = 35,  // DIO2 pin for LoRa
  .loraRST = 16    // RST pin for LoRa
};

const int analogPin = 32; // The pin where the pH sensor is connected

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- LINK LIBRARY WITH EASY NAME ----------------------------------
// -----------------------------------------------------------------------------------------------------

Preferences preferences;
int sensorIntervals[5] = {0};

static const uint32_t GPSBaud = 9600;
// The TinyGPSPlus object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(pinConfig.gpsRX, pinConfig.gpsTX);

BME280I2C bme;  // Default : forced mode, standby time = 1000 ms
                // Oversampling = bme_pressure ×1, bme_temperature ×1, bme_humidity ×1, filter off,


//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- LoRaWAN CONNECTION DETAILS -----------------------------------
// -----------------------------------------------------------------------------------------------------


// -----------------------
// ---- Andreas Login ----
// ------ down below -----
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // little endian (lsb)
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x43, 0x6D, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // little endian (lsb)
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x27, 0xB4, 0x8D, 0xDB, 0x34, 0x03, 0x0D, 0xB6, 0xA6, 0xF8, 0x5C, 0x50, 0xA1, 0x2A, 0x79, 0xEB }; // big endian (msb)
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


/*
// -----------------------
// ---- Elijah Login ----
// ------ down below -----
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // little endian (lsb)
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={ 0x66, 0x1F, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // little endian (lsb)
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x75, 0x94, 0x2F, 0xB7, 0xD7, 0xB4, 0x95, 0xD4, 0x07, 0xE4, 0x7E, 0xFA, 0x47, 0x1E, 0xC4, 0x4F }; // big endian (msb)
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
*/

/*
// -----------------------
// ---- Felix Login ----
// ------ down below -----
static const u1_t PROGMEM APPEUI[8]={ 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // little endian (lsb)
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={ 0x28, 0x6C, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // little endian (lsb)
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x25, 0x7F, 0x32, 0xCF, 0xF2, 0x88, 0x2B, 0x67, 0xD2, 0x98, 0xB2, 0xD9, 0x25, 0x99, 0xE7, 0x3E }; // big endian (msb)
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
*/

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- LORAWAN COMMUNICATION ----------------------------------------
// -----------------------------------------------------------------------------------------------------

// Pin mapping for LoRa module. This still ultimately uses the pins configured above.
const lmic_pinmap lmic_pins = {
  .nss = pinConfig.loraNSS,
  .rxtx = LMIC_UNUSED_PIN,  //not used
  .rst = pinConfig.loraRST,
  .dio = { pinConfig.loraDIO0, pinConfig.loraDIO1, pinConfig.loraDIO2 },
};

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(1);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));

              // Print each byte of the payload in hexadecimal format
              Serial.print(F("Payload: "));
              for (int i = 0; i < LMIC.dataLen; i++) {
                  if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                      Serial.print('0'); // Print leading zero for values less than 0x10
                  }
                  Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                  Serial.print(" ");
              }
              //hexToAscii(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
              Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- RECEIVED DATA PROCESSING -------------------------------------
// -----------------------------------------------------------------------------------------------------
//hexToAscii(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
void hexToAscii(const uint8_t* hexPayload, uint8_t len) {
    // Jedes Byte wird zu einem ASCII-Zeichen
    char asciiPayload[len + 1]; 
    for (int i = 0; i < len; i++) {
        // Direkte Umwandlung des Byte-Werts in ein ASCII-Zeichen
        asciiPayload[i] = (char)hexPayload[i];
    }
    asciiPayload[len] = '\0'; // Null-Terminator am Ende

    Serial.print(F("ASCII Payload: "));
    Serial.println(asciiPayload);

    processPayload(asciiPayload); // Diese Funktion muss implementiert sein
}

bool updatePush = false;
int push[5] = {0}; // Für den Fall "0;..."

void processPayload(const char* asciiPayload) {
    char* token = strtok((char*)asciiPayload, ";");

    if (token != NULL) {
        updatePush = (*token == '1');
        int index = 0; // Index für das Befüllen der Arrays

        while ((token = strtok(NULL, ";")) != NULL && index < 5) {
            if (updatePush) {
                // Verarbeite die Token als sensorIntervals, wenn updatePush true ist
                sensorIntervals[index] = atoi(token);
            } else {
                // Im Falle von "0;..." werden alle Ziffern als ein zusammenhängender String behandelt
                if (index == 0) { // Nur einmal nötig, da alle Ziffern im ersten Token nach "0;" stehen
                    int len = strlen(token);
                    for(int i = 0; i < len && i < 5; i++) {
                        push[i] = token[i] - '0';
                    }
                    break; // Breche die Schleife ab, da weitere Token für diesen Fall nicht erwartet werden
                }
            }
            index++;
        }
    }

    saveSensorIntervals();
    clearReceivedData(); // Lösche empfangene Daten

    // Debug-Ausgabe
    Serial.print("updatePush: ");
    Serial.println(updatePush ? "true" : "false");
    
    if (updatePush) {
        Serial.print("Sensor intervals: ");
        for (int i = 0; i < 5; i++) {
            Serial.print(sensorIntervals[i]);
            Serial.print(";");
        }
    } else {
        Serial.print("Push values: ");
        for (int i = 0; i < 5; i++) {
            Serial.print(push[i]);
            Serial.print(";");
        }
    }
    Serial.println();
}

void clearReceivedData() {
  // Überprüfe, ob empfangene Daten vorhanden sind
  if (LMIC.dataLen > 0) {
    // Iteriere über die gesamte Payload und setze jedes Byte zurück
    for (int i = 0; i < LMIC.dataLen; i++) {
      LMIC.frame[LMIC.dataBeg + i] = 0x00;
    }
    Serial.println(F("Received data cleared"));
  }
}

void saveSensorIntervals() {
  preferences.begin("dnam", false); // Starte Preferences im Schreibmodus

  // Speichere die neuen Werte
  for (int i = 0; i < 5; i++) {
    preferences.putInt(String(i).c_str(), sensorIntervals[i]);
  }
  Serial.println("Sucessfully saved preferences of sensors and intervals.");

  preferences.end(); // Beende die Verwendung von Preferences
}

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- SEND DATA FORMATTING -----------------------------------------
// -----------------------------------------------------------------------------------------------------

bool init_bme=0;
bool init_gps=1;

bool invalidGps = 0;
int invalidGpsCount = 0;
const int maxInvalidGpsCount = 25;

#define num_of_read 1 // number of iterations, each is actually two reads of the sensor (both directions)
const int Rx = 27200;  //fixed resistor attached in series to the sensor and ground...the same value repeated for all WM and Temp Sensor.
const long default_TempC = 24;
//const long open_resistance = 35000; //check the open resistance value by replacing sensor with an open and replace the value here...this value might vary slightly with circuit components
const long open_resistance = 50000000; //check the open resistance value by replacing sensor with an open and replace the value here...this value might vary slightly with circuit components
//const long short_resistance = 200; // similarly check short resistance by shorting the sensor terminals and replace the value here.
const long short_resistance = 3; // similarly check short resistance by shorting the sensor terminals and replace the value here.
const long short_CB = 240, open_CB = 255 ;
const float SupplyV = 3.319; // Assuming 5V output for SupplyV, this can be measured and replaced with an exact value if required
const float cFactor = 1.1; //correction factor optional for adjusting curve, 1.1 recommended to match IRROMETER devices as well as CS CR1000
int i, j = 0, WM1_CB = 0;
float SenV10K = 0, SenVWM1 = 0, SenVWM2 = 0, ARead_A1 = 0, ARead_A2 = 0, WM_Resistance = 0, WM1_Resistance = 0 ;

//pH
const float referenceVoltage = 3.3; // Adjust this if your board uses a different reference voltage

float bme_temp;
float bme_hum;
float bme_pres;
float smoisture;
float lmoisture;
float ltemp;
float ph;
float gps_lat;
float gps_lng;
float gps_alt;
float bat;

String status;

void do_send(osjob_t* j) {
  bme_temp = -1.0;
  bme_hum = -1.0;
  bme_pres = -1.0;
  smoisture = -1.0;
  lmoisture = -1.0;
  ltemp = -1.0;
  ph = -1.0;
  gps_lat = -1.0;
  gps_lng = -1.0;
  gps_alt = -1.0;
  bat = -1.0;

  //status = "11111";

  // Get bme_temp, bme_hum, bme_press
  if (sensorIntervals[0] == 1 || push[0] == 1) {
    Serial.print("[BME] ");
    sensor_bme();
    push[0] = 0;
    Serial.println(" ");
  }else{
    Serial.print("No BME; ");
  }
  // Get soil-moisture
  if (sensorIntervals[1] == 1 || push[1] == 1) {
    Serial.print("[Soil Moisture] ");
    sensor_smoisture();
    push[1] = 0;
    Serial.println(" ");
  }else{
    Serial.print("No Soil; ");
  }
  // Get leaf-moisture
  if (sensorIntervals[2] == 1 || push[2] == 1) {
    Serial.print("[Leaf Moisture] ");
    sensor_leaf();
    push[2] = 0;
    Serial.println(" ");
  }else{
    Serial.print("No Leaf; ");
  }
  // Get ph-Level
  if (sensorIntervals[3] == 1 || push[3] == 1) {
    Serial.print("[pH] ");
    sensor_ph();
    push[3] = 0;
    Serial.println(" ");
  }else{
    Serial.print("No PH; ");
  }
  // Get GPS - lat, long, alt
  if (sensorIntervals[4] == 1 || push[4] == 1 || (invalidGps == 1 && invalidGpsCount < maxInvalidGpsCount)) {
    Serial.print("[GPS] ");
    sensor_gps();
    push[1] = 0;
    Serial.println(" ");
  }else{
    Serial.println("No GPS; ... wanted ");
  }
  // Get Battery status
  sensor_bat();
  // generate Status Code
  //genStatusCode();

  char data[128];  // Stelle sicher, dass der Buffer groß genug ist.
  int len = 0;     // Verfolge die Länge des Strings.

  // Füge nur die Werte hinzu, wenn sie nicht -1.0 sind.
  //len += snprintf(data + len, sizeof(data) - len, "%s;", status.c_str());
  len += bme_temp != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", bme_temp) : snprintf(data + len, sizeof(data) - len, ";");
  len += bme_hum != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", bme_hum) : snprintf(data + len, sizeof(data) - len, ";");
  len += bme_pres != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", bme_pres) : snprintf(data + len, sizeof(data) - len, ";");
  len += smoisture != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", smoisture) : snprintf(data + len, sizeof(data) - len, ";");
  len += lmoisture != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", lmoisture) : snprintf(data + len, sizeof(data) - len, ";");
  len += ltemp != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", ltemp) : snprintf(data + len, sizeof(data) - len, ";");
  len += ph != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", ph) : snprintf(data + len, sizeof(data) - len, ";");
  len += gps_lat != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.6f;", gps_lat) : snprintf(data + len, sizeof(data) - len, ";");
  len += gps_lng != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.6f;", gps_lng) : snprintf(data + len, sizeof(data) - len, ";");
  len += gps_alt != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", gps_alt) : snprintf(data + len, sizeof(data) - len, ";");
  len += bat != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f", bat) : snprintf(data + len, sizeof(data) - len, "");

  // Sicherstellen, dass der String korrekt terminiert ist.
  Serial.print("Datastring to transfer: ");
  Serial.println(data);

  // Send sensor data
  sendData(data);
}

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- SEND DATA ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------

void sendData(char* data) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t*)data, strlen(data), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- SENSORS ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------

void sensor_bme() {
  // Read BME280 sensor data
  bme.read(bme_pres, bme_temp, bme_hum);

  char sensorData[64];
  snprintf(sensorData, sizeof(sensorData), "%.2fC,%.2f%%,%.2fPa", bme_temp, bme_hum, bme_pres);

  // Print the data before sending
  Serial.print("BME sensor data: ");
  Serial.print(sensorData);
}

void sensor_gps() {
    // Check if GPS location is valid
    if (gps.location.isValid()) {
      gps_lat = gps.location.lat();
      gps_lng = gps.location.lng();
      gps_alt = gps.altitude.meters();

      char gpsData[64];
      snprintf(gpsData, sizeof(gpsData), "%.6f,%.6f,%.6fm", gps_lat, gps_lng, gps_alt);

      // Print the data before sending
      //clearReceivedData();
      invalidGps = 0;
      invalidGpsCount = 0;
      Serial.print("GPS data: ");
      Serial.print(gpsData);
    } else {
      invalidGps = 1;
      invalidGpsCount++;
      Serial.print(F("GPS data not valid. Try Number: "));
      Serial.print(invalidGpsCount);
      // Schedule the next regular sensor data transmission
      ////os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
}

void sensor_bat() {
  // logic for Status Code
  bat = 42.0;
  //Serial.print("Battery: ");
  //Serial.println(bat);
}

void sensor_ph() {
  int sensorValue = analogRead(analogPin); // Read the value from the analog pin
  float voltage = sensorValue * (referenceVoltage / 4096.0); // Convert the value to voltage
  float pHV = (-5.6548 * voltage) + 15.509; // Calculate pH from the voltage

  if(pHV > 14){
    Serial.print("No valid pH value. Probably not connected - pH: ");
    Serial.print(pHV, 2);
    ph = -1.0;
  }else{
    // Print the voltage and pH value to the Serial Monitor
    Serial.print("Voltage: ");
    Serial.print(voltage, 4); // Print the voltage with 4 decimal places
    Serial.print(" V, pH: ");
    Serial.print(pHV, 2); // Print the pH value with 2 decimal places
    ph = pHV;
  }
}

void sensor_smoisture() {
  //Read the first Watermark sensor

  WM1_Resistance = readWMsensor();
  WM1_CB = myCBvalue(WM1_Resistance, default_TempC, cFactor);

  //*****************output************************************

  Serial.print("WM1 Resistance(Ohms)= ");
  Serial.print(WM1_Resistance);
  Serial.print("; WM1(cb/kPa)= ");
  Serial.print(abs(WM1_CB));
  Serial.print(" ");

  smoisture = WM1_CB;
}

//conversion of ohms to CB
int myCBvalue(int res, float TC, float cF) {   //conversion of ohms to CB
  int WM_CB;
  float resK = res / 1000.0;
  float tempD = 1.00 + 0.018 * (TC - 24.00);

  if (res > 550.00) { //if in the normal calibration range
    if (res > 8000.00) { //above 8k
      WM_CB = (-2.246 - 5.239 * resK * (1 + .018 * (TC - 24.00)) - .06756 * resK * resK * (tempD * tempD)) * cF;
    } else if (res > 1000.00) { //between 1k and 8k
      WM_CB = (-3.213 * resK - 4.093) / (1 - 0.009733 * resK - 0.01205 * (TC)) * cF ;
    } else { //below 1k
      WM_CB = (resK * 23.156 - 12.736) * tempD;
    }
  } else { //below normal range but above short (new, unconditioned sensors)
    if (res > 300.00)  {
      WM_CB = 0.00;
    }
    if (res < 300.00 && res >= short_resistance) { //wire short
      WM_CB = short_CB; //240 is a fault code for sensor terminal short
      Serial.print("Sensor Short WM ");
    }
  }
  if (res >= open_resistance || res==0) {

    WM_CB = open_CB; //255 is a fault code for open circuit or sensor not present

  }
  return WM_CB;
}

//read ADC and get resistance of sensor
float readWMsensor() {
  ARead_A1 = 0;
  ARead_A2 = 0;

  for (i = 0; i < num_of_read; i++) //the num_of_read initialized above, controls the number of read successive read loops that is averaged.
  {

    digitalWrite(12, HIGH);   //Set pin 12 as Vs
    delayMicroseconds(90); //wait 90 micro seconds and take sensor read
    ARead_A1 += analogRead(14); // read the analog pin and add it to the running total for this direction
    digitalWrite(12, LOW);      //set the excitation voltage to OFF/LOW

    delay(100); //0.1 second wait before moving to next channel or switching MUX

    // Now lets swap polarity, pin 12 is already low

    digitalWrite(27, HIGH); //Set pin 27 as Vs
    delayMicroseconds(90); //wait 90 micro seconds and take sensor read
    ARead_A2 += analogRead(14); // read the analog pin and add it to the running total for this direction
    digitalWrite(27, LOW);      //set the excitation voltage to OFF/LOW
  }

  SenVWM1 = ((ARead_A1 / 4096) * SupplyV) / (num_of_read); //get the average of the readings in the first direction and convert to volts
  Serial.print("First direction voltage: ");
  Serial.print(SenVWM1);
  SenVWM2 = ((ARead_A2 / 4096) * SupplyV) / (num_of_read); //get the average of the readings in the second direction and convert to volts
  Serial.print("; Second direction voltage: ");
  Serial.print(SenVWM2);
  Serial.print("; ");

  //double WM_ResistanceA = (Rx * (SupplyV - SenVWM1) / SenVWM1); //do the voltage divider math, using the Rx variable representing the known resistor
  //double WM_ResistanceA = Rx * SenVWM1 / (SupplyV - SenVWM1);  // reverse
  double WM_ResistanceA = (Rx * (1/((SupplyV/SenVWM1)-1))); //my attempt
  Serial.print("WM_ResistanceA: ");
  Serial.print(WM_ResistanceA);
  //double WM_ResistanceB = Rx * SenVWM2 / (SupplyV - SenVWM2);  // reverse
  //double WM_ResistanceB = (Rx * (SupplyV - SenVWM2) / SenVWM2); //do the voltage divider math, using the Rx variable representing the known resistor
  double WM_ResistanceB = Rx * ((SupplyV/SenVWM2)-1);  // my attempt
  Serial.print("; WM_ResistanceB: ");
  Serial.print(WM_ResistanceB);
  Serial.print("; ");
  double WM_Resistance = ((WM_ResistanceA + WM_ResistanceB) / 2); //average the two directions
  return WM_Resistance;
}

void sensor_leaf() {
  lmoisture = 33.69;
  ltemp = 42.01;
  //Serial.print("Leaf Moisture: ");
  //Serial.println(lmoisture);
}

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- STATUS HANDLING ----------------------------------------------
// -----------------------------------------------------------------------------------------------------

void genStatusCode() {
  // logic for Status Code
  status = "11011";
}

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- SETUP/LOOP ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));

    preferences.begin("dnam", false); // Starte Preferences in ReadOnly-Modus

    // Lese die gespeicherten Werte
    for (int i = 0; i < 5; i++) {
      sensorIntervals[i] = preferences.getInt(String(i).c_str(), 0); // Nutze '0' als Standardwert
    }

    // Temporary Value write
    sensorIntervals[0] = 1; //BME
    sensorIntervals[1] = 1; //Soil
    sensorIntervals[2] = 1; //Leaf
    sensorIntervals[3] = 1; //pH
    sensorIntervals[4] = 1; //GPS

    // Gebe die Werte einmalig aus
    Serial.print("saved Sensor intervals: ");
    for (int i = 0; i < 5; i++) {
      Serial.print(sensorIntervals[i]);
      Serial.print(" ");
    }
    Serial.println("");

    preferences.end(); // Beende die Verwendung von Preferences

    if (init_gps) {
      // Init of GPS Module
      ss.begin(GPSBaud);
    }

    while(!Serial) {} // Wait

    Wire.begin();

    while(!bme.begin())
    {
      Serial.println("Could not find BME280 sensor!");
      delay(1000);
    }

    switch(bme.chipModel())
    {
      case BME280::ChipModel_BME280:
        Serial.println("Found BME280 sensor! Success.");
        break;
      case BME280::ChipModel_BMP280:
        Serial.println("Found BMP280 sensor! No Humidity available.");
        break;
      default:
        Serial.println("Found UNKNOWN sensor! Error!");
    }

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // Soil Moisture
    // initialize the pins, 12 and 27 randomly chosen. In the voltage divider circuit example in figure 1(www.irrometer.com/200ss.html), pin 27 is the "Output Pin" and pin 12 is the "GND".
    // if the direction is reversed, the WM1_Resistance A and B formulas would have to be swapped.
    pinMode(12, OUTPUT);
    pinMode(27, OUTPUT);
    //set both low
    digitalWrite(12, LOW);
    digitalWrite(27, LOW);

    // pH Sensor
    analogReadResolution(12); // Set the ADC resolution to 12 bits (4096 levels)

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // US sepcific configuration
    LMIC_setAdrMode(1);
    LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

/*
This function is a little weird. Basically, it's an atbme_tempt to give both the GPS and LoRa modules enough CPU time.
It may not be necessary, and may be making the problem of dropped messages worse. I need to learn more about
the LoRaWAN protocol in general I think.
*/
void loop() {

  if (init_gps) {
    // Define a maximum number of bytes to process per loop iteration
    const int maxGPSBytesPerLoop = 10;
    int processedBytes = 0;

    // Process GPS data, but limit the number of bytes to avoid blocking
    while (ss.available() > 0 && processedBytes < maxGPSBytesPerLoop) {
      if (gps.encode(ss.read())) {
      // Optionally handle new GPS data here if needed
      }
      processedBytes++;
    }
  }

  // Call the LMIC function to handle LoRaWAN communication
  os_runloop_once();

  // (Optional) Add a small delay to prevent loop from running too fast,
  // but keep it short to ensure responsiveness
  delay(10);
}