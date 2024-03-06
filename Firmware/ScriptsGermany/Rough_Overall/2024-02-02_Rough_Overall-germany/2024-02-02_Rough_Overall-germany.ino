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

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- LoRaWAN CONNECTION DETAILS -----------------------------------
// -----------------------------------------------------------------------------------------------------

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0xB8, 0x33, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xCE, 0xD0, 0x77, 0x52, 0x32, 0x0F, 0xD0, 0x4B, 0x57, 0xF4, 0xF5, 0x4E, 0x58, 0x01, 0xF2, 0x34 };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- LORAWAN COMMUNICATION ----------------------------------------
// -----------------------------------------------------------------------------------------------------

static osjob_t sendjob;

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

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
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
            Serial.print('0');  // Print leading zero for values less than 0x10
          }
          Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        hexToAscii(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
      Serial.println((unsigned)ev);
      break;
  }
}

// -----------------------------------------------------------------------------------------------------
// -------------------------------------- RECEIVED DATA PROCESSING -------------------------------------
// -----------------------------------------------------------------------------------------------------

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

float bme_temp;
float bme_hum;
float bme_pres;
float gps_lat;
float gps_lng;
float gps_alt;
float bat;
float ph;
float smoisture;
float lmoisture;
String status;

void do_send(osjob_t* j) {
  bme_temp = -1.0;
  bme_hum = -1.0;
  bme_pres = -1.0;
  gps_lat = -1.0;
  gps_lng = -1.0;
  gps_alt = -1.0;
  bat = -1.0;
  ph = -1.0;
  smoisture = -1.0;
  lmoisture = -1.0;
  status = "11111";

  // Get bme_temp, bme_hum, bme_press
  sensor_bme();
  // Get GPS - lat, long, alt
  sensor_gps();
  // Get Battery status
  sensor_bat();
  // Get ph-Level
  sensor_ph();
  // Get soil-moisture
  sensor_smoisture();
  // Get leaf-moisture
  sensor_lmoisture();
  // generate Status Code
  genStatusCode();

  char data[128];  // Stelle sicher, dass der Buffer groß genug ist.
  int len = 0;     // Verfolge die Länge des Strings.

  // Füge Status-Code ein genStatusCode() und CleatReceiveData fehlt irgendwo
  // Füge nur die Werte hinzu, wenn sie nicht -1.0 sind.
  len += snprintf(data + len, sizeof(data) - len, "%s;", status.c_str());
  len += bme_temp != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", bme_temp) : snprintf(data + len, sizeof(data) - len, ";");
  len += bme_hum != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", bme_hum) : snprintf(data + len, sizeof(data) - len, ";");
  len += bme_pres != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", bme_pres) : snprintf(data + len, sizeof(data) - len, ";");
  len += gps_lat != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.6f;", gps_lat) : snprintf(data + len, sizeof(data) - len, ";");
  len += gps_lng != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.6f;", gps_lng) : snprintf(data + len, sizeof(data) - len, ";");
  len += gps_alt != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", gps_alt) : snprintf(data + len, sizeof(data) - len, ";");
  len += bat != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", bat) : snprintf(data + len, sizeof(data) - len, ";");
  len += ph != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", ph) : snprintf(data + len, sizeof(data) - len, ";");
  len += smoisture != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f;", smoisture) : snprintf(data + len, sizeof(data) - len, ";");
  len += lmoisture != -1.0 ? snprintf(data + len, sizeof(data) - len, "%.2f", lmoisture) : snprintf(data + len, sizeof(data) - len, "");

  // Sicherstellen, dass der String korrekt terminiert ist.
  data[len] = '\0';
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
  
  if (!init_bme) {
    Wire.begin();

    while (!bme.begin()) {
      Serial.println("Could not find BME280 sensor!");
      delay(1000);
    }

    init_bme=1;
  }

  switch (bme.chipModel()) {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No bme_humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }

  // Read BME280 sensor data
  bme.read(bme_pres, bme_temp, bme_hum);

  char sensorData[64];
  snprintf(sensorData, sizeof(sensorData), "%.2fC,%.2f%%,%.2fPa", bme_temp, bme_hum, bme_pres);

  // Print the data before sending
  Serial.print("BME sensor data: ");
  Serial.println(sensorData);
}

void sensor_gps() {
  // Check if we received "01" from the gateway to send GPS data
  if (push[1] == 1 || (invalidGps == 1 && invalidGpsCount < maxInvalidGpsCount)) {
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
      Serial.println(gpsData);
    } else {
      invalidGps = 1;
      invalidGpsCount++;
      Serial.print(F("GPS data not valid. Try Number: "));
      Serial.println(invalidGpsCount);
      // Schedule the next regular sensor data transmission
      ////os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
    push[1] = 0;
  }else{
    Serial.println("No GPS data wanted");
  }
}

void sensor_bat() {
  // logic for Status Code
  bat = 42.0;
  //Serial.print("Battery: ");
  //Serial.println(bat);
}

void sensor_ph() {
  ph = 5.0;
  //Serial.print("ph-Level: ");
  //Serial.println(ph);
}
void sensor_smoisture() {
  smoisture = 40.2;
  //Serial.print("Soil Moisture: ");
  //Serial.println(smoisture);
}
void sensor_lmoisture() {
  lmoisture = 33.69;
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

  while (!Serial) {}  // Wait

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

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