#include <HardwareSerial.h>

#define DE_RE_PIN 19 // DE and RE pins of MAX485 module connected together to GPIO 19
#define RX_PIN 9     // RX pin of ESP32 connected to RO of MAX485
#define TX_PIN 10    // TX pin of ESP32 connected to DI of MAX485
#define BAUD_RATE 9600 // Baud rate for RS485 communication

// Initialize the second hardware serial port on ESP32
HardwareSerial RS485Serial(1);

void setup() {
  pinMode(DE_RE_PIN, OUTPUT);
  
  // Start serial communication with the computer
  Serial.begin(BAUD_RATE);
  
  // Start RS485 communication
  RS485Serial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  digitalWrite(DE_RE_PIN, LOW); // Enable receiving mode
  Serial.println("RS485 Modbus RTU Slave");
}

void loop() {
  // Example of sending a request to read registers
  // Make sure to replace this with the actual request frames needed for your sensor
  uint8_t request[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x02}; // Example Modbus request
  sendRequest(request, sizeof(request));
  
  delay(1000); // Wait for a response
  
  // Read the response
  if (RS485Serial.available()) {
    digitalWrite(DE_RE_PIN, LOW); // Ensure it's in receiving mode
    while (RS485Serial.available()) {
      Serial.print(RS485Serial.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  delay(2000); // Wait before sending the next request
}

void sendRequest(uint8_t *request, uint16_t requestSize) {
  digitalWrite(DE_RE_PIN, HIGH); // Enable sending mode
  delay(10); // Short delay to ensure the line is ready
  RS485Serial.write(request, requestSize);
  RS485Serial.flush(); // Wait for transmission to complete
  digitalWrite(DE_RE_PIN, LOW); // Switch back to receiving mode
  delay(10); // Short delay to return to receiving mode
}
