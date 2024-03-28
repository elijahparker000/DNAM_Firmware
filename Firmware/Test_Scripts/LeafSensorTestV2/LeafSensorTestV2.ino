#include <SoftwareSerial.h>

#define RE_DE 19 // Assuming RE and DE are connected together and to GPIO 19
SoftwareSerial modbusSerial(9, 10); // RX, TX connected to the RS485 module

void setup() {
  Serial.begin(9600);
  modbusSerial.begin(9600); // Match the baud rate of the sensor
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW); // Initially, set to receive mode
}

void loop() {
  // Construct and send a Modbus RTU request
  digitalWrite(RE_DE, HIGH); // Switch to transmit mode
  delay(10); // Short delay before sending data
  byte request[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB}; // Pre-calculated request with CRC
  modbusSerial.write(request, sizeof(request));
  modbusSerial.flush(); // Ensure complete transmission of request
  digitalWrite(RE_DE, LOW); // Switch back to receive mode

  delay(2000); // Wait for a bit to receive the response

  // Attempt to read the response
  Serial.println("Response:");
  while(modbusSerial.available()) {
    Serial.print(modbusSerial.read(), HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  delay(5000); // Wait before sending the next request
}
