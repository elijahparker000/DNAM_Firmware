// Assuming the microcontroller's reference voltage is 3.3V for ADC conversion
const float referenceVoltage = 3.3; // Adjust this if your board uses a different reference voltage
const int analogPin = 32; // The pin where the pH sensor is connected

void setup() {
  Serial.begin(9600); // Start the serial communication with a baud rate of 9600
  analogReadResolution(12); // Set the ADC resolution to 12 bits (4096 levels)
}

void loop() {
  int sensorValue = analogRead(analogPin); // Read the value from the analog pin
  float voltage = sensorValue * (referenceVoltage / 4096.0); // Convert the value to voltage
  float pH = (-5.6548 * voltage) + 15.509; // Calculate pH from the voltage
  
  // Print the voltage and pH value to the Serial Monitor
  Serial.print("Voltage: ");
  Serial.print(voltage, 4); // Print the voltage with 4 decimal places
  Serial.print(" V, pH: ");
  Serial.println(pH, 2); // Print the pH value with 2 decimal places

  delay(1000); // Wait for a second before repeating the loop
}
