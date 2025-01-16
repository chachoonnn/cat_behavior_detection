#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to initialize
  Serial.println("\nI2C Scanner");
  Wire.begin();

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
    }
    delay(10);
  }
  Serial.println("Scanning complete.");
}


void loop() {
//  Serial.println("Testing I2C bus...");
//  Wire.beginTransmission(0x00);  // Start transmission to an invalid address
//  uint8_t error = Wire.endTransmission();
//  if (error == 0) {
//    Serial.println("I2C bus is working.");
//  } else {
//    Serial.println("No response from I2C bus. Check wiring and pull-ups.");
//  }
}
