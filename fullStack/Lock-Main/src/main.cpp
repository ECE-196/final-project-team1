#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 33 // Define your SDA pin
#define SCL_PIN 34 // Define your SCL pin

void setup() {
  // Initialize USBSerial Monitor
  USBSerial.begin(115200);
  delay(1000);
  USBSerial.println("\nI2C Scanner");

  // Initialize I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  USBSerial.println("Scanning for I2C devices...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    // Try to connect to the device at this address
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      // Device found
      USBSerial.print("I2C device found at address 0x");
      if (address < 16)
        USBSerial.print("0");
      USBSerial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      // Device error
      USBSerial.print("Unknown error at address 0x");
      if (address < 16)
        USBSerial.print("0");
      USBSerial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    USBSerial.println("No I2C devices found\n");
  else
    USBSerial.println("I2C scan complete\n");

  delay(5000); // Wait 5 seconds before the next scan
}
