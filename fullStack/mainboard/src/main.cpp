#include <Wire.h>
#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

BNO08x myIMU;

#define BNO08X_INT  35
#define BNO08X_RST  -1
#define SDA_PIN 33
#define SCL_PIN 34
#define CALIB_PIN 11  // Pin to trigger calibration
#define OUTPUT_PIN 46 // Pin to output HIGH

#define BNO08X_ADDR 0x4A  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

float accelXOffset = 0.0;
float accelYOffset = 0.0;
float accelZOffset = 0.0;

void setup() {
  USBSerial.begin(115200);
  
  while (!USBSerial) delay(10);
  
  USBSerial.println();
  USBSerial.println("BNO08x Calibration Example");

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(CALIB_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, HIGH); // Set OUTPUT_PIN to always HIGH

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    USBSerial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  if (myIMU.enableRawAccelerometer(1) == false) {
    USBSerial.println("Could not enable raw accelerometer");
    while (1);
  }

  USBSerial.println("Waiting for calibration trigger...");
}

void calibrateAccelerometer() {
  int samples = 100;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

  USBSerial.println("Calibrating...");

  for (int i = 0; i < samples; i++) {
    while (!myIMU.getSensorEvent()) delay(1); // Wait for new sensor event
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_RAW_ACCELEROMETER) {
      sumX += myIMU.getRawAccelX();
      sumY += myIMU.getRawAccelY();
      sumZ += myIMU.getRawAccelZ();
    }
    delay(10); // Small delay between readings
  }

  accelXOffset = sumX / samples;
  accelYOffset = sumY / samples;
  accelZOffset = sumZ / samples - 9.8; // Adjust for gravity on Z-axis

  USBSerial.println("Calibration complete:");
  USBSerial.print("X Offset: "); USBSerial.println(accelXOffset);
  USBSerial.print("Y Offset: "); USBSerial.println(accelYOffset);
  USBSerial.print("Z Offset: "); USBSerial.println(accelZOffset);
}

void loop() {
  // Check if the calibration pin is HIGH
  if (digitalRead(CALIB_PIN) == HIGH) {
    calibrateAccelerometer();
  }

  while (!myIMU.getSensorEvent()) delay(1); // Wait for new sensor event

  if (myIMU.getSensorEventID() == SENSOR_REPORTID_RAW_ACCELEROMETER) {
    float rawX = myIMU.getRawAccelX();
    float rawY = myIMU.getRawAccelY();
    float rawZ = myIMU.getRawAccelZ();

    // Apply calibration offsets
    float calibratedX = rawX - accelXOffset;
    float calibratedY = rawY - accelYOffset;
    float calibratedZ = rawZ - accelZOffset;

    USBSerial.print("Calibrated Accel: ");
    USBSerial.print(calibratedX); USBSerial.print("\t");
    USBSerial.print(calibratedY); USBSerial.print("\t");
    USBSerial.print(calibratedZ); USBSerial.println();

    delay(500); // Update rate for calibrated readings
  }
}
