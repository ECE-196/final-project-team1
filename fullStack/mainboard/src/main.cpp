#include <Wire.h>
#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
#include "IMUCalibration.h"
#include "PlayAudio.h"

BNO08x myIMU;
IMUCalibration calibration;

#define BNO08X_INT  35
#define BNO08X_RST  -1
#define SDA_PIN 33
#define SCL_PIN 34
#define CALIB_PIN 11  // Pin to trigger calibration
#define OUTPUT_PIN 46 // Pin to output HIGH

#define AUDIO_OUTPUT_PIN 14

#define BNO08X_ADDR 0x4A  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
PlayAudio audioPlayer(AUDIO_OUTPUT_PIN,60000);
// raw accel
int16_t x;
int16_t y;
int16_t z;

// raw gyros
int16_t gx;
int16_t gy;
int16_t gz;


unsigned long previousDebugMillis = 0;
#define DEBUG_INTERVAL_MILLISECONDS 500

void setup() {
  USBSerial.begin(115200);
  
  while (!USBSerial) delay(10);
  
  USBSerial.println();
  USBSerial.println("BNO08x Calibration Example");

  Wire.begin(SDA_PIN, SCL_PIN);


  if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
    if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
        USBSerial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    }
  }

  audioPlayer.begin();

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



    int timeSinceLastUSBSerialPrint = (millis() - previousDebugMillis);

    // Only print data to the terminal at a user deficed interval
    if(timeSinceLastUSBSerialPrint > DEBUG_INTERVAL_MILLISECONDS)
    {
        USBSerial.print("Accel: ");
        USBSerial.print(x);
        USBSerial.print("\t");
        USBSerial.print(y);
        USBSerial.print("\t");
        USBSerial.print(z);
        USBSerial.println();

        USBSerial.print("Gyro: ");
        USBSerial.print(gx);
        USBSerial.print("\t");
        USBSerial.print(gy);
        USBSerial.print("\t");
        USBSerial.print(gz);
        USBSerial.println();
        USBSerial.println("-------------------------------------------------------");

        previousDebugMillis = millis();


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


//   // if theft is detected, play the audio
//   if(theftDetected){
//     audioPlayer.playWaveform();
//   }
audioPlayer.playWaveform();
}

