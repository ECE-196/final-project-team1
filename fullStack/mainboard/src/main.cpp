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
  
  while(!USBSerial) delay(10);
  
  USBSerial.println();
  USBSerial.println("BNO08x Read Example");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
    if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
        USBSerial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    }
  }

  audioPlayer.begin();

  USBSerial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  USBSerial.println("Setting desired reports");

  if (myIMU.enableAccelerometer(1) == true) {
    USBSerial.println(F("Accelerometer enabled"));
  } else {
    USBSerial.println("Could not enable accelerometer");
  }

  if (myIMU.enableRawAccelerometer(1) == true) {
    USBSerial.println(F("Raw Accelerometer enabled"));
  } else {
    USBSerial.println("Could not enable raw accelerometer");
  }

  if (myIMU.enableGyro(1) == true) {
    USBSerial.println(F("Gyro enabled"));
  } else {
    USBSerial.println("Could not enable gyro");
  }

  if (myIMU.enableRawGyro(1) == true) {
    USBSerial.println(F("Raw Gyro enabled"));
  } else {
    USBSerial.println("Could not enable raw gyro");
  }

  if (myIMU.enableMagnetometer(1) == true) {
    USBSerial.println(F("Magnetometer enabled"));
  } else {
    USBSerial.println("Could not enable Magnetometer");
  }

  if (myIMU.enableRawMagnetometer(1) == true) {
    USBSerial.println(F("Raw Magnetometer enabled"));
  } else {
    USBSerial.println("Could not enable Raw Magnetometer");
  }

  USBSerial.println(F("Raw MEMS readings enabled"));
  USBSerial.println(F("Output is: (accel) x y z (gyro) x y z (mag) x y z"));
}

void loop() {
  delayMicroseconds(10);

  if (myIMU.wasReset()) {
    USBSerial.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true)
  {

    // keep track of if we've recieved an updated value on any one of the
    // reports we're looking for.
    uint8_t reportID = myIMU.getSensorEventID();

    switch (reportID) {
        case SENSOR_REPORTID_RAW_ACCELEROMETER:
            x = myIMU.getRawAccelX();
            y = myIMU.getRawAccelY();
            z = myIMU.getRawAccelZ();
            break;
        case SENSOR_REPORTID_RAW_GYROSCOPE:
            gx = myIMU.getRawGyroX();
            gy = myIMU.getRawGyroY();
            gz = myIMU.getRawGyroZ();
            break;
        default:
            break;
    }


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

    }
  }

//   // if theft is detected, play the audio
//   if(theftDetected){
//     audioPlayer.playWaveform();
//   }
audioPlayer.playWaveform();
}