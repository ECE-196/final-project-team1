#include <Wire.h>
#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h" 
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "IMUCalibration.h"

BNO08x myIMU;

#define BNO08X_INT  -1
#define BNO08X_RST  -1
#define SDA_PIN 6
#define SCL_PIN 7

#define BNO08X_ADDR 0x4A  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

// raw accel
int16_t x;
int16_t y;
int16_t z;

// raw gyros
int16_t gx;
int16_t gy;
int16_t gz;

// offset accel
int16_t offset_ax;
int16_t offset_ay;
int16_t offset_az;

// offset gyros
int16_t offset_gx;
int16_t offset_gy;
int16_t offset_gz;


unsigned long previousDebugMillis = 0;
#define DEBUG_INTERVAL_MILLISECONDS 500

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 2000; // 2 seconds in milliseconds

// Add these variables at the top with other globals
const int WARMUP_SAMPLES = 200;  // Samples to skip
const int CALIBRATION_SAMPLES = 200;  // Samples to collect
int sampleCounter = 0;  // Total sample counter
bool isCalibrating = true;
long sumAX = 0, sumAY = 0, sumAZ = 0;
long sumGX = 0, sumGY = 0, sumGZ = 0;
int accSample = 0;
int gyroSample = 0;

// Add these variables at the top
float ax = 0, ay = 0, az = 0;             // Scaled acceleration values
float gravity_x = 0, gravity_y = 0, gravity_z = 0;  // Gravity components
float acc_x = 0, acc_y = 0, acc_z = 0;             // Linear acceleration (without gravity)
float vel_x = 0, vel_y = 0, vel_z = 0;             // Velocity
float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;             // Position
unsigned long lastUpdateTime = 0;                   // For integration timing

// Add these constants at the top
const float ACCEL_SCALE = 9.81f / 16384.0f;  // Verify this value for your IMU
const float ALPHA = 0.2f;                     // Increased for faster gravity estimation
const float NOISE_THRESHOLD = 0.05f;          // Reduced noise threshold
const float VEL_THRESHOLD = 0.01f;            // Reduced velocity threshold

// Add magnetometer variables
float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
float initial_heading = 0.0f;
const float DRIFT_CORRECTION_THRESHOLD = 0.5f; // Adjust as needed

// Add these global variables at the top
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
const float GYRO_SCALE = 1.0f / 16.0f;  // Adjust for your IMU
const float COMP_FILTER_ALPHA = 0.96f;

// void setup() {
//   Serial.begin(115200);
  
//   while(!Serial) delay(10);
  
//   Serial.println();
//   Serial.println("BNO08x Read Example");

//   Wire.begin(SDA_PIN, SCL_PIN);

//   if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
//     if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
//         Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
//     }
//   }


//   Serial.println("Reading events");
//   delay(100);
// }

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");

  if (myIMU.enableAccelerometer(1) == true) {
    Serial.println(F("Accelerometer enabled"));
  } else {
    Serial.println("Could not enable accelerometer");
  }

  if (myIMU.enableRawAccelerometer(1) == true) {
    Serial.println(F("Raw Accelerometer enabled"));
  } else {
    Serial.println("Could not enable raw accelerometer");
  }

  if (myIMU.enableGyro(1) == true) {
    Serial.println(F("Gyro enabled"));
  } else {
    Serial.println("Could not enable gyro");
  }

  if (myIMU.enableRawGyro(1) == true) {
    Serial.println(F("Raw Gyro enabled"));
  } else {
    Serial.println("Could not enable raw gyro");
  }

  if (myIMU.enableMagnetometer(1) == true) {
    Serial.println(F("Magnetometer enabled"));
  } else {
    Serial.println("Could not enable Magnetometer");
  }

  if (myIMU.enableRawMagnetometer(1) == true) {
    Serial.println(F("Raw Magnetometer enabled"));
  } else {
    Serial.println("Could not enable Raw Magnetometer");
  }

  Serial.println(F("Raw MEMS readings enabled"));
  Serial.println(F("Output is: (accel) x y z (gyro) x y z (mag) x y z"));
}


static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static boolean doConnect = false;
static boolean connected = false;
static BLEAdvertisedDevice* myDevice;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEClient* pClient;

// Callback class for connection events
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("Connected to BLE Server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Disconnected from BLE Server");
  }
};

// Callback for when devices are found during scanning
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Check if the device has our service UUID
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};

// Function to connect to the server
bool connectToServer() {
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server
  if (!pClient->connect(myDevice)) {
    return false;
  }

  // Obtain a reference to the service we are after in the remote BLE server
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  // Obtain a reference to the characteristic in the service of the remote BLE server
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  connected = true;
  return true;
}

bool calibrating;

void setup() {
  Serial.begin(115200);

  // BNO08x setup
  while(!Serial) delay(10);
  
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
    if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
        Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    }
  }


  Serial.println("Reading events");
  delay(100);

  // BLE setup
  Serial.println("Starting BLE Client...");

  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  // Calibrate IMU and set offsets.
  CalibrationOffsets offsets = IMUCalibration::calibrateIMU(myIMU);
  offset_ax = offsets.accel_x;
  offset_ay = offsets.accel_y;
  offset_az = offsets.accel_z;
  offset_gx = offsets.gyro_x;
  offset_gy = offsets.gyro_y;
  offset_gz = offsets.gyro_z;

  calibrating = false;
}

void loop() {
  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (isCalibrating) {
    if (sampleCounter == 0) {
      Serial.println("Starting warmup period...");
    }
    
    if (myIMU.getSensorEvent()) {
      uint8_t reportID = myIMU.getSensorEventID();
      sampleCounter++;

      if (sampleCounter <= WARMUP_SAMPLES) {
        // Skip these samples
        if (sampleCounter == WARMUP_SAMPLES) {
          Serial.println("Warmup complete. Starting calibration...");
        }
        return;
      }

      // Only collect samples after warmup period
      if (sampleCounter <= (WARMUP_SAMPLES + CALIBRATION_SAMPLES)) {
        switch (reportID) {
          case SENSOR_REPORTID_RAW_ACCELEROMETER:
            sumAX += myIMU.getRawAccelX();
            sumAY += myIMU.getRawAccelY();
            sumAZ += myIMU.getRawAccelZ();
            //Serial.printf("Accel offsets: X=%d, Y=%d, Z=%d\n", sumAX, sumAY, sumAZ);
            accSample++;
            break;
          case SENSOR_REPORTID_RAW_GYROSCOPE:
            sumGX += myIMU.getRawGyroX();
            sumGY += myIMU.getRawGyroY();
            sumGZ += myIMU.getRawGyroZ();
            gyroSample++;
            break;
        }

        if (sampleCounter == (WARMUP_SAMPLES + CALIBRATION_SAMPLES)) {
          // Calculate offsets
          offset_ax = sumAX / accSample;
          offset_ay = sumAY / accSample;
          offset_az = sumAZ / accSample;
          offset_gx = sumGX / gyroSample;
          offset_gy = sumGY / gyroSample;
          offset_gz = sumGZ / gyroSample;

          Serial.println("Calibration complete!");
          Serial.printf("Accel offsets: X=%d, Y=%d, Z=%d\n", offset_ax, offset_ay, offset_az);
          Serial.printf("Gyro offsets: X=%d, Y=%d, Z=%d\n", offset_gx, offset_gy, offset_gz);
          
          isCalibrating = false;
        }
      }
    }
  } else {
    // Normal operation - apply calibration offsets
    if (myIMU.getSensorEvent()) {
      uint8_t reportID = myIMU.getSensorEventID();
      unsigned long currentTime = millis();
      float dt = (currentTime - lastUpdateTime) / 1000.0f; // Convert to seconds
      
      switch (reportID) {
        case SENSOR_REPORTID_RAW_ACCELEROMETER:
          // Get calibrated readings - restore the offset correction
          x = myIMU.getRawAccelX() - offset_ax;
          y = myIMU.getRawAccelY() - offset_ay;
          z = myIMU.getRawAccelZ() - offset_az;

          // Convert to m/s^2
          ax = x * ACCEL_SCALE;
          ay = y * ACCEL_SCALE;
          az = z * ACCEL_SCALE;

          // Add debug print for raw acceleration
          if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
              Serial.println("Raw Acceleration Values:");
              Serial.printf("Raw X: %d, Y: %d, Z: %d\n", x, y, z);
              Serial.printf("Scaled X: %.3f, Y: %.3f, Z: %.3f\n", ax, ay, az);
          }

          // Low-pass filter to estimate gravity
          gravity_x = gravity_x * (1.0f - ALPHA) + ax * ALPHA;
          gravity_y = gravity_y * (1.0f - ALPHA) + ay * ALPHA;
          gravity_z = gravity_z * (1.0f - ALPHA) + az * ALPHA;

          // Remove gravity to get linear acceleration
          acc_x = ax - gravity_x;
          acc_y = ay - gravity_y;
          acc_z = az - gravity_z;

          // Apply threshold to reduce noise
          acc_x = (abs(acc_x) < NOISE_THRESHOLD) ? 0 : acc_x;
          acc_y = (abs(acc_y) < NOISE_THRESHOLD) ? 0 : acc_y;
          acc_z = (abs(acc_z) < NOISE_THRESHOLD) ? 0 : acc_z;

          // First integration: acceleration to velocity
          vel_x += acc_x * dt;
          vel_y += acc_y * dt;
          vel_z += acc_z * dt;

          // Simple velocity drift correction
          vel_x = (abs(vel_x) < VEL_THRESHOLD) ? 0 : vel_x;
          vel_y = (abs(vel_y) < VEL_THRESHOLD) ? 0 : vel_y;
          vel_z = (abs(vel_z) < VEL_THRESHOLD) ? 0 : vel_z;

          // Second integration: velocity to position
          pos_x += vel_x * dt;
          pos_y += vel_y * dt;
          pos_z += vel_z * dt;

          lastUpdateTime = currentTime;

          // Print position every 2 seconds
          // if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
          //   Serial.println("Position (meters):");
          //   Serial.printf("X: %.3f, Y: %.3f, Z: %.3f\n", pos_x, pos_y, pos_z);
          //   Serial.println("Linear Acceleration (m/s^2):");
          //   Serial.printf("X: %.3f, Y: %.3f, Z: %.3f\n", acc_x, acc_y, acc_z);
          //   Serial.println("Gravity Vector:");
          //   Serial.printf("X: %.3f, Y: %.3f, Z: %.3f\n", gravity_x, gravity_y, gravity_z);
          //   Serial.println("--------------------");
            
          //   lastPrintTime = currentTime;
          // }
          break;
        case SENSOR_REPORTID_RAW_GYROSCOPE: {
          gx = (myIMU.getRawGyroX() - offset_gx) * GYRO_SCALE;
          gy = (myIMU.getRawGyroY() - offset_gy) * GYRO_SCALE;
          gz = (myIMU.getRawGyroZ() - offset_gz) * GYRO_SCALE;
          
          // Update orientation
          float dt = (currentTime - lastUpdateTime) / 1000.0f;
          roll += gx * dt;
          pitch += gy * dt;
          yaw += gz * dt;
          break;
        }
        case SENSOR_REPORTID_RAW_MAGNETOMETER: {
          mag_x = myIMU.getRawMagX();
          mag_y = myIMU.getRawMagY();
          mag_z = myIMU.getRawMagZ();
          
          // Calculate heading from magnetometer
          float heading = atan2(mag_y, mag_x);
          
          // Drift correction based on heading change
          if (abs(heading - initial_heading) > DRIFT_CORRECTION_THRESHOLD) {
            // Reset position if significant heading change detected
            pos_x = 0.0f;
            pos_y = 0.0f;
            pos_z = 0.0f;
            vel_x = 0.0f;
            vel_y = 0.0f;
            vel_z = 0.0f;
            initial_heading = heading;
          }
          break;
        }
        // default:
        //   break;
      }
    }
  }

  //BLE data sending
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected to the BLE Server.");
    } else {
      Serial.println("Failed to connect to the server; restarting scan.");
      BLEDevice::getScan()->start(0);
    }
    doConnect = false;
  }

  if (connected) {
    char message[150];
    snprintf(message, sizeof(message), "POS:%.3f,%.3f,%.3f|ACC:%.3f,%.3f,%.3f\n", 
            pos_x, pos_y, pos_z,
            acc_x, acc_y, acc_z);
    pRemoteCharacteristic->writeValue(message, strlen(message));
    
    Serial.print("Sent data: ");
    Serial.print(message);
    
    delay(100);
  }

  // Print IMU data every 2 seconds
  unsigned long currentTime = millis();
  // if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
  //   Serial.println("IMU Readings:");
  //   Serial.printf("Accelerometer (x,y,z): %d, %d, %d\n", x, y, z);
  //   Serial.printf("Gyroscope (x,y,z): %d, %d, %d\n", gx, gy, gz);
  //   Serial.printf("calibrated (x,y,z): %d, %d, %d\n", offset_ax, offset_ay, offset_az);
  //   Serial.println("--------------------");
    
  //   lastPrintTime = currentTime;
  // }
  // if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
  //           Serial.println("Position (meters):");
  //           Serial.printf("X: %.3f, Y: %.3f, Z: %.3f\n", pos_x, pos_y, pos_z);
  //           Serial.println("Linear Acceleration (m/s^2):");
  //           Serial.printf("X: %.3f, Y: %.3f, Z: %.3f\n", acc_x, acc_y, acc_z);
  //           Serial.println("Gravity Vector:");
  //           Serial.printf("X: %.3f, Y: %.3f, Z: %.3f\n", gravity_x, gravity_y, gravity_z);
  //           Serial.println("--------------------");
            
  //           lastPrintTime = currentTime;
  //         }

  // Add this after velocity calculations
  // Zero velocity when acceleration is near zero for a period
  static int zero_acceleration_count = 0;
  if (abs(acc_x) < NOISE_THRESHOLD && 
      abs(acc_y) < NOISE_THRESHOLD && 
      abs(acc_z) < NOISE_THRESHOLD) {
      zero_acceleration_count++;
      if (zero_acceleration_count > 10) {  // After ~10 samples of near-zero acceleration
          vel_x = 0;
          vel_y = 0;
          vel_z = 0;
      }
  } else {
      zero_acceleration_count = 0;
  }
}