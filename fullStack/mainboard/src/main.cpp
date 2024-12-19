#include <Wire.h>
#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h" 
#include "PlayAudio.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

BNO08x myIMU;

#define BNO08X_INT  -1
#define BNO08X_RST  -1
#define SDA_PIN 34
#define SCL_PIN 33

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

// Add with other global variables
float displacement = 0.0f;
const float DISPLACEMENT_THRESHOLD = 0.5f;  // 0.5 meters
int audioPlayCount = 0;
const int MAX_AUDIO_PLAYS = 5;
bool audioTriggered = false;

// Add with other global variables
const unsigned long WARMUP_TIME = 30000;  // 30 seconds in milliseconds
unsigned long startTime = 0;
bool warmupComplete = false;

// Add with other global variables
const unsigned long RESET_INTERVAL = 300000;  // 5 minutes in milliseconds
unsigned long lastResetTime = 0;

// Add this function at the top with other functions
void printLocation() {
    USBSerial.println("\n==== Current Device Location ====");
    USBSerial.printf("X Position: %.3f meters\n", pos_x);
    USBSerial.printf("Y Position: %.3f meters\n", pos_y);
    USBSerial.printf("Z Position: %.3f meters\n", pos_z);
    USBSerial.println("===============================\n");
}

// Add this function after printLocation()
void resetDisplacement() {
    pos_x = 0.0f;
    pos_y = 0.0f;
    pos_z = 0.0f;
    vel_x = 0.0f;
    vel_y = 0.0f;
    vel_z = 0.0f;
    displacement = 0.0f;
    lastResetTime = millis();
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  USBSerial.println("Setting desired reports");

  if (myIMU.enableAccelerometer(10) == true) {
    USBSerial.println(F("Accelerometer enabled at 100Hz"));
  } else {
    USBSerial.println("Could not enable accelerometer");
  }

  if (myIMU.enableRawAccelerometer(10) == true) {
    USBSerial.println(F("Raw Accelerometer enabled at 100Hz"));
  } else {
    USBSerial.println("Could not enable raw accelerometer");
  }

  if (myIMU.enableGyro(10) == true) {
    USBSerial.println(F("Gyro enabled at 100Hz"));
  } else {
    USBSerial.println("Could not enable gyro");
  }

  if (myIMU.enableRawGyro(10) == true) {
    USBSerial.println(F("Raw Gyro enabled at 100Hz"));
  } else {
    USBSerial.println("Could not enable raw gyro");
  }

  if (myIMU.enableMagnetometer(10) == true) {
    USBSerial.println(F("Magnetometer enabled at 100Hz"));
  } else {
    USBSerial.println("Could not enable Magnetometer");
  }

  if (myIMU.enableRawMagnetometer(10) == true) {
    USBSerial.println(F("Raw Magnetometer enabled at 100Hz"));
  } else {
    USBSerial.println("Could not enable Raw Magnetometer");
  }

  USBSerial.println(F("Raw MEMS readings enabled"));
  USBSerial.println(F("Output is: (accel) x y z (gyro) x y z (mag) x y z"));
}

// BLE PART
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Message handling
String messageBuffer = "";
const char END_MARKER = '\n';
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Safe print function
void safePrint(const String& message) {
    portENTER_CRITICAL(&mux);
    USBSerial.print(message);
    USBSerial.flush(); // Ensure complete transmission
    portEXIT_CRITICAL(&mux);
}

// Add these helper functions
void clearBuffer() {
    portENTER_CRITICAL(&mux);
    messageBuffer = "";
    portEXIT_CRITICAL(&mux);
}

void appendToBuffer(const String& data) {
    portENTER_CRITICAL(&mux);
    messageBuffer += data;
    portEXIT_CRITICAL(&mux);
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        clearBuffer();  // Clear buffer atomically
        safePrint("Client Connected\n");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        clearBuffer();  // Clear buffer atomically
        safePrint("Client Disconnected\n");
        pServer->getAdvertising()->start();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        
        if (rxValue.length() > 0) {
            // Convert received data to String and append to buffer atomically
            String receivedData = String(rxValue.c_str());
            appendToBuffer(receivedData);
            
            portENTER_CRITICAL(&mux);
            // Process complete messages
            int endIndex;
            while ((endIndex = messageBuffer.indexOf(END_MARKER)) != -1) {
                // Extract complete message
                String completeMessage = messageBuffer.substring(0, endIndex);
                // Only keep unprocessed part
                messageBuffer = messageBuffer.substring(endIndex + 1);
                
                // Print complete message while still in critical section
                USBSerial.print("Received: ");
                USBSerial.println(completeMessage);
                USBSerial.flush();
            }
            portEXIT_CRITICAL(&mux);
        }
    }
};


void setup() {
  USBSerial.begin(115200);

  // BNO08x setup
  while (!USBSerial) delay(10);

  USBSerial.println();
  USBSerial.println("BNO08x Read Example");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
    if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
        USBSerial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    }
  }

  USBSerial.println("Reading events");
  delay(100);

  // Calibrate IMU and set offsets.
  offset_ax = offset_ay = offset_az = 0;
  offset_gx = offset_gy = offset_gz = 0;


  // Create the BLE Device
  BLEDevice::init("ESP32S3_BLE_Server");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                  );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();

  USBSerial.println("BLE Server is ready, waiting for client connection...\n");

  // Add with other global variables
  startTime = millis();
  lastResetTime = millis();
}

void loop() {
  if (myIMU.wasReset()) {
    USBSerial.print("sensor was reset ");
    setReports();
  }

  // Normal operation - apply calibration offsets
  if (myIMU.getSensorEvent()) {
    uint8_t reportID = myIMU.getSensorEventID();
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f; // Convert to seconds

    if (currentTime - lastResetTime >= RESET_INTERVAL) {
        resetDisplacement();
        USBSerial.println("Displacement values reset (5-minute interval)");
    }

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
          USBSerial.println("Raw Acceleration Values:");
          USBSerial.printf("Raw X: %d, Y: %d, Z: %d\n", x, y, z);
          USBSerial.printf("Scaled X: %.3f, Y: %.3f, Z: %.3f\n", ax, ay, az);
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

        displacement = sqrt(pos_x * pos_x + pos_y * pos_y + pos_z * pos_z);

        lastUpdateTime = currentTime;

        if (displacement > DISPLACEMENT_THRESHOLD && !audioTriggered && audioPlayCount < MAX_AUDIO_PLAYS) {
            if (!warmupComplete) {
                if (millis() - startTime >= WARMUP_TIME) {
                    warmupComplete = true;
                }
            } else {
                PlayAudio audio(21, 60000);
                audio.begin();
                audio.playWaveform();
                audioPlayCount++;
                
                if (audioPlayCount >= MAX_AUDIO_PLAYS) {
                    audioTriggered = true;
                }
            }
        }
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
    }
  }

  // Print IMU data every 2 seconds
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    printLocation();

    // Debug info (optional)
    USBSerial.println("Motion Data:");
    USBSerial.printf("Acceleration (m/s²) - X: %.3f, Y: %.3f, Z: %.3f\n", acc_x, acc_y, acc_z);
    USBSerial.printf("Velocity (m/s) - X: %.3f, Y: %.3f, Z: %.3f\n", vel_x, vel_y, vel_z);

    lastPrintTime = currentTime;
  }

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