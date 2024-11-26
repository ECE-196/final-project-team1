#include <Wire.h>
#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h" 
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

BNO08x myIMU;

#define BNO08X_INT  -1
#define BNO08X_RST  -1
#define SDA_PIN 6
#define SCL_PIN 7

#define BNO08X_ADDR 0x4A  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

// // raw accel
// int16_t x;
// int16_t y;
// int16_t z;

// // raw gyros
// int16_t gx;
// int16_t gy;
// int16_t gz;


// unsigned long previousDebugMillis = 0;
// #define DEBUG_INTERVAL_MILLISECONDS 500

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

// // Here is where you define the sensor outputs you want to receive
// void setReports(void) {
//   Serial.println("Setting desired reports");

//   if (myIMU.enableAccelerometer(1) == true) {
//     Serial.println(F("Accelerometer enabled"));
//   } else {
//     Serial.println("Could not enable accelerometer");
//   }

//   if (myIMU.enableRawAccelerometer(1) == true) {
//     Serial.println(F("Raw Accelerometer enabled"));
//   } else {
//     Serial.println("Could not enable raw accelerometer");
//   }

//   if (myIMU.enableGyro(1) == true) {
//     Serial.println(F("Gyro enabled"));
//   } else {
//     Serial.println("Could not enable gyro");
//   }

//   if (myIMU.enableRawGyro(1) == true) {
//     Serial.println(F("Raw Gyro enabled"));
//   } else {
//     Serial.println("Could not enable raw gyro");
//   }

//   if (myIMU.enableMagnetometer(1) == true) {
//     Serial.println(F("Magnetometer enabled"));
//   } else {
//     Serial.println("Could not enable Magnetometer");
//   }

//   if (myIMU.enableRawMagnetometer(1) == true) {
//     Serial.println(F("Raw Magnetometer enabled"));
//   } else {
//     Serial.println("Could not enable Raw Magnetometer");
//   }

//   Serial.println(F("Raw MEMS readings enabled"));
//   Serial.println(F("Output is: (accel) x y z (gyro) x y z (mag) x y z"));
// }

// void loop() {
//   delayMicroseconds(10);

//   if (myIMU.wasReset()) {
//     Serial.print("sensor was reset ");
//     setReports();
//   }

//   // Has a new event come in on the Sensor Hub Bus?
//   if (myIMU.getSensorEvent() == true)
//   {

//     // keep track of if we've recieved an updated value on any one of the
//     // reports we're looking for.
//     uint8_t reportID = myIMU.getSensorEventID();

//     switch (reportID) {
//         case SENSOR_REPORTID_RAW_ACCELEROMETER:
//             x = myIMU.getRawAccelX();
//             y = myIMU.getRawAccelY();
//             z = myIMU.getRawAccelZ();
//             break;
//         case SENSOR_REPORTID_RAW_GYROSCOPE:
//             gx = myIMU.getRawGyroX();
//             gy = myIMU.getRawGyroY();
//             gz = myIMU.getRawGyroZ();
//             break;
//         default:
//             break;
//     }


//     int timeSinceLastSerialPrint = (millis() - previousDebugMillis);

//     // Only print data to the terminal at a user deficed interval
//     if(timeSinceLastSerialPrint > DEBUG_INTERVAL_MILLISECONDS)
//     {
//         Serial.print("Accel: ");
//         Serial.print(x);
//         Serial.print("\t");
//         Serial.print(y);
//         Serial.print("\t");
//         Serial.print(z);
//         Serial.println();

//         Serial.print("Gyro: ");
//         Serial.print(gx);
//         Serial.print("\t");
//         Serial.print(gy);
//         Serial.print("\t");
//         Serial.print(gz);
//         Serial.println();
//         Serial.println("-------------------------------------------------------");

//         previousDebugMillis = millis();

//     }
//   }
// }

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

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Client...");

  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop() {
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
    // Send "Hello World" every 2 seconds
    String message = "Hello World\n";
    pRemoteCharacteristic->writeValue(message.c_str(), message.length());
    Serial.print("Sent message (hex): ");
    // Print each byte in hexadecimal
    for(int i = 0; i < message.length(); i++) {
      if(message[i] < 16) Serial.print("0"); // Add leading zero for single digit hex
      Serial.print(message[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print("Sent message (ascii): ");
    Serial.println(message);
    delay(2000);
  }
}