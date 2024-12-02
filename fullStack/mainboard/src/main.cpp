#include <Wire.h>
#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h" 
#include "PlayAudio.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

BNO08x myIMU;
#define BNO08X_INT  35
#define BNO08X_RST  -1
#define SDA_PIN 33
#define SCL_PIN 34
#define BNO08X_ADDR 0x4A  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
#define AUDIO_OUTPUT_PIN 21

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
PlayAudio audioplayer(AUDIO_OUTPUT_PIN,60000);


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
  USBSerial.println("Reading events");
  delay(100);

  audioplayer.begin();
}

// // Here is where you define the sensor outputs you want to receive
// void setReports(void) {
//   USBSerial.println("Setting desired reports");
//   if (myIMU.enableAccelerometer(1) == true) {
//     USBSerial.println(F("Accelerometer enabled"));
//   } else {
//     USBSerial.println("Could not enable accelerometer");
//   }
//   if (myIMU.enableRawAccelerometer(1) == true) {
//     USBSerial.println(F("Raw Accelerometer enabled"));
//   } else {
//     USBSerial.println("Could not enable raw accelerometer");
//   }
//   if (myIMU.enableGyro(1) == true) {
//     USBSerial.println(F("Gyro enabled"));
//   } else {
//     USBSerial.println("Could not enable gyro");
//   }
//   if (myIMU.enableRawGyro(1) == true) {
//     USBSerial.println(F("Raw Gyro enabled"));
//   } else {
//     USBSerial.println("Could not enable raw gyro");
//   }
//   if (myIMU.enableMagnetometer(1) == true) {
//     USBSerial.println(F("Magnetometer enabled"));
//   } else {
//     USBSerial.println("Could not enable Magnetometer");
//   }
//   if (myIMU.enableRawMagnetometer(1) == true) {
//     USBSerial.println(F("Raw Magnetometer enabled"));
//   } else {
//     USBSerial.println("Could not enable Raw Magnetometer");
//   }
//   USBSerial.println(F("Raw MEMS readings enabled"));
//   USBSerial.println(F("Output is: (accel) x y z (gyro) x y z (mag) x y z"));
// }
// void loop() {
//   delayMicroseconds(10);
//   if (myIMU.wasReset()) {
//     USBSerial.print("sensor was reset ");
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

//     int timeSinceLastUSBSerialPrint = (millis() - previousDebugMillis);
//     // Only print data to the terminal at a user deficed interval
//     if(timeSinceLastUSBSerialPrint > DEBUG_INTERVAL_MILLISECONDS)
//     {
//         USBSerial.print("Accel: ");
//         USBSerial.print(x);
//         USBSerial.print("\t");
//         USBSerial.print(y);
//         USBSerial.print("\t");
//         USBSerial.print(z);
//         USBSerial.println();
//         USBSerial.print("Gyro: ");
//         USBSerial.print(gx);
//         USBSerial.print("\t");
//         USBSerial.print(gy);
//         USBSerial.print("\t");
//         USBSerial.print(gz);
//         USBSerial.println();
//         USBSerial.println("-------------------------------------------------------");
//         previousDebugMillis = millis();
//     }
//   }
//   audioplayer.playWaveform();
// }

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

// void setup() {
//     USBSerial.begin(115200);
//     while(!USBSerial) delay(100);
//     safePrint("Starting BLE Server...\n");

//     // Create the BLE Device
//     BLEDevice::init("ESP32S3_BLE_Server");

//     // Create the BLE Server
//     BLEServer *pServer = BLEDevice::createServer();
//     pServer->setCallbacks(new MyServerCallbacks());

//     // Create the BLE Service
//     BLEService *pService = pServer->createService(SERVICE_UUID);

//     // Create a BLE Characteristic
//     pCharacteristic = pService->createCharacteristic(
//                         CHARACTERISTIC_UUID,
//                         BLECharacteristic::PROPERTY_READ |
//                         BLECharacteristic::PROPERTY_WRITE |
//                         BLECharacteristic::PROPERTY_NOTIFY
//                     );

//     pCharacteristic->setCallbacks(new MyCallbacks());

//     // Start the service
//     pService->start();

//     // Start advertising
//     BLEAdvertising *pAdvertising = pServer->getAdvertising();
//     pAdvertising->addServiceUUID(SERVICE_UUID);
//     pAdvertising->setScanResponse(true);
//     pAdvertising->setMinPreferred(0x06);
//     pAdvertising->setMinPreferred(0x12);
//     pAdvertising->start();

//     safePrint("BLE Server is ready, waiting for client connection...\n");
// }

void loop() {
    delay(10);
}