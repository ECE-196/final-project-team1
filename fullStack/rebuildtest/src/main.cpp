#include <Arduino.h>
#include <Wire.h>

void setup() {
    USBSerial.begin(115200); // Match this baud rate with monitor_speed in platformio.ini
    while (!Serial) {     // Wait for the serial port to initialize (optional)
        delay(100);
        USBSerial.println("can not connect to serial");
    }
    USBSerial.println("Serial communication is active!");
}

void loop() {
    USBSerial.println("Hello, World!");
    delay(1000); // Print every second
}
