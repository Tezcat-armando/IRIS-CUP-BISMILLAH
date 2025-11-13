// ESP32 USB-To-UART Bridge
// Reference: https://techoverflow.net/2021/11/19/how-to-use-esp32-as-usb-to-uart-converter-in-platformio/

#include <Arduino.h>

// #include "pins.h"
// GPIO untuk UART ke STM32
#define STM32_RX 16  // GPIO16 -> STM32 TX
#define STM32_TX 17  // GPIO17 -> STM32 RX

void setup() {
  // Serial connects to the computer
  Serial.begin(115200);
  // Serial2 is the hardware UART port that connects to external circuitry
  Serial2.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);

  Serial.println("Esp32 USB-To-UART started.");
}

void loop() {
  // Copy byte incoming via PC serial
  while (Serial.available() > 0) {
    Serial2.write(Serial.read());
  }
 
  // Copy bytes incoming via UART serial
  while (Serial2.available() > 0) {
    Serial.write(Serial2.read());
  }
}
