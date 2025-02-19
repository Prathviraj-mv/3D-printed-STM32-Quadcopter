#include "Arduino.h"

void setup() {
    Serial1.begin(115200);  // Use UART1 (PA9 TX, PA10 RX)
}

void loop() {
    for (int i = 1; i <= 100; i++) {
        Serial1.print("Count: ");
        Serial1.println(i);
        delay(500);
    }
}



//stm32
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

void setup() {
    Serial.begin(115200);  // USB Serial Monitor
    mySerial.begin(115200); // Communication with STM32
}

void loop() {
    if (mySerial.available()) {
        String received = mySerial.readStringUntil('\n');
        Serial.println("Received: " + received);
    }
}
