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
