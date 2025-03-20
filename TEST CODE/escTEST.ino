#include <Servo.h>

// === Pin Assignments ===
#define PIN_ESC1 PB0   // ✅ ESC 1 (Working)
#define PIN_ESC2 PA0   // ✅ ESC 2
#define PIN_ESC3 PA1   // ✅ ESC 3
#define PIN_ESC4 PA6   // ✅ ESC 4 (Now using A6)

#define PIN_LED  PC13 // ✅ Onboard LED

// === PWM Signal Definitions (in microseconds) ===
#define PWM_MIN 1000   // Minimum throttle (Stopped)
#define PWM_ARM 1050   // Arming throttle
#define PWM_LOW 1150   // Lower power throttle

// === Timing Constants ===
#define DELAY_ARM  3000  // Wait time for arming (3 sec)
#define DELAY_TEST 1000  // Time per motor test (1 sec)
#define DELAY_OFF  500   // Delay before next motor test

Servo ESC_1, ESC_2, ESC_3, ESC_4;

void setup() {
    pinMode(PIN_LED, OUTPUT);

    ESC_1.attach(PIN_ESC1);
    ESC_2.attach(PIN_ESC2);
    ESC_3.attach(PIN_ESC3);
    ESC_4.attach(PIN_ESC4);  // ✅ Now using A6 instead of A3

    Serial.begin(115200);
    Serial.println("🚀 Arming ESCs...");

    // Arm ESCs
    ESC_1.writeMicroseconds(PWM_MIN);
    ESC_2.writeMicroseconds(PWM_MIN);
    ESC_3.writeMicroseconds(PWM_MIN);
    ESC_4.writeMicroseconds(PWM_MIN);
    delay(DELAY_ARM);

    ESC_1.writeMicroseconds(PWM_ARM);
    ESC_2.writeMicroseconds(PWM_ARM);
    ESC_3.writeMicroseconds(PWM_ARM);
    ESC_4.writeMicroseconds(PWM_ARM);
    delay(DELAY_ARM);

    Serial.println("✅ ESCs Armed!");
}

void loop() {
    testESC(ESC_1);
    testESC(ESC_2);
    testESC(ESC_3);
    testESC(ESC_4);
}

void testESC(Servo &ESC) {
    digitalWrite(PIN_LED, HIGH);
    ESC.writeMicroseconds(PWM_LOW);  // 🔽 Lower power
    delay(DELAY_TEST);
    ESC.writeMicroseconds(PWM_MIN);  // Stop motor
    digitalWrite(PIN_LED, LOW);
    delay(DELAY_OFF);
}
