#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <MPU6050.h>  // Using Electronic Cats' MPU6050 library

// Create sensor objects
MPU6050 mpu;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

#define LED_PIN PC13  // Onboard LED

void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(LED_PIN, OUTPUT);

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("Failed to connect to MPU6050!");
        while (1);
    }
    Serial.println("MPU6050 Connected!");

    // Initialize BMP180
    if (!bmp.begin()) {
        Serial.println("Could not find BMP180 sensor!");
        while (1);
    }
    Serial.println("BMP180 Found!");
}

void loop() {
    // Read MPU6050 data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print("Accel X: "); Serial.print(ax);
    Serial.print(" Y: "); Serial.print(ay);
    Serial.print(" Z: "); Serial.println(az);

    Serial.print("Gyro X: "); Serial.print(gx);
    Serial.print(" Y: "); Serial.print(gy);
    Serial.print(" Z: "); Serial.println(gz);

    // Read BMP180 data
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
        Serial.print("Pressure: "); Serial.print(event.pressure);
        Serial.println(" hPa");
    }

    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("BMP180 Temp: "); Serial.print(temperature);
    Serial.println(" C");

    // Blink LED (PC13 is active LOW)
    digitalWrite(LED_PIN, LOW);  // Turn ON
    delay(500);
    digitalWrite(LED_PIN, HIGH); // Turn OFF
    delay(500);
}
