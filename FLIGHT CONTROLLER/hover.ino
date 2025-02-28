#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Servo.h>

// Sensor and ESC Setup
MPU6050 mpu;
Adafruit_BMP085_Unified bmp(10085);
Servo esc[4];

// STM32 Pin Definitions
#define LED_PIN         PC13
#define BUTTON_PIN      PB0
const uint8_t ESC_PINS[4] = {PA0, PA1, PA2, PA3};

// Flight Control Variables
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float altitude = 0.0, groundAltitude = 0.0;
bool altitudeCalibrated = false, droneActive = false, signalLost = false;

// PID Variables
float rollPID = 0.0, pitchPID = 0.0, yawPID = 0.0;
float rollSetpoint = 0.0, pitchSetpoint = 0.0, yawSetpoint = 0.0;
float rollError = 0.0, pitchError = 0.0, yawError = 0.0;
float rollLastError = 0.0, pitchLastError = 0.0, yawLastError = 0.0;
const float Kp = 1.2, Ki = 0.01, Kd = 0.5;

// Kalman Filter Variables
float kalmanZ = 0.0, kalmanGain = 0.1;

// Safety Limits
const float MAX_ROLL = 45.0, MAX_PITCH = 45.0;

// Button Debounce Variables
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Function Declarations
void initSensors();
void calibrateAltitude();
float getAltitude();
void readMPU();
void applyKalmanFilter();
void computePID();
void executeCommand(char cmd);
void emergencyLanding();
void checkFailsafe();
void controlMotors(float throttle, float rollAdjust, float pitchAdjust, float yawAdjust);
bool readButton();

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    for (uint8_t i = 0; i < 4; i++) {
        esc[i].attach(ESC_PINS[i]);
    }
    initSensors();
}

void loop() {
    if (readButton()) {
        droneActive = !droneActive;
        if (!droneActive) {
            controlMotors(0, 0, 0, 0);
        }
        delay(500);
    }

    if (!droneActive) return;

    if (!altitudeCalibrated) calibrateAltitude();

    readMPU();
    altitude = getAltitude();
    applyKalmanFilter();
    computePID();
    checkFailsafe();

    Serial.print("ALTITUDE: "); Serial.print(altitude);
    Serial.print(" | ROLL: "); Serial.print(roll);
    Serial.print(" | PITCH: "); Serial.print(pitch);
    Serial.print(" | YAW: "); Serial.println(yaw);

    if (Serial.available()) executeCommand(Serial.read());
    delay(20);
}

void initSensors() {
    if (!mpu.testConnection()) while (1);
    if (!bmp.begin()) while (1);
}

void calibrateAltitude() {
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
        groundAltitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure);
        altitudeCalibrated = true;
    }
}

float getAltitude() {
    sensors_event_t event;
    bmp.getEvent(&event);
    return event.pressure ? bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure) - groundAltitude : altitude;
}

void readMPU() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    roll  = ax / 16384.0;
    pitch = ay / 16384.0;
    yaw   = gz / 131.0;
}

void applyKalmanFilter() {
    kalmanZ = kalmanGain * altitude + (1 - kalmanGain) * kalmanZ;
}

void computePID() {
    rollError  = rollSetpoint - roll;
    pitchError = pitchSetpoint - pitch;
    yawError   = yawSetpoint - yaw;

    rollPID  = Kp * rollError + Ki * (rollError + rollLastError) + Kd * (rollError - rollLastError);
    pitchPID = Kp * pitchError + Ki * (pitchError + pitchLastError) + Kd * (pitchError - pitchLastError);
    yawPID   = Kp * yawError + Ki * (yawError + yawLastError) + Kd * (yawError - yawLastError);

    rollLastError  = rollError;
    pitchLastError = pitchError;
    yawLastError   = yawError;
}

void executeCommand(char cmd) {
    switch (cmd) {
        case 'W': rollSetpoint = 10.0; break;
        case 'S': rollSetpoint = -10.0; break;
        case 'A': pitchSetpoint = 10.0; break;
        case 'D': pitchSetpoint = -10.0; break;
        case 'U': controlMotors(70, 0, 0, 0); break;
        case 'L': controlMotors(30, 0, 0, 0); break;
        case 'H': Serial.println("Holding at 15ft..."); delay(5000); emergencyLanding(); break;
        case 'C': Serial.println("Emergency Landing!"); emergencyLanding(); break;
    }
}

void checkFailsafe() {
    if (altitude <= 0 && (fabs(roll) > MAX_ROLL || fabs(pitch) > MAX_PITCH)) {
        Serial.println("CRASH DETECTED! MOTORS SHUTDOWN!");
        controlMotors(0, 0, 0, 0);
        while (1);
    }
    if (signalLost) {
        Serial.println("SIGNAL LOST! AUTO LANDING...");
        emergencyLanding();
    }
}

void emergencyLanding() {
    controlMotors(30, 0, 0, 0);
    delay(5000);
    controlMotors(0, 0, 0, 0);
}

void controlMotors(float throttle, float rollAdjust, float pitchAdjust, float yawAdjust) {
    esc[0].write(throttle + rollAdjust);
    esc[1].write(throttle - rollAdjust);
    esc[2].write(throttle + pitchAdjust);
    esc[3].write(throttle - pitchAdjust);
}

bool readButton() {
    bool reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) lastDebounceTime = millis();
    lastButtonState = reading;
    return (millis() - lastDebounceTime) > debounceDelay && reading == HIGH;
}
