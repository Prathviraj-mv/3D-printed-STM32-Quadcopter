#include <Wire.h>
#include <MPU6050.h>

#define NUM_CHANNELS 4  // Channels for Throttle, Roll, Pitch, Yaw
#define NUM_MOTORS 4    // Motors M1, M2, M3, M4

MPU6050 mpu;

// Channel Pins (Receiver PWM Inputs)
const int pwmPins[NUM_CHANNELS] = {PB8, PB13, PB14, PA7};

// ESC Output Pins
const int motorPins[NUM_MOTORS] = {PA0, PA1, PB0, PA6};

// Variables for timing
volatile unsigned long riseTime[NUM_CHANNELS];
volatile unsigned long pwmValues[NUM_CHANNELS];
volatile bool newSignal[NUM_CHANNELS] = {false};

unsigned long previousTime = 0;
float dt;

float accAngleX, accAngleY;
float gyroAngleX = 0, gyroAngleY = 0;
float compAngleX = 0, compAngleY = 0;
float alpha = 0.98;

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

void calibrateMPU() {
    long sumX = 0, sumY = 0, sumZ = 0;
    int samples = 2000;
    Serial.println("Calibrating... Keep the drone completely still.");
    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        delay(2);
    }
    gyroX_offset = sumX / (float)samples;
    gyroY_offset = sumY / (float)samples;
    gyroZ_offset = sumZ / (float)samples;
    Serial.println("Calibration done.");
}

void risingEdge(int channel) {
    riseTime[channel] = micros();
    attachInterrupt(digitalPinToInterrupt(pwmPins[channel]), [channel]() { fallingEdge(channel); }, FALLING);
}

void fallingEdge(int channel) {
    pwmValues[channel] = micros() - riseTime[channel];
    newSignal[channel] = true;
    attachInterrupt(digitalPinToInterrupt(pwmPins[channel]), [channel]() { risingEdge(channel); }, RISING);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    calibrateMPU();
    previousTime = millis();
    
    for (int i = 0; i < NUM_CHANNELS; i++) {
        pinMode(pwmPins[i], INPUT);
        attachInterrupt(digitalPinToInterrupt(pwmPins[i]), [i]() { risingEdge(i); }, RISING);
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(motorPins[i], OUTPUT);
    }
}

void loop() {
    unsigned long currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accX = (float)ax / 16384.0;
    float accY = (float)ay / 16384.0;
    float accZ = (float)az / 16384.0;

    float gyroXrate = ((float)gx - gyroX_offset) / 131.0;
    float gyroYrate = ((float)gy - gyroY_offset) / 131.0;
    float gyroZrate = ((float)gz - gyroZ_offset) / 131.0;

    accAngleX = atan2(accY, accZ) * 180 / PI;
    accAngleY = atan2(-accX, accZ) * 180 / PI;

    gyroAngleX += gyroXrate * dt;
    gyroAngleY += gyroYrate * dt;

    compAngleX = alpha * (compAngleX + gyroXrate * dt) + (1 - alpha) * accAngleX;
    compAngleY = alpha * (compAngleY + gyroYrate * dt) + (1 - alpha) * accAngleY;

    int throttle = pwmValues[2];
    int roll = pwmValues[0];
    int pitch = pwmValues[1];
    int yaw = pwmValues[3];

    float rollAngle = ((float)(roll - 1500) / 500.0) * 30.0;
    float pitchAngle = ((float)(pitch - 1500) / 500.0) * 30.0;
    float yawRate = ((float)(yaw - 1500) / 500.0) * 30.0;

    Serial.print("CompAngleX: "); Serial.print(compAngleX);
    Serial.print(" | CompAngleY: "); Serial.print(compAngleY);
    Serial.print(" | GyroRateZ (Yaw rate): "); Serial.print(gyroZrate);
    Serial.print(" | Throttle: "); Serial.print(throttle);
    Serial.print(" | Roll Angle: "); Serial.print(rollAngle, 2);
    Serial.print(" | Pitch Angle: "); Serial.print(pitchAngle, 2);
    Serial.print(" | Yaw Rate: "); Serial.println(yawRate, 2);
    
    delay(10);
}
