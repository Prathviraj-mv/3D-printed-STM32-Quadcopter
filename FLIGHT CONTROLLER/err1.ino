#include <Wire.h>
#include <MPU6050.h>

#define NUM_CHANNELS 4  // Channels for Throttle, Roll, Pitch, Yaw
#define NUM_MOTORS 4    // Motors M1, M2, M3, M4

MPU6050 mpu;

// Roll PID variables
float rollAngle;
float roll_error, roll_prev_error = 0, roll_integral = 0, roll_derivative, roll_output;
float Kp_roll = 3.0, Ki_roll = 0.02, Kd_roll = 8.0;

// Pitch PID variables
float  pitchAngle;
float pitch_error, pitch_prev_error = 0, pitch_integral = 0, pitch_derivative, pitch_output;
float Kp_pitch = 3.0, Ki_pitch = 0.02, Kd_pitch = 8.0;

unsigned long prevTime = 0;

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
//pid code
void computePID() {
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  // ----- ROLL PID -----
  roll_error = rollAngle - compAngleX;
  roll_integral += roll_error * dt;
  roll_derivative = (roll_error - roll_prev_error) / dt;
  roll_output = Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative;
  roll_prev_error = roll_error;

  // ----- PITCH PID -----
  pitch_error = pitchAngle - compAngleY;
  pitch_integral += pitch_error * dt;
  pitch_derivative = (pitch_error - pitch_prev_error) / dt;
  pitch_output = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative;
  pitch_prev_error = pitch_error;
}

//pid to motor

void motorMixing(int throttle) {
  // 4-motor mixing (X configuration)
  int motor1 = throttle + pitch_output + roll_output;   // Front Right
  int motor2 = throttle + pitch_output - roll_output;   // Front Left
  int motor3 = throttle - pitch_output - roll_output;   // Rear Left
  int motor4 = throttle - pitch_output + roll_output;   // Rear Right

  motor1 = constrain(motor1, 1100, 1900);
  motor2 = constrain(motor2, 1100, 1900);
  motor3 = constrain(motor3, 1100, 1900);
  motor4 = constrain(motor4, 1100, 1900);

  // Write PWM signals to ESC pins
  writeMotor(motorPins[0], motor1);
  writeMotor(motorPins[0], motor2);
  writeMotor(motorPins[0], motor3);
  writeMotor(motorPins[0], motor4);
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

    computePID();//callout
    motorMixing(throttle);//callout


    Serial.print("CompAngleX: "); Serial.print(compAngleX);
    Serial.print(" | CompAngleY: "); Serial.print(compAngleY);
    Serial.print(" | GyroRateZ (Yaw rate): "); Serial.print(gyroZrate);
    Serial.print(" | Throttle: "); Serial.print(throttle);
    Serial.print(" | Roll Angle: "); Serial.print(rollAngle, 2);
    Serial.print(" | Pitch Angle: "); Serial.print(pitchAngle, 2);
    Serial.print(" | Yaw Rate: "); Serial.println(yawRate, 2);
    
    delay(10);
}
