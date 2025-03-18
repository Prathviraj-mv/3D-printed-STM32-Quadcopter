#include <Wire.h>                   // Library for I2C communication
#include <Adafruit_Sensor.h>         // Unified sensor library
#include <Adafruit_BMP085_U.h>       // Library for BMP180 pressure sensor

// Create the BMP180 sensor object
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Define the I2C address for MPU6050
#define MPU6050_ADDR 0x68

// Define sea-level pressure in hPa for accurate altitude calculation
#define SEALEVELPRESSURE_HPA 1013.25

// Variables for storing gyroscope readings
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;

// Variables for accelerometer readings
float AccX, AccY, AccZ;

// Altitude calibration variable
float baselineAltitudeCM = 0.0;

// Kalman filter variables
float kalmanAngleX = 0, kalmanAngleY = 0, kalmanAngleZ = 0, kalmanAltitude = 0;
float biasX = 0, biasY = 0, biasZ = 0, biasAlt = 0;
float P[2][2] = {{1, 0}, {0, 1}}; // Error covariance matrix

// Function to initialize MPU6050
void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);   // Power management register
  Wire.write(0x00);   // Wake up the sensor
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);   // Configuration register
  Wire.write(0x05);   // Set low-pass filter
  Wire.endTransmission();
}

// Function to read gyro and accelerometer data from MPU6050
void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);   // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  // Read raw acceleration values
  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();

  // Read raw gyro values
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw values to meaningful physical values
  AccX = rawAccX / 16384.0; // Scale factor for ±2g
  AccY = rawAccY / 16384.0;
  AccZ = rawAccZ / 16384.0;

  RateRoll = rawGyroX / 65.5; // Scale factor for ±500°/s
  RatePitch = rawGyroY / 65.5;
  RateYaw = rawGyroZ / 65.5;
}

// Function to apply Kalman filter
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias) {
  float rate = newRate - bias;
  angle += dt * rate;

  // Update estimation error covariance
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + 1);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += 0.003;

  // Calculate Kalman gain
  float S = P[0][0] + 0.03;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Apply correction
  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  // Update error covariance
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

// Function to get altitude from BMP180
float getAltitudeCM() {
  sensors_event_t event;
  bmp.getEvent(&event);

  if (event.pressure) {
    float temperature;
    bmp.getTemperature(&temperature);
    float altitudeM = bmp.pressureToAltitude(SEALEVELPRESSURE_HPA, event.pressure, temperature);
    return altitudeM * 100.0; // Convert meters to centimeters
  }
  return 0.0; // Return 0 if sensor data is invalid
}

void setup() {
  Serial.begin(57600);   // Start serial communication
  Wire.begin();          // Start I2C
  initMPU6050();         // Initialize MPU6050

  if (!bmp.begin()) {
    Serial.println("BMP180 sensor not found!");
    while (1);
  }

  delay(1000); // Allow sensor stabilization
  baselineAltitudeCM = getAltitudeCM(); // Set zero altitude at startup
  Serial.println("Drone initialized at 0m altitude.");
}

void loop() {
  readMPU6050(); // Get IMU data

  // Get altitude and apply Kalman filter
  float currentAltitudeCM = getAltitudeCM();
  kalmanAltitude = kalmanFilter(currentAltitudeCM, 0, 0.05, kalmanAltitude, biasAlt);

  // Apply Kalman filter to roll, pitch, yaw
  kalmanAngleX = kalmanFilter(AccX, RateRoll, 0.05, kalmanAngleX, biasX);
  kalmanAngleY = kalmanFilter(AccY, RatePitch, 0.05, kalmanAngleY, biasY);
  kalmanAngleZ = kalmanFilter(AccZ, RateYaw, 0.05, kalmanAngleZ, biasZ);

  // Compute relative altitude from startup position
  float relativeAltitudeCM = kalmanAltitude - baselineAltitudeCM;

  // Print filtered data
  Serial.print("Roll: "); Serial.print(kalmanAngleX);
  Serial.print(" Pitch: "); Serial.print(kalmanAngleY);
  Serial.print(" Yaw: "); Serial.print(kalmanAngleZ);
  Serial.print(" Altitude: "); Serial.print(relativeAltitudeCM);
  Serial.println(" cm");

  delay(50); // Short delay for loop stability
}
