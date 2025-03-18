#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// MPU6050 object
MPU6050 mpu;

// BMP180 object
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
#define SEALEVELPRESSURE_HPA (1013.25)

// Offsets and filter variables
float accX_offset = 0, accY_offset = 0, accZ_offset = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
float filteredPitch = 0, filteredRoll = 0, filteredYaw = 0;
float filterFactor = 0.05;
unsigned long previousTime = 0;

// BMP baseline altitude
float baselineAltitudeCM = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  initializeMPU();
  calibrateMPU();

  initializeBMP();

  previousTime = micros();
}

void loop() {
  float dt = calculateDeltaTime();

  int16_t ax, ay, az, gx, gy, gz;
  readMPUData(ax, ay, az, gx, gy, gz);

  updateAngles(ax, ay, az, gx, gy, gz, dt);

  float altitude = detectUpwardMovementAndGetAltitude(az);

  printData(altitude);

  delay(50);
}

// === Initialization Functions ===
void initializeMPU() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed.");
    while (1);
  }
  Serial.println("MPU6050 connected and ready.");
}

void initializeBMP() {
  Serial.println(F("Initializing BMP180..."));
  if (!bmp.begin()) {
    Serial.println(F("BMP180 sensor not found!"));
    while (1);
  }
  delay(1000);
  baselineAltitudeCM = getAltitudeCM();
  Serial.print("Baseline altitude set: ");
  Serial.print(baselineAltitudeCM);
  Serial.println(" cm");
}

// === Calibration ===
void calibrateMPU() {
  long accX_sum = 0, accY_sum = 0, accZ_sum = 0;
  long gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
  int count = 0;

  Serial.println("Calibrating MPU... Hold still for 2 seconds.");

  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    accX_sum += ax;
    accY_sum += ay;
    accZ_sum += az;
    gyroX_sum += gx;
    gyroY_sum += gy;
    gyroZ_sum += gz;

    count++;
    delay(5);
  }

  accX_offset = accX_sum / (float)count;
  accY_offset = accY_sum / (float)count;
  accZ_offset = accZ_sum / (float)count;
  gyroX_offset = gyroX_sum / (float)count;
  gyroY_offset = gyroY_sum / (float)count;
  gyroZ_offset = gyroZ_sum / (float)count;

  Serial.println("Calibration Complete.");
}

// === Helper Functions ===
float calculateDeltaTime() {
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;
  return dt;
}

void readMPUData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
}

void updateAngles(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt) {
  float accX = (ax - accX_offset);
  float accY = (ay - accY_offset);
  float accZ = (az - accZ_offset);

  float accAngleX = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
  float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  float gyroX = (gx - gyroX_offset) / 131.0;
  float gyroY = (gy - gyroY_offset) / 131.0;
  float gyroZ = (gz - gyroZ_offset) / 131.0;

  gyroAngleX += gyroX * dt;
  gyroAngleY += gyroY * dt;
  gyroAngleZ += gyroZ * dt;

  float roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  float pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  float yaw = gyroAngleZ;

  filteredRoll = filteredRoll * (1 - filterFactor) + roll * filterFactor;
  filteredPitch = filteredPitch * (1 - filterFactor) + pitch * filterFactor;
  filteredYaw = filteredYaw * (1 - filterFactor) + yaw * filterFactor;
}

float detectUpwardMovementAndGetAltitude(int16_t accZ) {
  const float upwardThreshold = -1500;
  if (accZ < upwardThreshold) {
    return getAltitudeCM() - baselineAltitudeCM;
  } else {
    return 0;
  }
}

float getAltitudeCM() {
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    float temperature;
    bmp.getTemperature(&temperature);
    float altitudeM = bmp.pressureToAltitude(SEALEVELPRESSURE_HPA, event.pressure, temperature);
    return altitudeM * 100.0;
  }
  return 0;
}

void printData(float altitude) {
  Serial.print("Roll: "); Serial.print(filteredRoll, 2);
  Serial.print(" | Pitch: "); Serial.print(filteredPitch, 2);
  Serial.print(" | Yaw: "); Serial.print(filteredYaw, 2);
  Serial.print(" | Altitude: "); Serial.print(altitude, 2);
  Serial.println(" cm");
}
