#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Manual offsets (you can adjust later if needed)
float manual_accX_offset = 0;
float manual_accY_offset = 0;
float manual_accZ_offset = 0;
float manual_gyroX_offset = 0;
float manual_gyroY_offset = 0;
float manual_gyroZ_offset = 0;

// Auto-calculated offsets
float accX_offset = 0, accY_offset = 0, accZ_offset = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

float gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
unsigned long previousTime = 0;

// Filtered values
float filteredPitch = 0, filteredRoll = 0, filteredYaw = 0;
float filterFactor = 0.05;  // Lower = smoother, but slower response

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected.");
  } else {
    Serial.println("MPU6050 connection failed.");
    while (1);
  }

  delay(2000);
  Serial.println("Calibrating for 2 seconds... Keep MPU6050 still.");
  calibrateForTwoSeconds();
  Serial.println("Calibration done. Starting data read...");
  previousTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0; 
  previousTime = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  float accX = (ax - accX_offset - manual_accX_offset);
  float accY = (ay - accY_offset - manual_accY_offset);
  float accZ = (az - accZ_offset - manual_accZ_offset);
  float gyroX = (gx - gyroX_offset - manual_gyroX_offset) / 131.0;  
  float gyroY = (gy - gyroY_offset - manual_gyroY_offset) / 131.0;
  float gyroZ = (gz - gyroZ_offset - manual_gyroZ_offset) / 131.0;

  // Calculate accelerometer angles
  float accAngleX = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
  float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Integrate gyro
  gyroAngleX += gyroX * dt;
  gyroAngleY += gyroY * dt;
  gyroAngleZ += gyroZ * dt;

  // Complementary filter with swapped logic
  float roll  = 0.96 * gyroAngleX + 0.04 * accAngleX;   // Was pitch, now roll
  float pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;   // Was roll, now pitch
  float yaw   = gyroAngleZ;  // only from gyro

  // Apply smoothing (low-pass filter)
  filteredPitch = filteredPitch * (1 - filterFactor) + pitch * filterFactor;
  filteredRoll  = filteredRoll  * (1 - filterFactor) + roll  * filterFactor;
  filteredYaw   = filteredYaw   * (1 - filterFactor) + yaw   * filterFactor;

  // Print stable values with correct naming
  Serial.print("Roll: ");
  Serial.print(filteredRoll, 2);
  Serial.print(" | Pitch: ");
  Serial.print(filteredPitch, 2);
  Serial.print(" | Yaw: ");
  Serial.println(filteredYaw, 2);

  delay(10);
}

void calibrateForTwoSeconds() {
  long accX_sum = 0, accY_sum = 0, accZ_sum = 0;
  long gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
  unsigned long startTime = millis();
  int count = 0;

  while (millis() - startTime < 2000) {  // Collect samples for 2 seconds
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

  Serial.println("Auto Calibration Offsets:");
  Serial.print("accX_offset: "); Serial.println(accX_offset);
  Serial.print("accY_offset: "); Serial.println(accY_offset);
  Serial.print("accZ_offset: "); Serial.println(accZ_offset);
  Serial.print("gyroX_offset: "); Serial.println(gyroX_offset);
  Serial.print("gyroY_offset: "); Serial.println(gyroY_offset);
  Serial.print("gyroZ_offset: "); Serial.println(gyroZ_offset);
}
