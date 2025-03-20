#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

unsigned long previousTime = 0;
float dt;

float accAngleX, accAngleY;
float gyroAngleX = 0, gyroAngleY = 0;
float compAngleX = 0, compAngleY = 0;
float alpha = 0.98;  // complementary filter factor

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

  Serial.println("Calibration done:");
  Serial.print("Gyro X offset: "); Serial.println(gyroX_offset);
  Serial.print("Gyro Y offset: "); Serial.println(gyroY_offset);
  Serial.print("Gyro Z offset: "); Serial.println(gyroZ_offset);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  calibrateMPU();  // Do the gyro offset calibration
  previousTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to g and degrees/second
  float accX = (float)ax / 16384.0;
  float accY = (float)ay / 16384.0;
  float accZ = (float)az / 16384.0;

  // Subtract offsets and convert to °/s
  float gyroXrate = ((float)gx - gyroX_offset) / 131.0;  // for ±250 dps
  float gyroYrate = ((float)gy - gyroY_offset) / 131.0;
  float gyroZrate = ((float)gz - gyroZ_offset) / 131.0;

  // Calculate angles from accelerometer
  accAngleX = atan2(accY, accZ) * 180 / PI;
  accAngleY = atan2(-accX, accZ) * 180 / PI;

  // Integrate gyro angles
  gyroAngleX += gyroXrate * dt;
  gyroAngleY += gyroYrate * dt;

  // Complementary filter to combine both
  compAngleX = alpha * (compAngleX + gyroXrate * dt) + (1 - alpha) * accAngleX;
  compAngleY = alpha * (compAngleY + gyroYrate * dt) + (1 - alpha) * accAngleY;

  // Print results
  Serial.print("CompAngleX: "); Serial.print(compAngleX);
  Serial.print("\tCompAngleY: "); Serial.print(compAngleY);
  Serial.print("\tGyroRateZ (Yaw rate): "); Serial.println(gyroZrate);

  delay(10);
}
