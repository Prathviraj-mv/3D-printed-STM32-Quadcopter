#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// Create the BMP085 (or BMP180) object from the Unified library
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

#define SEALEVELPRESSURE_HPA (1013.25)
#define BUTTON_PIN 7  // Define button pin

float baselineAltitudeCM = 0.0;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};
float PitchOffset = 23.2;
float RollOffset = 0.0;

// PID parameters
float Kp = 1.2, Ki = 0.02, Kd = 0.5;
float prevErrorRoll = 0, integralRoll = 0;
float prevErrorPitch = 0, integralPitch = 0;
int minThrottle = 1000, maxThrottle = 2000;

// Height control
float targetAltitudeCM = 300.0;  // Adjustable hover altitude
float altitudeToleranceCM = 50.0; // Hover range between 250cm and 350cm
unsigned long hoverDuration = 10000; // Hover duration in milliseconds
bool hoverMode = false;
unsigned long hoverStartTime = 0;
bool takeoffInitiated = false;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

float pidControl(float error, float &prevError, float &integral) {
  integral += error * 0.004;
  float derivative = (error - prevError) / 0.004;
  prevError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

void updateMotors(float altitudeError) {
  float rollError = -KalmanAngleRoll;
  float pitchError = -KalmanAnglePitch;
  
  float rollCorrection = pidControl(rollError, prevErrorRoll, integralRoll);
  float pitchCorrection = pidControl(pitchError, prevErrorPitch, integralPitch);
  float altitudeCorrection = pidControl(altitudeError, prevErrorPitch, integralPitch);
  
  int throttleA0 = constrain(1500 + rollCorrection - pitchCorrection + altitudeCorrection, minThrottle, maxThrottle);
  int throttleA1 = constrain(1500 - rollCorrection - pitchCorrection + altitudeCorrection, minThrottle, maxThrottle);
  int throttleA2 = constrain(1500 + rollCorrection + pitchCorrection + altitudeCorrection, minThrottle, maxThrottle);
  int throttleA3 = constrain(1500 - rollCorrection + pitchCorrection + altitudeCorrection, minThrottle, maxThrottle);
  
  analogWrite(A0, throttleA0);
  analogWrite(A1, throttleA1);
  analogWrite(A2, throttleA2);
  analogWrite(A3, throttleA3);
}

void setup() {
  Serial.begin(57600);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  baselineAltitudeCM = getAltitudeCM();
  LoopTimer = micros();
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW && !takeoffInitiated) {
    hoverMode = true;
    hoverStartTime = millis();
    takeoffInitiated = true;
  }

  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  
  float currentAltitude = getAltitudeCM();
  float altitudeError = 0;
  if (hoverMode) {
    if (millis() - hoverStartTime < hoverDuration) {
      if (currentAltitude < targetAltitudeCM - altitudeToleranceCM) {
        altitudeError = targetAltitudeCM - currentAltitude;
      } else if (currentAltitude > targetAltitudeCM + altitudeToleranceCM) {
        altitudeError = targetAltitudeCM - currentAltitude;
      }
    } else {
      hoverMode = false;
      takeoffInitiated = false;
    }
  }
  
  updateMotors(altitudeError);
  
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
