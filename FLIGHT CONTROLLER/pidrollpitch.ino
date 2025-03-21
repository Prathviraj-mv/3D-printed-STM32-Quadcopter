// Roll PID variables
float compAngleX, rollAngle;
float roll_error, roll_prev_error = 0, roll_integral = 0, roll_derivative, roll_output;
float Kp_roll = 3.0, Ki_roll = 0.02, Kd_roll = 8.0;

// Pitch PID variables
float compAngleY, pitchAngle;
float pitch_error, pitch_prev_error = 0, pitch_integral = 0, pitch_derivative, pitch_output;
float Kp_pitch = 3.0, Ki_pitch = 0.02, Kd_pitch = 8.0;

unsigned long prevTime = 0;

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
  pwmWrite(motorPins[0], motor1);
  pwmWrite(motorPins[0], motor2);
  pwmWrite(motorPins[0], motor3);
  pwmWrite(motorPins[0], motor4);
}
