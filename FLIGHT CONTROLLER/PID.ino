// ROLL

float roll_angle; // from mpu
float roll_setpoint; //from transmitter
float throttle;

float error, previous_error = 0, integral = 0, derivative, output; // pid variables
float Kp = 3.0, Ki = 0.02, Kd = 8.0;
unsigned long prevTime;

void setup() {
  // Setup PWM outputs & input reading
  prevTime = micros();
}

void loop() {
  // Get roll_angle from MPU6050
  roll_angle = getRollAngle();  // Your custom function  get value from mpu
  
  // Read roll_setpoint and throttle from receiver
  roll_setpoint = map(roll_pwm_input, 1000, 2000, -30, 30); // map throttle
  throttle = map(throttle_pwm_input, 1000, 2000, 1100, 1900); // Esc-safe range
  
  // Calculate PID
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  error = roll_setpoint - roll_angle;
  integral += error * dt;
  derivative = (error - previous_error) / dt;

  output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  // Motor mixing
  int motor1 = throttle + output;
  int motor2 = throttle - output;

  motor1 = constrain(motor1, 1100, 1900);
  motor2 = constrain(motor2, 1100, 1900);

  // Output to ESCs
  pwmWrite(MOTOR1_PIN, motor1);
  pwmWrite(MOTOR2_PIN, motor2);
}







/// ROLL AND  PITCH

// Roll PID variables
float roll_angle, roll_setpoint;
float roll_error, roll_prev_error = 0, roll_integral = 0, roll_derivative, roll_output;
float Kp_roll = 3.0, Ki_roll = 0.02, Kd_roll = 8.0;

// Pitch PID variables
float pitch_angle, pitch_setpoint;
float pitch_error, pitch_prev_error = 0, pitch_integral = 0, pitch_derivative, pitch_output;
float Kp_pitch = 3.0, Ki_pitch = 0.02, Kd_pitch = 8.0;

unsigned long prevTime = 0;

void computePID() {
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  // ----- ROLL PID -----
  roll_error = roll_setpoint - roll_angle;
  roll_integral += roll_error * dt;
  roll_derivative = (roll_error - roll_prev_error) / dt;
  roll_output = Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative;
  roll_prev_error = roll_error;

  // ----- PITCH PID -----
  pitch_error = pitch_setpoint - pitch_angle;
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
  pwmWrite(MOTOR1_PIN, motor1);
  pwmWrite(MOTOR2_PIN, motor2);
  pwmWrite(MOTOR3_PIN, motor3);
  pwmWrite(MOTOR4_PIN, motor4);
}


// ROLL PITCH YAW

// Roll PID variables
float roll_angle, roll_setpoint;
float roll_error, roll_prev_error = 0, roll_integral = 0, roll_derivative, roll_output;
float Kp_roll = 3.0, Ki_roll = 0.02, Kd_roll = 8.0;

// Pitch PID variables
float pitch_angle, pitch_setpoint;
float pitch_error, pitch_prev_error = 0, pitch_integral = 0, pitch_derivative, pitch_output;
float Kp_pitch = 3.0, Ki_pitch = 0.02, Kd_pitch = 8.0;

// Yaw PID variables
float yaw_angle, yaw_setpoint;
float yaw_error, yaw_prev_error = 0, yaw_integral = 0, yaw_derivative, yaw_output;
float Kp_yaw = 2.0, Ki_yaw = 0.01, Kd_yaw = 5.0;

unsigned long prevTime = 0;

void computePID() {
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  // --- Roll PID ---
  roll_error = roll_setpoint - roll_angle;
  roll_integral += roll_error * dt;
  roll_derivative = (roll_error - roll_prev_error) / dt;
  roll_output = Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative;
  roll_prev_error = roll_error;

  // --- Pitch PID ---
  pitch_error = pitch_setpoint - pitch_angle;
  pitch_integral += pitch_error * dt;
  pitch_derivative = (pitch_error - pitch_prev_error) / dt;
  pitch_output = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative;
  pitch_prev_error = pitch_error;

  // --- Yaw PID ---
  yaw_error = yaw_setpoint - yaw_angle;
  yaw_integral += yaw_error * dt;
  yaw_derivative = (yaw_error - yaw_prev_error) / dt;
  yaw_output = Kp_yaw * yaw_error + Ki_yaw * yaw_integral + Kd_yaw * yaw_derivative;
  yaw_prev_error = yaw_error;
}

void motorMixing(int throttle) {
  // Motor Mixing for Quad X
  int motor1 = throttle + pitch_output + roll_output - yaw_output; // Front Right
  int motor2 = throttle + pitch_output - roll_output + yaw_output; // Front Left
  int motor3 = throttle - pitch_output - roll_output - yaw_output; // Rear Left
  int motor4 = throttle - pitch_output + roll_output + yaw_output; // Rear Right

  motor1 = constrain(motor1, 1100, 1900);
  motor2 = constrain(motor2, 1100, 1900);
  motor3 = constrain(motor3, 1100, 1900);
  motor4 = constrain(motor4, 1100, 1900);

  // Write PWM signals to ESC pins
  pwmWrite(MOTOR1_PIN, motor1);
  pwmWrite(MOTOR2_PIN, motor2);
  pwmWrite(MOTOR3_PIN, motor3);
  pwmWrite(MOTOR4_PIN, motor4);
}
