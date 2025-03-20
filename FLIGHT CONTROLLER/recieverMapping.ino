#define NUM_CHANNELS 4  // Channels for Throttle, Roll, Pitch, Yaw
#define NUM_MOTORS 4    // Motors M1, M2, M3, M4

// Channel Pins (Receiver PWM Inputs)
const int pwmPins[NUM_CHANNELS] = {PB8, PA15, PB6, PA7};

// ESC Output Pins
const int motorPins[NUM_MOTORS] = {PA0, PA1, PB0, PA6};

// Variables for timing
volatile unsigned long riseTime[NUM_CHANNELS];
volatile unsigned long pwmValues[NUM_CHANNELS];
volatile bool newSignal[NUM_CHANNELS] = {false};

void risingEdge(int channel) {
    riseTime[channel] = micros();
    attachInterrupt(pwmPins[channel], [channel]() { fallingEdge(channel); }, FALLING);
}

void fallingEdge(int channel) {
    pwmValues[channel] = micros() - riseTime[channel];
    newSignal[channel] = true;
    attachInterrupt(pwmPins[channel], [channel]() { risingEdge(channel); }, RISING);
}

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < NUM_CHANNELS; i++) {
        pinMode(pwmPins[i], INPUT);
        attachInterrupt(pwmPins[i], [i]() { risingEdge(i); }, RISING);
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(motorPins[i], OUTPUT);
    }
}

void writeMotor(int motorPin, int value) {
    value = constrain(value, 1000, 2000); // ESC signal range
    int pulseWidth = map(value, 1000, 2000, 1000, 2000);
    digitalWrite(motorPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(motorPin, LOW);
}

void loop() {
    int throttle = pwmValues[2];
    int roll = pwmValues[0];
    int pitch = pwmValues[1];
    int yaw = pwmValues[3];

float rollAngle = ((float)(roll - 1500) / 500.0) * 30.0;
  float pitchAngle = ((float)(pitch - 1500) / 500.0) * 30.0;
  float yawRate = ((float)(yaw - 1500) / 500.0) * 30.0;


  Serial.print("Throttle: ");
  Serial.print(throttle);
  Serial.print(" | Roll Angle: ");
  Serial.print(rollAngle, 2);
  Serial.print(" | Pitch Angle: ");
  Serial.print(pitchAngle, 2);
  Serial.print(" | Yaw Rate: ");
  Serial.println(yawRate, 2);
  delay(20);
}
