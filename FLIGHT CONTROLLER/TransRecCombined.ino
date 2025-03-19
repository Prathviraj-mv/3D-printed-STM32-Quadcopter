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

    if (throttle > 1200) {
        if (roll < 1350) {
            if (throttle > 1600) {
                int increase = throttle + (throttle * 0.25);
                int decrease = throttle - (throttle * 0.25);
                writeMotor(motorPins[0], decrease);
                writeMotor(motorPins[1], decrease);
                writeMotor(motorPins[2], increase);
                writeMotor(motorPins[3], increase);
            } else {
                int increase = throttle + (throttle * 0.25);
                writeMotor(motorPins[0], throttle);
                writeMotor(motorPins[1], throttle);
                writeMotor(motorPins[2], increase);
                writeMotor(motorPins[3], increase);
            }
        } else if (roll > 1650) {
            if (throttle > 1600) {
                int increase = throttle + (throttle * 0.25);
                int decrease = throttle - (throttle * 0.25);
                writeMotor(motorPins[0], increase);
                writeMotor(motorPins[1], increase);
                writeMotor(motorPins[2], decrease);
                writeMotor(motorPins[3], decrease);
            } else {
                int increase = throttle + (throttle * 0.25);
                writeMotor(motorPins[0], increase);
                writeMotor(motorPins[1], increase);
                writeMotor(motorPins[2], throttle);
                writeMotor(motorPins[3], throttle);
            }
        } else if (pitch < 1350) {
            if (throttle > 1600) {
                int increase = throttle + (throttle * 0.25);
                int decrease = throttle - (throttle * 0.25);
                writeMotor(motorPins[0], increase);
                writeMotor(motorPins[3], increase);
                writeMotor(motorPins[1], decrease);
                writeMotor(motorPins[2], decrease);
            } else {
                int increase = throttle + (throttle * 0.25);
                writeMotor(motorPins[0], increase);
                writeMotor(motorPins[3], increase);
                writeMotor(motorPins[1], throttle);
                writeMotor(motorPins[2], throttle);
            }
        } else if (pitch > 1650) {
            if (throttle > 1600) {
                int increase = throttle + (throttle * 0.25);
                int decrease = throttle - (throttle * 0.25);
                writeMotor(motorPins[1], increase);
                writeMotor(motorPins[2], increase);
                writeMotor(motorPins[0], decrease);
                writeMotor(motorPins[3], decrease);
            } else {
                int increase = throttle + (throttle * 0.25);
                writeMotor(motorPins[1], increase);
                writeMotor(motorPins[2], increase);
                writeMotor(motorPins[0], throttle);
                writeMotor(motorPins[3], throttle);
            }
        } else if (yaw < 1350) {
            if (throttle > 1600) {
                int increase = throttle + (throttle * 0.25);
                int decrease = throttle - (throttle * 0.25);
                writeMotor(motorPins[0], increase);
                writeMotor(motorPins[2], increase);
                writeMotor(motorPins[1], decrease);
                writeMotor(motorPins[3], decrease);
            } else {
                int increase = throttle + (throttle * 0.25);
                writeMotor(motorPins[0], increase);
                writeMotor(motorPins[2], increase);
                writeMotor(motorPins[1], throttle);
                writeMotor(motorPins[3], throttle);
            }
        } else if (yaw > 1650) {
            if (throttle > 1600) {
                int increase = throttle + (throttle * 0.25);
                int decrease = throttle - (throttle * 0.25);
                writeMotor(motorPins[1], increase);
                writeMotor(motorPins[3], increase);
                writeMotor(motorPins[0], decrease);
                writeMotor(motorPins[2], decrease);
            } else {
                int increase = throttle + (throttle * 0.25);
                writeMotor(motorPins[1], increase);
                writeMotor(motorPins[3], increase);
                writeMotor(motorPins[0], throttle);
                writeMotor(motorPins[2], throttle);
            }
        } else {
            for (int i = 0; i < NUM_MOTORS; i++) {
                writeMotor(motorPins[i], throttle);
            }
        }
    } else {
        for (int i = 0; i < NUM_MOTORS; i++) {
            writeMotor(motorPins[i], 1000);
        }
    }

    Serial.print("Throttle: "); Serial.print(throttle);
    Serial.print(" | Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.println(yaw);
    delay(20);
}
