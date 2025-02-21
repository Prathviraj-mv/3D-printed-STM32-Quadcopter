#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// Create the BMP085 (or BMP180) object from the Unified library
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Adjust to your local sea-level pressure for best accuracy (in hPa)
#define SEALEVELPRESSURE_HPA (1013.25)

// This will store our “baseline” altitude in cm so that we can “zero” at startup
float baselineAltitudeCM = 0.0;

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP180 Zero-Calibration in Centimeters"));

  // Initialize the BMP180
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP085/BMP180 sensor, check wiring!"));
    while (1);
  }
  // Give the sensor a moment to stabilize
  delay(1000);

  // Read the current altitude (in meters), convert to centimeters, store as baseline
  baselineAltitudeCM = getAltitudeCM();
  Serial.println(F("Calibration complete."));
  Serial.print(F("Baseline altitude set to: "));
  Serial.print(baselineAltitudeCM, 2);
  Serial.println(F(" cm"));
  Serial.println(F("------------------------------------"));
}

void loop() {
  // Get the current altitude in centimeters
  float currentAltitudeCM = getAltitudeCM();

  // Compute relative altitude (difference from baseline)
  float relativeAltitudeCM = currentAltitudeCM - baselineAltitudeCM;

  // Print the relative altitude
  Serial.print(F("Relative Altitude = "));
  Serial.print(relativeAltitudeCM, 2);
  Serial.println(F(" cm"));

  // (Optional) Slow down loop
  delay(1000);
}

float getAltitudeCM() {
  sensors_event_t event;
  bmp.getEvent(&event);

  if (event.pressure) {
    // Get temperature in Celsius
    float temperature;
    bmp.getTemperature(&temperature);

    // Convert the current pressure + temperature to altitude (in meters)
    float altitudeM = bmp.pressureToAltitude(SEALEVELPRESSURE_HPA, event.pressure, temperature);

    // Convert meters to centimeters
    float altitudeCM = altitudeM * 100.0;
    return altitudeCM;
  }

}
