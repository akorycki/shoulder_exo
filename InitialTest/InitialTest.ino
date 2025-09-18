#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <Servo.h>

// ---- EMG Setup ----
const int emgPin = A0;           // EMG envelope output
const int smoothSamples = 10;    // smoothing window
int readings[smoothSamples];     // buffer for smoothing
int index = 0;                   // buffer index
long total = 0;                  // running total
int average = 0;                 // smoothed value
const int emgThreshold = 200;    // EMG activation threshold

// ---- Servo Setup ----
Servo myServo;
const int servoPin = 11;

// ---- AS5600 ----
Adafruit_AS5600 as5600;

uint16_t getScaledAngle() {
  // Read raw angle from AS5600 (0–4095)
  uint16_t raw = as5600.getRawAngle();

  // Shift so 3350 = 0 (adjustable offset)
  uint16_t shifted = (raw + 4096 - 3350) % 4096;

  return shifted;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // ---- Servo ----
  myServo.attach(servoPin);
  myServo.writeMicroseconds(1500); // neutral stop

  // ---- EMG buffer init ----
  for (int i = 0; i < smoothSamples; i++) {
    readings[i] = 0;
  }

  // ---- Initialize AS5600 ----
  if (!as5600.begin()) {
    Serial.println("AS5600 not detected. Check wiring.");
    while (1);
  }

  as5600.enableWatchdog(false);
  as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
  as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
  as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
  as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);

  as5600.setZPosition(0);
  as5600.setMPosition(4095);
  as5600.setMaxAngle(4095);

  Serial.println("System initialized: AS5600, EMG, Servo ready.");
}

void loop() {
  // ---- EMG Read & Smooth ----
  int emgValue = analogRead(emgPin); // raw 0–1023
  total -= readings[index];
  readings[index] = emgValue;
  total += readings[index];
  index = (index + 1) % smoothSamples;
  average = total / smoothSamples;

  // ---- AS5600 Data ----
  uint16_t scaled = 0;
  bool magnetOK = as5600.isMagnetDetected();
  if (magnetOK) {
    scaled = getScaledAngle();
  }

  // ---- Servo Logic ----
  if (magnetOK && scaled > 2000) {
    // Safety stop if encoder exceeds limit
    myServo.writeMicroseconds(1500);
  } else if (average > emgThreshold) {
    // Spin servo when muscle active
    myServo.writeMicroseconds(1750);
  } else {
    // Otherwise stop
    myServo.writeMicroseconds(1500);
  }

  // ---- Serial Output for Plotting & Debug ----
  Serial.print(emgValue); Serial.print(" "); Serial.print(average); // EMG raw + smooth
  Serial.print(" | Enc: ");
  if (magnetOK) {
    Serial.println(scaled);
  } else {
    Serial.println("NoMag");
  }

  delay(10); // Small delay for stability
}
