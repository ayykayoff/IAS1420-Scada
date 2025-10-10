#include <ModbusRTU.h>

// --- Pins ---
const int enA    = 9;    // L298N ENA (PWM) — ENA jumper must be removed
const int in1    = 8;    // Direction (kept HIGH)
const int in2    = 7;    // Direction (kept LOW)
const int potPin = A0;   // Potentiometer -> setpoint (0..100 °C)
const int tempPin= A1;   // Temperature sensor (0..5V = 0..100 °C)
const int ledPin = 2;    // Over-temp LED

// --- Fan PWM behavior ---
const int PWM_MIN = 245; // your fan starts only from ~240
const int PWM_MAX = 255; // L298N full
const float FULL_SPEED_SPAN = 20.0; // °C above setpoint to reach full speed
const float LED_HYST = 0.5;         // °C hysteresis to avoid LED flicker

int lastPWM = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // One direction only (BLDC fan)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(enA, 0);
  digitalWrite(ledPin, LOW);

  Serial.begin(9600);
  Serial.println("Thermostat mode: pot=A0 sets °C, fan follows temperature.");
}

void loop() {
  // --- Read inputs ---
  int potValue = analogRead(potPin);   // 0..1023
  int rawTemp  = analogRead(tempPin);  // 0..1023

  // Map pot to setpoint 0..100 °C
  float setpointC = (potValue * 100.0) / 1023.0;

  // Calibrated temperature: 0V=0°C, 5V=100°C
  float temperatureC = (rawTemp * 100.0) / 1023.0;

  // --- Control law ---
  // If temp <= setpoint: fan off. If above, scale linearly from PWM_MIN to PWM_MAX
  float error = temperatureC - setpointC;
  int targetPWM = 0;

  if (error > 0) {
    float ratio = error / FULL_SPEED_SPAN;    // 0.0 .. 1.0+
    if (ratio > 1.0) ratio = 1.0;
    targetPWM = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * ratio);
    if (targetPWM < PWM_MIN) targetPWM = PWM_MIN; // ensure start threshold
    if (targetPWM > PWM_MAX) targetPWM = PWM_MAX;
  }

  // --- Start kick: if we were stopped and need to run, give a short 100% pulse
  if (lastPWM == 0 && targetPWM > 0) {
    analogWrite(enA, 255);
    delay(250); // short kick to guarantee spin-up
  }
  analogWrite(enA, targetPWM);
  lastPWM = targetPWM;

  // --- LED logic (with tiny hysteresis to avoid chatter) ---
  static bool ledOn = false;
  if (!ledOn && (temperatureC > setpointC + LED_HYST)) {
    ledOn = true;
    digitalWrite(ledPin, HIGH);
  } else if (ledOn && (temperatureC < setpointC - LED_HYST)) {
    ledOn = false;
    digitalWrite(ledPin, LOW);
  }

  // --- Debug ---
  Serial.print("Setpoint: ");
  Serial.print(setpointC, 1);
  Serial.print(" C  | Temp: ");
  Serial.print(temperatureC, 1);
  Serial.print(" C  | PWM: ");
  Serial.print(targetPWM);
  Serial.print("  | LED: ");
  Serial.println(ledOn ? "ON" : "OFF");

  delay(200);
}
