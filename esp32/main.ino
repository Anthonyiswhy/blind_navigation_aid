
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h> //
// Ultrasonic defines
#define TRIG_PIN 5
#define ECHO_PIN 18

// ToF XSHUT pins
#define XSHUT1 25
#define XSHUT2 26

VL53L0X tof1;
VL53L0X tof2;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 100; // 10 Hz

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ToF init (Pololu style)
  Wire.begin();
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);

  // Bring up tof1, set address 0x30
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  tof1.init(true);        // Pololu style; for Adafruit use tof1.begin()
  tof1.setAddress(0x30);
  tof1.startContinuous();

  // Bring up tof2, set address 0x31
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  tof2.init(true);
  tof2.setAddress(0x31);
  tof2.startContinuous();

  Serial.println("ESP32 sensors ready");
}

int readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  int cm = (int)(duration * 0.034 / 2.0);
  if (cm <= 0 || cm > 400) return -1;
  return cm;
}

void loop() {
  int t1 = -1, t2 = -1, ultra = -1;
  // Safe reads from ToFs (Pololu)
  if (tof1.readRangeContinuousMillimeters() > 0) {
    t1 = (int)(tof1.readRangeContinuousMillimeters() / 10);
  } else {
    t1 = -1;
  }
  if (tof2.readRangeContinuousMillimeters() > 0) {
    t2 = (int)(tof2.readRangeContinuousMillimeters() / 10);
  } else {
    t2 = -1;
  }
  ultra = readUltrasonic();

  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    // Format: #tof1,tof2,ultra
    Serial.printf("#%d,%d,%d\n", (t1 > 0 ? t1 : 0), (t2 > 0 ? t2 : 0), (ultra > 0 ? ultra : 0));
  }
  delay(10);
}
