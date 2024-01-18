#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SHTC3.h>
// #include <DFRobot_BMP3XX.h>
#include <WiFi.h>

const int ledPin[] = {12, 13, 14, 15};

void setup() {
  for (int i = 0; i < 4; i ++) pinMode(ledPin[i], OUTPUT);
}

void loop() {
  for (int i = 0; i < 4; i ++) digitalWrite(ledPin[i], HIGH);
  delay(1000);
  for (int i = 0; i < 4; i ++) digitalWrite(ledPin[i], LOW);
  delay(1000);
}
