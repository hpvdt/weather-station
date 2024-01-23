#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SHTC3.h>
#include <WiFi.h>
#include <Adafruit_BMP3XX.h>
#include <BluetoothSerial.h>
#include <Wire.h>

//const int ledPin = 2;
#define LedPin 15
#define SEALEVELPRESSURE_HPA (1033.1)

Adafruit_BMP3XX bmp;
Adafruit_SHTC3 shtc3;
BluetoothSerial SerialBT;

void setup() {
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, HIGH);

  Serial.begin(9600);

  while (!Serial)
    delay(10);     // will pause until serial console opens

  SerialBT.begin("ESP32");
  Serial.println("Connect to Bluetooth");

  Serial.println("SHTC3 test");
  if (! shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    while (1) delay(1);
  }  Serial.println("Found SHTC3 sensor");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {  
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  SerialBT.println("BMP390 Data");
  SerialBT.printf("Temperature = %.2f *C", bmp.temperature);
  SerialBT.println();
  float pressurehPa = bmp.pressure/100;
  SerialBT.printf("Pressure = %.2f hPa", pressurehPa);
  SerialBT.println();

  SerialBT.printf("Altitude = %.2f m", bmp.readAltitude(SEALEVELPRESSURE_HPA));  
  SerialBT.println();

  SerialBT.println("SHTC3 Data");
  sensors_event_t humidity, temp;
  shtc3.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  SerialBT.printf("Temperature = %.2f *C", temp.temperature);
  SerialBT.println();

  SerialBT.printf("Humidity = %.2f %rH", humidity.relative_humidity);
  SerialBT.println();

  SerialBT.println();


  delay(2000);
}
