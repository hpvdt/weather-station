#include <Arduino.h>
#include <Adafruit_SHTC3.h>
#include <WiFi.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// Set pin for power indicator LED and WiFi connected LED
#define LedPin 15
#define Wifi_indicator 13

// Set sea level pressure for determining altitude
#define SEALEVELPRESSURE_HPA (1013.25)

// Set up WiFi credentials
const char* ssid     = "Weather Station";

// Initalize sensor and bluetooth objects
Adafruit_BMP3XX bmp;
Adafruit_SHTC3 shtc3;

// Set the port number for the WiFi server
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

// Function to calculate air density
float air_density(float p, float t, float h){
  float P1 = pow((6.1078*10), ((7.5*t)/(t+237.3)));
  float PV = P1*h;
  float PD = (p*100) - PV;
  float rho = (PD/(287.058*(t+273.3))) + (PV/(461.495*(t+273.3)));
  return rho;
}

// Get Sensor Readings and return JSON object  
String getSensorReadings(){
  JSONVar readings;
  readings["temperature"] = String(bmp.readTemperature());
  sensors_event_t humidity, temp;
  shtc3.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  float N = (log(humidity.relative_humidity/100)+((17.25*bmp.readTemperature())/(237.3+bmp.readTemperature())))/17.27;
  float D = (237.3*N)/(1-N);
  readings["dewpoint"] = String(D);
  readings["humidity"] = String(humidity.relative_humidity);
  readings["pressure"] = String(bmp.readPressure()/100.0F);
  readings["altitude"] = String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  readings["density"] = String(air_density(bmp.readPressure(), bmp.readTemperature(), humidity.relative_humidity));
  String jsonString = JSON.stringify(readings);
  return jsonString;
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi Server
void initWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  delay(100);
  IPAddress IP(192, 168, 10, 1);
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(IP, IP, NMask);
}

// Function to notify all clients with updated readings
void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
}

// Function to handle requests for new data
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = (char*)data;
    //Check if the message is "getReadings"
    if (strcmp((char*)data, "getReadings") == 0) {
    //if it is, send current sensor readings
      String sensorReadings = getSensorReadings();
      Serial.print(sensorReadings);
      notifyClients(sensorReadings);
    }
  }
}

// Function to handle websocket events other than new data
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// Initialize WebSocket
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  // Set up power indicator LED
  pinMode(LedPin, OUTPUT);
  pinMode(Wifi_indicator, OUTPUT);
  digitalWrite(LedPin, HIGH);

  // open serial 
  Serial.begin(115200);
  // whil
  //delay(10);     // will pause until serial console opens
  
  // initialize 
  initWiFi();
  initSPIFFS();
  initWebSocket();

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  // Start server
  server.begin();

  // Start SCHT3 Sensor
  Serial.println("SHTC3 test");
  if (! shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    while (1) delay(1);
  }  Serial.println("Found SHTC3 sensor");

  //Start BMP390 Sensor
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
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
  if ((millis() - lastTime) > timerDelay) {
    digitalWrite(Wifi_indicator, HIGH);
    String sensorReadings = getSensorReadings();
    Serial.print(sensorReadings);
    notifyClients(sensorReadings);

    lastTime = millis();
  }
  digitalWrite(Wifi_indicator, LOW);
  ws.cleanupClients();
}