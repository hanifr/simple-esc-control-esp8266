/*********
  M. H. M. Ramli 
  SEA-IC, UiTM Shah Alam
  Please report if there is any bugs in the program :: tronexia@gmail.com

  Program: ESP8266 transmitter for acquire joystick signal in PWM
*********/
// #include <Arduino.h>
// #include <SoftwareSerial.h>
// #include <Wire.h>
#include <WiFi.h>
// MQTT Broker
#include <MQTT.h>
#define sub1 "TRX/design/IO"
// #define sub2 "TRX/design/req"
// #define sub3 "TRX/design/res"
const char ssid[]     = "SEA-IC";              // replace with your WiFi SSID
const char password[] = "seaic2022";           // replace with your WiFi password
const char clientId[] = "TXX-1";                // replace with your MQTT Client Id
const char server[]   = "XXXXXXXXX";    // replace with your MQTT Broker

WiFiClient net;
MQTTClient client;

// joystick signal value
int JSTICK;

// time constants
 unsigned long lastMillis = 0;


unsigned long previousMillis = 0;
 unsigned long interval = 30000;

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void WiFiCheckConnect(){
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
}
void setup() {
  // STEP 1: initialize Serial Monitor
  Serial.begin(115200);
  // STEP 2 - Initialize WiFi connectivity
  initWiFi();
  client.begin(server, net);
//   client.onMessage(messageReceived);
  // STEP 3 - Connecting to WiFi and MQTT broker
  connect();
}

void loop() {
 //check WiFi Connection
 WiFiCheckConnect();
 //check MQTT connection
  checkConnect();
// Read Analog value
  JSTICK = analogRead(A0);
  JSTICK = constrain(JSTICK, 0, 1023);
  client.publish(sub1, String(JSTICK));
  delay(50);
}
//-----------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////
// Sub-function to connect to WiFi and MQTT broker
void connect() {
  
  Serial.print("Connecting to WiFi ...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" connected!");

  Serial.print("Connecting to MQTT broker ...");
  while (!client.connect(clientId)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" connected!");

 // client.subscribe(topic);
 client.subscribe(sub1);
}
////////////////////////////////////////////////////////////////////
void checkConnect(){
  client.loop();

  if (!client.connected()) {
    connect();
  }
}
//-------------------------------------------------//