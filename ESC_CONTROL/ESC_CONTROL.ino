/*********
  M. H. M. Ramli 
  SEA-IC, UiTM Shah Alam
  Please report if there is any bugs in the program :: tronexia@gmail.com

  Program: XY-MD02 Transmitter and broadcast publicly all received data 
*********/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <WiFi.h>
// MQTT Broker
#include <MQTT.h>
#define sub1 "TRX/design/IO"
#define sub2 "TRX/design/req"
#define sub3 "TRX/design/res"
const char ssid[]     = "SEA-IC";              // replace with your WiFi SSID
const char password[] = "seaic2022";           // replace with your WiFi password
const char clientId[] = "RX-1";                // replace with your MQTT Client Id
const char server[]   = "txio.uitm.edu.my";    // replace with your MQTT Broker

WiFiClient net;
MQTTClient client;
//
//#include <Servo.h>
//
//static const int ESC_pin[2] = {9, 10};
//Servo ESC[2]; 

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
  // STEP 2 - Initialize Servo elements
//  for(int i = 0; i < 2; ++i) {
//        if(!ESC[i].attach(ESC_pin[i])) {
//            Serial.print("Servo ");
//            Serial.print(i);
//            Serial.println("attach error");
//        }
//    }
  // STEP 3 - Initialize WiFi connectivity
  initWiFi();
  client.begin(server, net);
  client.onMessage(messageReceived);

  // STEP 4 - Connecting to WiFi and MQTT broker
  connect();
}

void loop() {
 //check WiFi Connection
 WiFiCheckConnect();
 //check MQTT connection
  checkConnect();
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
 client.subscribe(sub2);
}
////////////////////////////////////////////////////////////////////
void messageReceived(String &topic, String &payload) {
//Serial.println("incoming: " + topic + " - " + payload); 
  Serial.println("Topic: " +  topic);
  Serial.println("Payload: " +  payload);
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
  String sensorType = topic;

  if (sensorType == sub1)
  {
    String PWM_1  = payload.substring(0,payload.indexOf(","));
    int PWM1 = PWM_1.toInt();
    String PWM_2  = payload.substring(payload.indexOf(",")+1);
    int PWM2 = PWM_2.toInt();
    Serial.println("PWM 1: " + String(PWM1) + ", PWM 2: " + String(PWM2));
//    ESC[0].write(PWM1); ESC[1].write(PWM2);
  }
}
////////////////////////////////////////////////////////////////////
void checkConnect(){
  client.loop();

  if (!client.connected()) {
    connect();
  }
}
//-------------------------------------------------//