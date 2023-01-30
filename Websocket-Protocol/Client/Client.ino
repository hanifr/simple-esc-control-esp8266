/*
  Esp32 Websockets Client
*/

#include <ArduinoWebsockets.h>
#include <WiFi.h>
// #include <ESP8266WiFi.h>
#include <Servo.h>

const char* ssid = "SEA-IC"; 
const char* password = "seaic2022"; 
const char* websockets_server_host = "192.168.1.111"; 
const uint16_t websockets_server_port = 8888; 

using namespace websockets;
int ref_PWM;
float PWM_CMD;
//static const int ESC_pin[2] = {2, 14};
const int ESC_pin[2] = {2, 14};
Servo ESC[2]; 

WebsocketsClient client;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 2; ++i) {
    if (!ESC[i].attach(ESC_pin[i], 1000, 2000)) {
      Serial.print("Servo ");
      Serial.print(i);
      Serial.println("attach error");
    }
  }
  delay(2000);
  connectWiFi();
  connectWebsocket();
}

void connectWiFi() {
    WiFi.begin(ssid, password);
    int i = 0;
    while (WiFi.status() != WL_CONNECTED && i < 10) {
        Serial.print(".");
        delay(1000);
        i++;
    }
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("No Wifi!");
        return;
    }
    Serial.println("Connected to Wifi, Connecting to server.");
}

void connectWebsocket() {
    if (client.connect(websockets_server_host, websockets_server_port, "/")) {
        Serial.println("Connected!");
    } else {
        Serial.println("Not Connected!");
    }
    client.onMessage([&](WebsocketsMessage message){
//        Serial.print("Got Message: ");
        Serial.println(message.data());
        ref_PWM = atoi(message.data().c_str());
        PWM_CMD = map(ref_PWM, 0, 1023, 1100, 1900);
        ESC[0].writeMicroseconds(PWM_CMD);
        ESC[1].writeMicroseconds(PWM_CMD);
    });
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
    if (client.available()) {
      client.poll();
    }
  } else {
    Exit();
  }
  delay(50);
}

void Exit() {
  WiFi.disconnect();
  delay(500);
  ESP.restart();
}
