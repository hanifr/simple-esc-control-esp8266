/*
  Esp32 Websockets Client
*/

#include <ArduinoWebsockets.h>
#include <WiFi.h>
// #include <ESP8266WiFi.h>
#include <WiFi.h>
#include <Servo.h>

const char* ssid = "SEA-IC"; //Enter SSID
const char* password = "seaic2022"; //Enter Password
const char* websockets_server_host = "192.168.1.111"; //Enter server adress
const uint16_t websockets_server_port = 8888; // Enter server port

using namespace websockets;
int PWM;
int ref_PWM;
float PWM_CMD;
static const int ESC_pin[2] = {2, 14};
Servo ESC[2]; 

WebsocketsClient client;
bool wsConnected;
void setup() {
    Serial.begin(115200);
    for(int i = 0; i < 2; ++i) {
       if(!ESC[i].attach(ESC_pin[i], 1000, 2000)) {
           Serial.print("Servo ");
           Serial.print(i);
           Serial.println("attach error");
       }
    }
    delay(2000);
    connectWiFi();
    connectWebsocket();
}
void connectWiFi(){
    WiFi.begin(ssid, password);

    // Wait some time to connect to wifi
    for(int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
        Serial.print(".");
        delay(1000);
    }

    // Check if connected to wifi
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("No Wifi!");
        return;
    }

    Serial.println("Connected to Wifi, Connecting to server.");
}
void onMessageCallback(WebsocketsMessage message) {
    PWM = message.data().toInt();
}

void onEventsCallback(WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Connnection Opened");
        wsConnected = true;
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Connnection Closed");
        wsConnected = false;
    } else if(event == WebsocketsEvent::GotPing) {
        Serial.println("Got a Ping!");
    } else if(event == WebsocketsEvent::GotPong) {
        Serial.println("Got a Pong!");
    }
}

void connectWebsocket(){
    // Connect to server
    bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
    if(connected) {
        Serial.println("Connected!");
        PWM = 0;
    } else {
        Serial.println("Not Connected!");
    }

    // Setup Callbacks
    client.onMessage(onMessageCallback);
    client.onEvent(onEventsCallback);
}

bool pollWebsocket(){
  if (client.available()){
    client.poll(); // Get ready to receive next message
    return true;  
  }
  else{
    return false;
  }
}

void readJoystick() {
ref_PWM = PWM;

PWM_CMD = map(ref_PWM, 0, 1023, 1100, 1900);
}

void updateMotor(){
ESC[0].writeMicroseconds(PWM_CMD);
ESC[1].writeMicroseconds(PWM_CMD);
}

void loop() {

if (WiFi.status() == WL_CONNECTED) {
  if (pollWebsocket()) {
      readJoystick();
      updateMotor(); // move this line inside the "if (pollWebsocket())" block so the motor is updated only when new data is received
    }
  else {
  connectWebsocket();
    }
  }
else {
  connectWiFi();
  connectWebsocket();
  }
}