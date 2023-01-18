/*
	Esp32 Websockets Client
*/

#include <ArduinoWebsockets.h>
#include <WiFi.h>
// #include <ESP8266WiFi.h>
#include <Servo.h>

const char* ssid = "ssid"; //Enter SSID
const char* password = "password"; //Enter Password
const char* websockets_server_host = "serverip_or_name"; //Enter server adress
const uint16_t websockets_server_port = 8888; // Enter server port

using namespace websockets;
int PWM;
int* ref_PWM;
float PWM_CMD;
static const int ESC_pin[2] = {2, 14};
Servo ESC[2]; 

WebsocketsClient client;
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
    // Connect to wifi
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
    // try to connect to Websockets server
    connectWebsocket();
}
void onMessageCallback(WebsocketsMessage message) {
    // const char* msg = (message.data()).toInt();
    PWM = (message.data()).toInt();;
    // int x = atoi(myData);
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
  client.connect(websockets_server_host, websockets_server_port, "/");

  Serial.println("Connected to Wifi, Connecting to server.");
  // try to connect to Websockets server
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

int* getControlReference(){
  return PWM;
}
void readJoystick() {
  ref_PWM = getControlReference();

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


    }
    else {
      connectWebsocket();
    }
  }
  else {
    connectWifi();
    connectWebsocket();
  }
  updateMotor();
}