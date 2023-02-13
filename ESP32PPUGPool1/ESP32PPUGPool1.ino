#include <WiFi.h>
// MQTT Broker
#include <MQTT.h>
#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
// https://dl.espressif.com/dl/package_esp32_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json

SoftwareSerial modbus(13, 15);

#define sub1 "np/ppug/s/data"
#define sub2 "np/ppug/s/req"
#define sub3 "np/ppug/s/res"
//const char ssid[]     = "Thermal System";    // replace with your WiFi SSID
//const char password[] = "thermalpoc2022";    // replace with your WiFi password
const char ssid[]     = "txio1";              // replace with your WiFi SSID
const char password[] = "12345678";           // replace with your WiFi password
const char clientId[] = "PPUG-TMP";                // replace with your MQTT Client Id
const char server[]   = "www.txio.live";    // replace with your MQTT Broker

WiFiClient net;
MQTTClient client;


int loraId = 1;
// time constants for delays
unsigned long lastMillis = 0;
int aquacultureError = 0;
#define DELAY_1M (1UL * 60 * 1000) // 1 Minute
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET     4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Register for Aqua-Sensors
//-----------------------------------------------------------------------------------
#define TXENABLE                            2
#define FRAMESIZE_M                        25
#define BYTE_00_M                        0x05
#define BYTE_01_M                        0x03
#define BYTE_02_M                        0x00
#define BYTE_03_M                        0x00
#define BYTE_04_M                        0x00
#define BYTE_05_M                        0x0A
#define BYTE_RESPONSE_M                  0x14
#define MAX_MILLIS_TO_WAIT_M             2000

// Register for Water-level-Sensors
//-----------------------------------------------------------------------------------
#define MAX_MILLIS_TO_WAIT                  300                                
#define FRAMESIZE                           9                                   
#define WLV1_ADDR                           0x01      // BYTE 1
#define WLV2_ADDR                           0x03      // BYTE 1  
#define WL_REG                              0x03      // BYTE 4: Water Level Reservoir   m     BYTE_3 0x00, BYTE_7 0x74 BYTE_8 0x0A
#define FCN_CODE                            0x03      // BYTE 2
#define BYTE_03                             0x00      // BYTE 3

#define BYTE_05                             0x00      // BYTE 5
#define BYTE_06                             0x01      // BYTE 6
//-----------------------------------------------------------------------------------

//Modbus Constants and parameters
float TMP, DO, DD, PH, NH4;
String DTX, REBOOT, TMPData;
int TMPVD;
unsigned long previousMillis = 0;
unsigned long interval = 30000;

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
  Serial.begin(9600);
  modbus.begin(9600);
  pinMode(TXENABLE, OUTPUT);

// LCD Settings
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println(F("SSD1306 allocation failed"));
    display.clearDisplay();
    display.setRotation(2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, 0);
    display.print(F("NEXPLEX LoRa:"));
    display.display();


  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(18, 23, 26);
  if (!LoRa.begin(923E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  display.print(F("OK"));
  display.println(" " + loraId);
  display.display();

  // STEP 3 - Initialize WiFi connectivity
  WiFi.begin(ssid, password);
  client.begin(server, net);
  client.onMessage(messageReceived);

  // STEP 4 - Connecting to WiFi and MQTT broker
  connect();

}
///////////////////////////////////////////////
void DAQ(){
  float ph, disoxy, salinity, temperature, nh4;
  if(aquaculture(&ph, &disoxy, &salinity, &temperature, &nh4) == aquacultureError){
    return;
  }
  //String aquadata = (String)ph + "," + (String)disoxy + "," + (String)salinity + "," + (String)temperature + "," + (String)nh4;
  //Serial.println("Aqua-Sensor-Data: "+ aquadata);
  
  DTX = "{\"ID\":";
  DTX += (String)loraId;
  DTX +=  ",\"DO\":";
  DTX += (String)disoxy;
  DTX += ",\"pH\":";
  DTX += String(ph);
  DTX +=  ",\"DD\":";
  DTX += String(salinity);
  DTX +=  ",\"NH4\":";
  DTX += String(nh4);
  DTX +=  ",\"TMP\":";
  DTX += String(TMP);
  DTX += "}";
  Serial.println(DTX);
  LoRa.beginPacket();
  LoRa.print(DTX);
  LoRa.endPacket();
  delayMicroseconds(5000);
}
///////////////////////////////////////////////
void loop() {
  //check WiFi Connection
 WiFiCheckConnect();
 //check MQTT connection
  checkConnect();
  BroadCast();
  delay(500);
}
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
///////////////////////////////////////////////
void messageReceived(String &topic, String &payload) {
//Serial.println("incoming: " + topic + " - " + payload); 
  Serial.println("Topic: " +  topic);
  Serial.println("Payload: " +  payload);
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
  String sensorType = topic;

  if (sensorType == sub2)
  {
    String sensor1  = payload.substring(0,payload.indexOf(","));
    int tSensors = sensor1.toInt();
    if (tSensors == 200 || tSensors == loraId)
    {
      client.publish(sub3, String("ID 1 is Online"));
      DAQ();
      client.publish(sub3, String(DTX));
    }
    if (tSensors == 404)
    { 
      REBOOT = 404;
      client.publish(sub3, String(REBOOT));
      ESP.restart();
    }
  }
}
////////////////////////////////////////////////////////////////////
void checkConnect(){
  client.loop();
  if (!client.connected()) {
    connect();
  }
}
void BroadCast(){
 // publish a message roughly every second.
// if (millis() - lastMillis > 900000L) {
 if (millis() - lastMillis > 5000) {
    lastMillis = millis();
 //Serial.println("time is good");
    DAQ();
    client.publish(sub1, DTX);
 }
}
////////////////////////////////////////////////////////////////////
int aquaculture(float* ph, float* disoxy, float *salinity, float* temperature, float* nh4){
  int valid = 1;
  uint16_t temp;
  unsigned long resptime;
  uint8_t frame[FRAMESIZE_M] = {BYTE_00_M, BYTE_01_M, BYTE_02_M, BYTE_03_M, BYTE_04_M, BYTE_05_M, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float p = 0.00, d = 0.00, s = 0.00, t = 0.00, n = 0.00;
  
  temp = calculateCRC(frame, FRAMESIZE_M - 19); //calculate out crc only from first 6 bytes

  frame[6] = lowByte(temp);
  frame[7] = highByte(temp);

  Serial.println("=========================================================");

//  String request = "Request: ";
//  for(int n=0; n<FRAMESIZE_M - 17; n++) {
//    request += frame[n] < 0x10 ? " 0" : " ";
//    request += String(frame[n], HEX);
//    request.toUpperCase();
//  }
//  Serial.println(request);

  digitalWrite(TXENABLE, HIGH);

  modbus.write(frame, FRAMESIZE_M - 17); //send 8 bytes

  digitalWrite(TXENABLE, LOW);

  resptime = millis();

  while ((modbus.available() < FRAMESIZE_M) && ((millis() - resptime) < MAX_MILLIS_TO_WAIT_M)) {
    delay(1);
  }
  
  if(modbus.available()) {
    
    String response = "Response: ";
    for(int n=0; n<FRAMESIZE_M; n++) {
      frame[n] = modbus.read();
//      response += frame[n] < 0x10 ? " 0" : " ";
//      response += String(frame[n], HEX);
//      response.toUpperCase();
    }

//    Serial.println(response);

     if (frame[0] == BYTE_00_M && frame[1] == BYTE_01_M && frame[2] == BYTE_RESPONSE_M) {
      if ((calculateCRC(frame, FRAMESIZE_M - 2)) == ((frame[24] << 8) | frame[23])) {  //calculate crc from first 7 bytes and compare with received crc (bytes 7 & 8)
        ((uint8_t*)&p)[3] = frame[3];
        ((uint8_t*)&p)[2] = frame[4];
        ((uint8_t*)&p)[1] = frame[5];
        ((uint8_t*)&p)[0] = frame[6];

        ((uint8_t*)&d)[3] = frame[7];
        ((uint8_t*)&d)[2] = frame[8];
        ((uint8_t*)&d)[1] = frame[9];
        ((uint8_t*)&d)[0] = frame[10];

        ((uint8_t*)&s)[3] = frame[11];
        ((uint8_t*)&s)[2] = frame[12];
        ((uint8_t*)&s)[1] = frame[13];
        ((uint8_t*)&s)[0] = frame[14];

        ((uint8_t*)&t)[3] = frame[15];
        ((uint8_t*)&t)[2] = frame[16];
        ((uint8_t*)&t)[1] = frame[17];
        ((uint8_t*)&t)[0] = frame[18];

        ((uint8_t*)&n)[3] = frame[19];
        ((uint8_t*)&n)[2] = frame[20];
        ((uint8_t*)&n)[1] = frame[21];
        ((uint8_t*)&n)[0] = frame[22];

        *ph = p, *disoxy = d, *salinity = s, *temperature = t, *nh4 = n;
      }
      else{
        valid = 0;
      }
    }
    else{
      valid = 0;
    } 
  }
  else{
    valid = 0;
  }

  return valid;
}
//////////////////////////////////////////////////////////
uint16_t calculateCRC(uint8_t *array, uint8_t num) {
  uint16_t temp, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0; i < num; i++) {
    temp = temp ^ array[i];
    for (uint8_t j = 8; j; j--) {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  return temp;
}
////////////////////////////////////////////////////////////////////
boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}
////////////////////////////////////////////////////////////////////
