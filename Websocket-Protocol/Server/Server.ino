
#include <ArduinoWebsockets.h>
#include <WiFi.h>

const char* ssid = "ssid"; //Enter SSID
const char* password = "password"; //Enter Password

using namespace websockets;

WebsocketsServer server;

void setup() {
  Serial.begin(115200);
  // Connect to wifi
  WiFi.begin(ssid, password);

  // Wait some time to connect to wifi
  while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());   //You can get IP address assigned to ESP

  server.listen(8888);
  Serial.print("Is server live? ");
  Serial.println(server.available());
}

void loop() {
  if (server.available()) {
    WebsocketsClient client = server.accept();
    while(client.connected()){
      String msg = client.readString();
      int JSTICK = analogRead(A0);
      // log
      Serial.print("Got Message: ");
      Serial.println(msg);
      client.send(String(JSTICK));
    }
    client.connect();
  }
  
  delay(200);
}

