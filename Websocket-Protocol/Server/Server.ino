
#include <WiFi.h>
#include <WebSocketsServer.h>

// Replace with your network credentials
const char *ssid = "SEA-IC";
const char *password = "seaic2022";

WebSocketsServer webSocket = WebSocketsServer(8888);

String JS;
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        // Send message to client
        webSocket.sendTXT(num, "Connected");
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      // Send message to client
      webSocket.sendTXT(num, JS);
      break;
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());   //You can get IP address assigned to ESP


  Serial.println("WebSocket server started");
}

void loop() {
  // int JSTICK = random(1000,2000);
  int JSTICK = analogRead(A0);
  webSocket.loop();
        JS = String(JSTICK);
  webSocket.broadcastTXT(JS);
//  delay(1000);
}

