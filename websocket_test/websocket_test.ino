#include <WiFiClient.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
// #include <Adafruit_NeoPixel.h>
#define LED_RED 2   //赤LED PIN番号
#define LED_GREEN 3 //緑LED PIN番号
#define LED_BLUE 4  //青LED PIN番号

const char* ssid = "PicoW";
const char* password = "12345678";

const IPAddress ip(192, 168, 40, 1);
const IPAddress subnet(255, 255, 255, 0);

int red   = 0;
int blue  = 0;
int green = 0;

WebSocketsServer webSocket = WebSocketsServer(8081);

void setup() {
  Serial.begin(9600);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  // サーバーのIPアドレスを表示
  Serial.println("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // WebSocketサーバーを開始
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("WebSocket server started.");
}

void loop() {
  webSocket.loop();
  analogWrite(LED_RED, red);
  analogWrite(LED_GREEN, green);
  analogWrite(LED_BLUE, blue);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      break;
    }
    case WStype_TEXT:
      String text = String((char *)payload).substring(0, length);
      Serial.printf("[%u] Text: %s\n", num, text.c_str()); 
      if (text == "red on")         red   = 255;
      else if (text == "green on")  green = 255;
      else if (text == "blue on")   blue  = 255;
      else if (text == "red off")   red   = 0;
      else if (text == "green off") green = 0;
      else if (text == "blue off")  blue  = 0;
      break;
  }
}
