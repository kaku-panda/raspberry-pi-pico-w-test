#include <WiFiClient.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pwm.pio.h"
#include <string.h>

#ifndef PICO_DEFAULT_LED_PIN
#error pio/pwm example requires a board with a regular LED
#endif

#define LED_PIN 0

const char* ssid = "PicoW";
const char* password = "12345678";

const IPAddress ip(192, 168, 40, 1);
const IPAddress subnet(255, 255, 255, 0);

int red   = 0;
int blue  = 0;
int green = 0;

WebSocketsServer webSocket = WebSocketsServer(8081);

// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_put_blocking(pio, sm, period);
  pio_sm_exec(pio, sm, pio_encode_pull(false, false));
  pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
  pio_sm_set_enabled(pio, sm, true);
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
  pio_sm_put_blocking(pio, sm, level);
}

PIO pio = pio0;
uint offset;
int sm;
int level = 0;


void setup() {
  // todo get free sm
  sm = 0;
  offset = pio_add_program(pio, &pwm_program);
  // gpio_init(LED_PIN);
  // gpio_set_dir(LED_PIN, GPIO_OUT);
  pwm_program_init(pio, sm, offset, LED_PIN);
  pio_pwm_set_period(pio, sm, (1u << 16) - 1);
  sleep_ms(300);  // has to sleep for more than 300ms until Serial print begins to works
  Serial.begin(115200);
  Serial.printf("Loaded program at %d\n", offset);

  // websocket
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
  pio_pwm_set_level(pio, sm, level * level);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      webSocket.sendTXT(num, "Welcome to the WebSocket server!");
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
      else if(text.startsWith("accel ")) {
        int value = text.substring(4).toInt(); // "accel "の後ろの数値を取得
        level = value & 0xFF;                  // 下位8ビットを取得
      }
      break;
  }
}
