#include <WiFiClient.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pwm.pio.h"
#include "sensor.pio.h"
#include <string.h>

#ifndef PICO_DEFAULT_LED_PIN
#error pio/pwm example requires a board with a regular LED
#endif

#define LED_PIN_0 0
#define LED_PIN_1 1
#define LED_PIN_2 2
#define LED_PIN_3 3

#define SENSOR_PIN_0 12
#define SENSOR_PIN_1 13
#define SENSOR_PIN_2 14
#define SENSOR_PIN_3 15
#define SENSOR_PIN_4 8
#define SENSOR_PIN_5 9

#define TEST_PIN_0 16
#define TEST_PIN_1 17
#define TEST_PIN_2 18
#define TEST_PIN_3 19

const char* ssid = "PicoW";
const char* password = "12345678";

const IPAddress ip(192, 168, 40, 1);
const IPAddress subnet(255, 255, 255, 0);

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

unsigned int readLatestDataFromFifo(PIO pio, uint sm) {
  unsigned int latestData = 0;
  while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
      latestData = pio_sm_get(pio, sm);
  }
  return latestData;
}

PIO pio_0 = pio0;
// PIO pio_1 = pio1;

uint pio_0_offset;
uint pio_1_offset;
int pio_0_sm_0 = 0;
int pio_0_sm_1 = 1;
int pio_0_sm_2 = 2;
int pio_0_sm_3 = 3;
int pio_1_sm_0 = 0;

unsigned int leftMotorAccelForward   = 0;
unsigned int leftMotorAccelBackward  = 0;
unsigned int rightMotorAccelForward  = 0;
unsigned int rightMotorAccelBackward = 0;
unsigned int sensorReg = 0;
unsigned int oldSensorReg = 0;

int dma_chan_0;
int dma_chan_1;

void setup() {

  // GPIOピンを出力として初期化
  gpio_init(TEST_PIN_0);
  gpio_init(TEST_PIN_1);
  gpio_init(TEST_PIN_2);
  gpio_init(TEST_PIN_3);

  // GPIOピンを出力モードに設定
  gpio_set_dir(TEST_PIN_0, GPIO_OUT);
  gpio_set_dir(TEST_PIN_1, GPIO_OUT);
  gpio_set_dir(TEST_PIN_2, GPIO_OUT);
  gpio_set_dir(TEST_PIN_3, GPIO_OUT);

  // GPIOピンに高電圧を出力してONにする
  gpio_put(TEST_PIN_0, 1);
  gpio_put(TEST_PIN_1, 0);
  gpio_put(TEST_PIN_2, 1);
  gpio_put(TEST_PIN_3, 0);

  // PIOの初期化
  pio_0_offset = pio_add_program(pio_0, &pwm_program);
  pwm_program_init(pio_0, pio_0_sm_0, pio_0_offset, LED_PIN_0);
  pwm_program_init(pio_0, pio_0_sm_1, pio_0_offset, LED_PIN_1);
  pwm_program_init(pio_0, pio_0_sm_2, pio_0_offset, LED_PIN_2);
  pwm_program_init(pio_0, pio_0_sm_3, pio_0_offset, LED_PIN_3);
  pio_pwm_set_period(pio_0, pio_0_sm_0, (1u << 16) - 1);
  pio_pwm_set_period(pio_0, pio_0_sm_1, (1u << 16) - 1);
  pio_pwm_set_period(pio_0, pio_0_sm_2, (1u << 16) - 1);
  pio_pwm_set_period(pio_0, pio_0_sm_3, (1u << 16) - 1);
  Serial.printf("Loaded program at %d\n", pio_0_offset);

  // pio_1_offset    = pio_add_program(pio_1, &read_sensors_program);
  // pio_sm_config c = pio_get_default_sm_config();
  // sm_config_set_in_pins(&c, SENSOR_PIN_0);
  // sm_config_set_sideset_pins(&c, 0);
  // pio_sm_init(pio_1, pio_1_sm_0, pio_1_offset, &c);
  // pio_sm_set_enabled(pio_1, pio_1_sm_0, true);
  // Serial.printf("Loaded program at %d\n", pio_1_offset);

  // dma_chan_0 = dma_claim_unused_channel(true);
  // dma_chan_1 = dma_claim_unused_channel(true);
  // dma_channel_config dma_c_0 = dma_channel_get_default_config(dma_chan_0);
  // dma_channel_config dma_c_1 = dma_channel_get_default_config(dma_chan_1);
  // channel_config_set_transfer_data_size(&dma_c_0, DMA_SIZE_32);
  // channel_config_set_transfer_data_size(&dma_c_1, DMA_SIZE_32);
  // channel_config_set_read_increment(&dma_c_0, false);
  // channel_config_set_read_increment(&dma_c_1, false);
  // channel_config_set_write_increment(&dma_c_0, false);
  // channel_config_set_write_increment(&dma_c_1, false);
  // channel_config_set_dreq(&dma_c_0, pio_get_dreq(pio_1, pio_1_sm_0, false));
  // channel_config_set_dreq(&dma_c_1, pio_get_dreq(pio_1, pio_1_sm_0, false));
  // channel_config_set_chain_to(&dma_c_0, dma_chan_1);
  // channel_config_set_chain_to(&dma_c_1, dma_chan_0);
  // dma_channel_configure(dma_chan_0, &dma_c_0, &sensorReg, &pio_1->rxf[pio_1_sm_0], 1, true);
  // dma_channel_configure(dma_chan_1, &dma_c_1, &sensorReg, &pio_1->rxf[pio_1_sm_0], 1, true);

  sleep_ms(1000);



  // Serial
  Serial.begin(115200);

  // websocket
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);    

  Serial.println("WebSocket server started.");
}

void loop1() {
  pio_pwm_set_level(pio_0, pio_0_sm_0, leftMotorAccelForward   * leftMotorAccelForward);
  pio_pwm_set_level(pio_0, pio_0_sm_1, rightMotorAccelForward  * rightMotorAccelForward);
  pio_pwm_set_level(pio_0, pio_0_sm_2, leftMotorAccelBackward  * leftMotorAccelBackward);
  pio_pwm_set_level(pio_0, pio_0_sm_3, rightMotorAccelBackward * rightMotorAccelBackward);
}

void loop() {
  webSocket.loop();
}


void decodeAccel(String str) {
  String hex_0 = str.substring(6, 8);
  String hex_1 = str.substring(8, 10);
  String hex_2 = str.substring(10, 12);
  String hex_3 = str.substring(12, 14);
  leftMotorAccelForward   = strtoul(hex_0.c_str(), NULL, 16);
  rightMotorAccelForward  = strtoul(hex_1.c_str(), NULL, 16);
  leftMotorAccelBackward  = strtoul(hex_2.c_str(), NULL, 16);
  rightMotorAccelBackward = strtoul(hex_3.c_str(), NULL, 16);
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
      if(text.startsWith("accel ")) {
        decodeAccel(text);
      }
      break;
  }
}
