/**
   Copyright (c) 2020 Raspberry Pi (Trading) Ltd.

   SPDX-License-Identifier: BSD-3-Clause

   Ported to Arduino IDE
   By vabenecosi
*/

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pwm.pio.h"

#ifndef PICO_DEFAULT_LED_PIN
#error pio/pwm example requires a board with a regular LED
#endif

#define LED_PIN 0

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

}

void loop() {

  Serial.printf("Level = %d\n", level);
  pio_pwm_set_level(pio, sm, level * level);
  level = (level + 1) % 256;
  // sleep_ms(10);
}