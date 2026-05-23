#pragma once

#include "driver/pcnt.h"

// ===================== PINS =====================
#define ENC1_A 4
#define ENC1_B 5

#define ENC2_A 20
#define ENC2_B 21

#define PULSES_PER_REV 200

// ===================== VARIABLES =====================
int16_t prev_count1 = 0;
int16_t prev_count2 = 0;

long total_count1 = 0;
long total_count2 = 0;

// ===================== CONFIG PCNT =====================
// Configures both channels for proper quadrature (2x resolution + direction)
void setup_pcnt(int unit, int pinA, int pinB) {

  // --- Channel 0: count on A edges, direction from B ---
  pcnt_config_t cfg0 = {};
  cfg0.pulse_gpio_num = pinA;
  cfg0.ctrl_gpio_num  = pinB;
  cfg0.channel        = PCNT_CHANNEL_0;
  cfg0.unit           = (pcnt_unit_t)unit;
  cfg0.pos_mode       = PCNT_COUNT_INC;   // rising  edge on A → +1
  cfg0.neg_mode       = PCNT_COUNT_DIS;   // falling edge on A → ignore (channel 1 handles it)
  cfg0.lctrl_mode     = PCNT_MODE_REVERSE; // B low  → reverse count direction
  cfg0.hctrl_mode     = PCNT_MODE_KEEP;    // B high → keep count direction
  cfg0.counter_h_lim  = 30000;
  cfg0.counter_l_lim  = -30000;
  pcnt_unit_config(&cfg0);

  // --- Channel 1: count on B edges, direction from A ---
  pcnt_config_t cfg1 = {};
  cfg1.pulse_gpio_num = pinB;
  cfg1.ctrl_gpio_num  = pinA;
  cfg1.channel        = PCNT_CHANNEL_1;
  cfg1.unit           = (pcnt_unit_t)unit;
  cfg1.pos_mode       = PCNT_COUNT_INC;
  cfg1.neg_mode       = PCNT_COUNT_DIS;
  cfg1.lctrl_mode     = PCNT_MODE_KEEP;    // A low  → keep (opposite of ch0)
  cfg1.hctrl_mode     = PCNT_MODE_REVERSE; // A high → reverse
  cfg1.counter_h_lim  = 30000;
  cfg1.counter_l_lim  = -30000;
  pcnt_unit_config(&cfg1);

  // Glitch filter: ~1 µs at 80 MHz APB clock
  pcnt_set_filter_value((pcnt_unit_t)unit, 100);
  pcnt_filter_enable((pcnt_unit_t)unit);

  pcnt_counter_pause((pcnt_unit_t)unit);
  pcnt_counter_clear((pcnt_unit_t)unit);
  pcnt_counter_resume((pcnt_unit_t)unit);
}

// ===================== SETUP =====================
void encoders_setup() {
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

  setup_pcnt(PCNT_UNIT_0, ENC1_A, ENC1_B);
  setup_pcnt(PCNT_UNIT_1, ENC2_A, ENC2_B);

  Serial.println("PCNT quadrature ready...");
}

// ===================== UPDATE (call every loop tick) =====================
void encoders() {
  int16_t count1, count2;

  pcnt_get_counter_value(PCNT_UNIT_0, &count1);
  pcnt_get_counter_value(PCNT_UNIT_1, &count2);

  // Delta since last call
  int16_t delta1 = count1 - prev_count1;
  int16_t delta2 = count2 - prev_count2;

  // Accumulate into long totals (survives PCNT hardware overflow)
  total_count1 += delta1;
  total_count2 += delta2;

  prev_count1 = count1;
  prev_count2 = count2;
}
