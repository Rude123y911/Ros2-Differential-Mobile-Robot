#pragma once
#include "pid_drive.h"
// ===================== PINS =====================
#define MOT_A1_PIN 1
#define MOT_A2_PIN 2
#define MOT_B1_PIN 42
#define MOT_B2_PIN 41
#define EEP_PIN    15

// ===================== PWM CONFIG =====================
const int freq       = 1000;
const int resolution = 8;

// ===================== STALL TIMEOUT =====================
#define MOVE_TIMEOUT_MS 5000  // stop automatically if target not reached in 5 s

// ===================== NON-BLOCKING STATE MACHINE =====================
enum MoveState { IDLE, MOVING_FORWARD, MOVING_BACKWARD, MOVING_LEFT, MOVING_RIGHT };

static MoveState      current_move       = IDLE;
static long           move_target_pulses = 0;
static long           move_start_count1   = 0;
static long           move_start_count2   = 0;
static unsigned long  move_start_ms      = 0;
static int            base_pwm = 0;

// ===================== LOW-LEVEL HELPERS =====================
void set_motor_pwm(int pwm, int pin1, int pin2) {
  if (pwm < 0) {
    ledcWrite(pin1, -pwm);
    ledcWrite(pin2, 0);
  } else {
    ledcWrite(pin1, 0);
    ledcWrite(pin2, pwm);
  }
}

void set_motor_currents(int pwm_A, int pwm_B) {
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);
}

// ===================== SETUP =====================
void motor_setup() {
  pinMode(EEP_PIN, OUTPUT);
  digitalWrite(EEP_PIN, HIGH);

  ledcAttach(MOT_A1_PIN, freq, resolution);
  ledcAttach(MOT_A2_PIN, freq, resolution);
  ledcAttach(MOT_B1_PIN, freq, resolution);
  ledcAttach(MOT_B2_PIN, freq, resolution);

  set_motor_currents(0, 0);
}

// ===================== STOP =====================
void stop_motors() {
  set_motor_currents(0, 0);
  current_move = IDLE;
}

// ===================== BEGIN MOVES (non-blocking) =====================
void begin_move_forward(float turns, int pwm_speed) {
  base_pwm = pwm_speed;
  move_target_pulses = (long)(turns * PULSES_PER_REV);
  move_start_count1   = total_count1; 
  move_start_count2   = total_count2;   // track either wheel (both move equally)
  move_start_ms      = millis();
  current_move       = MOVING_FORWARD;
  straight_pid.reset();
  set_motor_currents(pwm_speed, pwm_speed);
}

void begin_move_backward(float turns, int pwm_speed) {
  base_pwm = pwm_speed;
  move_target_pulses = (long)(turns * PULSES_PER_REV);
  move_start_count1   = total_count1; 
  move_start_count2   = total_count2;
  move_start_ms      = millis();
  current_move       = MOVING_BACKWARD;
  straight_pid.reset();
  set_motor_currents(-pwm_speed, -pwm_speed);
}

void begin_move_right(float turns, int pwm_speed) {
  // Only motor A runs → track encoder 1 (FIX: was wrongly watching encoder 2)
  move_target_pulses = (long)(turns * PULSES_PER_REV);
  move_start_count2   = total_count2;
  move_start_ms      = millis();
  current_move       = MOVING_RIGHT;
  set_motor_currents(pwm_speed, 0);
}

void begin_move_left(float turns, int pwm_speed) {
  // Only motor B runs → track encoder 2
  move_target_pulses = (long)(turns * PULSES_PER_REV);
  move_start_count1   = total_count1;
  move_start_ms      = millis();
  current_move       = MOVING_LEFT;
  set_motor_currents(0, pwm_speed);
}

// ===================== STATUS =====================
bool is_moving() { return current_move != IDLE; }

// ===================== UPDATE (call every loop tick) =====================
void update_movement() {
  if (current_move == IDLE) return;

  // Update sensors first
  encoders();
  update_odometry();

  // Stall / timeout guard
  if (millis() - move_start_ms > MOVE_TIMEOUT_MS) {
    stop_motors();
    Serial.println("WARN: move timeout — possible stall");
    return;
  }

  long traveled = 0;

  switch (current_move) {

    case MOVING_FORWARD: {
      long enc1_delta = total_count1 - move_start_count1;
      long enc2_delta = total_count2 - move_start_count2;
      traveled = (enc1_delta + enc2_delta) / 2;

      // PID: error = enc1 - enc2 (positivo → izq va más rápido → frenar izq)
      int correction = straight_pid.compute((float)(enc2_delta - enc1_delta));
      set_motor_currents(base_pwm - correction, base_pwm + correction);

      if (traveled >= move_target_pulses) {
        stop_motors();
        Serial.println("OK: forward done");
      }
        break;
    }

    case MOVING_BACKWARD: {
      long enc1_delta = total_count1 - move_start_count1;
      long enc2_delta = total_count2 - move_start_count2;
      traveled = (enc1_delta + enc2_delta) / 2;

      int correction = straight_pid.compute((float)(enc1_delta - enc2_delta));
      // En reversa los signos se invierten
      set_motor_currents(-(base_pwm - correction), -(base_pwm + correction));

      if (traveled <= -move_target_pulses) {
        stop_motors();
        Serial.println("OK: backward done");
      }
      break;
    }

    case MOVING_RIGHT:
      traveled = total_count2 - move_start_count2;  // encoder 1 = motor A
      if (traveled >= move_target_pulses) {
        stop_motors();
        Serial.println("OK: right done");
      }
      break;

    case MOVING_LEFT:
      traveled = total_count1 - move_start_count1;  // encoder 2 = motor B
      if (traveled >= move_target_pulses) {
        stop_motors();
        Serial.println("OK: left done");
      }
      break;

    default:
      break;
  }
}

// ===================== BLOCKING HELPER (kept for calibration use) =====================
void spin_and_wait(int pwm_A, int pwm_B, int duration) {
  set_motor_currents(pwm_A, pwm_B);
  delay(duration);
}
