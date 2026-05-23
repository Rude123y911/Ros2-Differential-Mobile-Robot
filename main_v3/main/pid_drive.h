#pragma once

// ===================== PID STRAIGHT-LINE CONTROLLER =====================
// Keeps both wheels in sync during forward/backward moves.
// Error = cumulative encoder difference (left - right pulses since move start).
// A positive error means left is ahead → slow left, speed up right.

struct PIDState {
  float kP = 1.2f;   // ← ajusta este primero
  float kI = 0.05f;  // ← pequeño, evita windup
  float kD = 0.8f;   // ← amortigua oscilaciones

  float integral   = 0.0f;
  float prev_error = 0.0f;

  int   max_correction = 40;  // límite de corrección en PWM (0-255)

  void reset() {
    integral   = 0.0f;
    prev_error = 0.0f;
  }

  // Devuelve cuánto restar/sumar al PWM base
  // error positivo → rueda izquierda (enc1) va más rápido
  int compute(float error) {
    integral  += error;
    // Anti-windup: limita el integral
    integral   = constrain(integral, -200.0f, 200.0f);

    float derivative = error - prev_error;
    prev_error = error;

    float output = kP * error + kI * integral + kD * derivative;
    return (int)constrain(output, -max_correction, max_correction);
  }
};

static PIDState straight_pid;