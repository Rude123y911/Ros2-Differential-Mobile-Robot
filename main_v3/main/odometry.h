#pragma once

#include <math.h>  // provides M_PI, cos(), sin()

#define WHEEL_DIAMETER 0.065  // meters (6.5 cm)
#define WHEEL_BASE     0.22   // meters, distance between wheels

// ===================== ROBOT POSE =====================
float x     = 0.0;
float y     = 0.0;
float theta = 0.0;  // radians, CCW positive

// ===================== INTERNAL STATE =====================
long prev_count1_odom = 0;
long prev_count2_odom = 0;

// ===================== UPDATE =====================
void update_odometry() {

  // Delta pulses since last call
  long delta1 = total_count1 - prev_count1_odom;  // encoder 1 → left  wheel
  long delta2 = total_count2 - prev_count2_odom;  // encoder 2 → right wheel

  prev_count1_odom = total_count1;
  prev_count2_odom = total_count2;

  // Pulses → metres
  const float dist_per_pulse = (M_PI * WHEEL_DIAMETER) / PULSES_PER_REV;

  float dL = delta2 * dist_per_pulse;  // right wheel distance (encoder 2)
  float dR = delta1 * dist_per_pulse;  // left  wheel distance (encoder 1)


  // Differential-drive kinematics
  float dS     = (dR + dL) / 2.0f;               // forward distance
  float dTheta = (dR - dL) / WHEEL_BASE;          // heading change

  // Integrate pose using midpoint heading for better accuracy
  float theta_mid = theta + dTheta / 2.0f;
  x     += dS * cosf(theta_mid);
  y     += dS * sinf(theta_mid);
  theta += dTheta;
}
