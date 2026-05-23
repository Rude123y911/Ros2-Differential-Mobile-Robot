#include "encoders_setup.h"
#include "odometry.h"
#include "motor_setup.h"
#include "pid_drive.h"

void setup() {
  Serial.begin(115200);
  delay(2000);
  encoders_setup();
  motor_setup();
  delay(2000);
  Serial.println("Ready. Commands: w=forward, s=backward, a=left, d=right");
}

void loop() {
  update_movement();  // non-blocking: checks encoder progress every tick

  if (!is_moving() && Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if      (cmd == "w") begin_move_forward (2, 150);
    else if (cmd == "s") begin_move_backward(2, 150);
    else if (cmd == "a") begin_move_left    (1, 150);
    else if (cmd == "d") begin_move_right   (1, 150);
    else Serial.println("Unknown command");
  }
}
