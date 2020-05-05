#include <Servo.h>
int MARKER_PWM = 10;
int MIN_ANGLE = 0;
int MAX_ANGLE = 180;
Servo mark_servo; // servo object mark_servo created

void MarkMine() {
  /* function to activate marking system for robot
   *  Uses a FOR loop to have servo move smoothly from 
   *  MIN_ANGLE to MAX_ANGLE, and back to MIN_ANGLE.
   */
  int angle = MIN_ANGLE;
  for (angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += 1) {
    mark_servo.write(angle);
    delay(10);
  }
  for (angle = MAX_ANGLE; angle >= MIN_ANGLE; angle = angle - 1) {
    mark_servo.write(angle);
    delay(10);
  }
}
void setup() {
  // put your setup code here, to run once:
  pinMode(10, OUTPUT);
  mark_servo.attach(MARKER_PWM); // mark_servo attached to pwm pin MARKER_PWM
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000); // wait 10 seconds to prevent accidental marker activation
  MarkMine();
}
