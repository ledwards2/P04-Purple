/* Function to test movement for robot */



// pin numbers for motorshield:

/* A CHANNEL: LEFT SIDE
 * B CHANNEL: RIGHT SIDE
 */
int DIRECTION_A = 12;
int BRAKE_A = 9;
int PWM_A = 3;


int DIRECTION_B = 13;
int BRAKE_B = 8;
int PWM_B = 11;

int DRIVE_SPEED = 91;

void StartForward() {
  /* Function to have robot start driving forward */
  analogWrite(PWM_A, DRIVE_SPEED) ;
  analogWrite(PWM_B, DRIVE_SPEED);
  // both motor speed pwm pins are written to the desired drive speed
}

void Stop() {
  /* Function to have robot stop */
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  // both motor channels are written to stop (pwm 0)
}

void TurnLeft(int turn_time) {
  /* Function to make robot perform gentle left turn for turn_time
   *  Parameter: int turn_time: time for turn to be perfomed (miliseconds)
   *  Action: robot performs gentle left turn; where right motors are set
   *     to drive_speed and left motors are stationary. 
   */
  Stop();
  analogWrite(PWM_B, DRIVE_SPEED);
  delay(turn_time); // right side motors are driven while left is stopped
  Stop();
}

void TightLeft(int turn_time) {
  /* function to make robot perform tight turns in the left direction (anticlockwise)
   *  Parameter: turn_time: the time for the turn to be performed (miliseconds)
   *  Action: robot turns left for turn_time
    */
  Stop();
  // reverse left motor
  digitalWrite(DIRECTION_A, HIGH);
  analogWrite(PWM_A, DRIVE_SPEED);
  analogWrite(PWM_B, DRIVE_SPEED);
  delay(turn_time);
  Stop();
  // change back to normal motor direction
  digitalWrite(DIRECTION_A, LOW);

}
void TurnRight(int turn_time) {
  Stop();
  analogWrite(PWM_A, DRIVE_SPEED);
  delay(turn_time);
  Stop();
}

void TightRight(int turn_time) {
  /* Function for robot to perform a tight turn in the right direction (clockwise)
   *  Parameter: int turn_time: the time for the turn to be performed for (miliseconds)
   *  Action: robot turns clockwise for turn_time
   */
  Stop();
  digitalWrite(DIRECTION_B, HIGH);
  analogWrite(PWM_B, DRIVE_SPEED);
  analogWrite(PWM_A, DRIVE_SPEED);
  delay(turn_time);
  Stop();
  digitalWrite(DIRECTION_B, LOW);
}

void Reverse(int drive_time) {
  /* Function to make robot drive backwards for int drive_time
   *  Parameter: int drive_time: the time to drive backwards for, in miliseconds
   *  Action: robot drives backwards for drive_time in miliseconds
   */
  Stop();
  digitalWrite(DIRECTION_B, HIGH);
  digitalWrite(DIRECTION_A, HIGH);
  analogWrite(PWM_A, DRIVE_SPEED);
  analogWrite(PWM_B, DRIVE_SPEED);
  delay(drive_time);
  Stop();
  digitalWrite(DIRECTION_B, LOW);
  digitalWrite(DIRECTION_A, LOW);
}

void setup() {
  // put your setup code here, to run once:
  // pin setup for motorshield
  pinMode(DIRECTION_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  pinMode(PWM_A, OUTPUT) ; 

  pinMode(DIRECTION_B, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  // disable braking on each channel
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW); 
  // set up direction pins
  // Assuming wheels turn forward with direction pins LOW:
  digitalWrite(DIRECTION_A, LOW);
  digitalWrite(DIRECTION_B, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  // move robot forward for 1 second:
  StartForward();
  delay(1000);
  Stop();
  // gentle left turn for 1 second, then wait 0.5 seconds:
  TurnLeft(1000);
  delay(500);
  // gentle right turn for 1 second, then wait 0.5 seconds:
  TurnRight(1000);
  delay(500);
  // tight left turn for 1 second, then wait 0.5 seconds:
  TightLeft(1000);
  delay(500);
  // tight right turn for 1 second, then wait 1 second:
  TightRight(1000);
  delay(1000);
}
