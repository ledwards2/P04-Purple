#include <Wire.h> // arduino I2C library
#include <Adafruit_MLX90393.h> // Melexis MLX90393 Library by Kevin Townsend, for Adafruit Industries. Available at: https://github.com/adafruit/Adafruit_MLX90393_Library

#include <HCSR04.h> // HCSR04 Distance Sensor Library by Martin Sosic, Available at: https://github.com/Martinsos/arduino-lib-hc-sr04 
#include <Servo.h> /*Servo library, copyright Michael Margolis (2009), licenced under GNU Lesser General Public Licence v2.1 
                    Available At: https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h */
                    
Adafruit_MLX90393 mag = Adafruit_MLX90393();
// Created magnetometer object

// pin setup: HC-SR04 Distance Sensor
int TRIG_LEFT = 2;
int ECHO_LEFT = 5;
int TRIG_RIGHT = 4;
int ECHO_RIGHT = 6;

UltraSonicDistanceSensor LeftDistance(TRIG_LEFT, ECHO_LEFT);
UltraSonicDistanceSensor RightDistance(TRIG_RIGHT, ECHO_RIGHT);
// Created sensor objects for left and right distance sensors
Servo mark_servo; 
// created object for marking servo

// Motor Driver Setup : A Channel (LEFT)
int DIRECTION_A = 12;
int BRAKE_A = 9;
int PWM_A = 3;
// Motor Driver Setup : B Channel (RIGHT)
int DIRECTION_B = 13;
int BRAKE_B = 8;
int PWM_B = 11;

// DRIVE SPEED PWM in range (0, 255)
int DRIVE_SPEED = 91;


// marking system config:
int MARKER_PWM = 10;

//Threshold Values
int DIST_THRESH = 5; //cm
int abs_mag_thresh = 30000; // microTesla

int MAX_TURN_TIME = 2000; // miliseconds

int MIN_MARK_ANGLE = 0; // angle [0, 180]
int MAX_MARK_ANGLE = 180; // angle [0, 180] > MIN_MARK_ANGLE

float ReadDist(UltraSonicDistanceSensor *sensor) {
  /* Function to return the distance measured by a HC-SR04 ultrasonic distance sensor
   *  Returns: 
   *  float: distance(cm)
   */
  return sensor->measureDistanceCm();
}

float MagAbsolute(Adafruit_MLX90393 sensor) {
  /* function to return the magnitude of a magnetic field
   *  Parameters: 
   *  Adafruit_MLX90393 *sensor: Adafruit MLX90393 Sensor Object
   *  Returns:
   *  float: the absolute value of magnetic strength, in the same unit as the sensor
   */
  float x, y, z;
  if (sensor.readData(&x, &y, &z)) {
    return sqrt(sq(x) + sq(y) + sq(z));
  }
}

float ReadMag(Adafruit_MLX90393 sensor, char axis) {
  /* Function to return magnetic field strength across an axis.
   *  Using Adafruit MLX90393 Library by Kevin Townsend. 
   *  Parameter: axis(char): the axis to return field strength across
   *  Returns: float: the field strength (in microTesla) across the input axis
   */
  float x, y, z;
  if(sensor.readData(&x, &y, &z)) {
    if(axis == 'x') {
      return x;
    }
    if(axis='y') {
      return y;
    }
    if(axis='z') {
      return z;
    }
  }
  
  
}

void Stop() {
  /* function to make robot stop */
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}

void ActivateMarker() {
  /* Function to activate robot marking system.
   *  Uses Arduino Servo library to control one SG09 Servo Motor,
   *  which moves smoothly from MIN_MARK_ANGLE to MAX_MARK_ANGLE
   */
   int angle = MIN_MARK_ANGLE;
   for (angle = MIN_MARK_ANGLE; angle <= MAX_MARK_ANGLE; angle += 1) {
    digitalWrite(LED_BUILTIN, HIGH);
    mark_servo.write(angle);
    delay(5);
    digitalWrite(LED_BUILTIN, LOW);
    delay(5);
   }
   for (angle = MAX_MARK_ANGLE; angle >= MIN_MARK_ANGLE; angle -= 1) {
    mark_servo.write(angle);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(5);
    digitalWrite(LED_BUILTIN, LOW);
    delay(5);
   }
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

  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIRECTION_A, LOW);
  digitalWrite(DIRECTION_B, LOW);
  // begin connection with magnetometer
  mag.begin();
  // bein connection with servo pin and Servo library
  mark_servo.attach(MARKER_PWM);
  // initialize led pin for secondary marking system
  pinMode(LED_BUILTIN, OUTPUT);
}



void loop() {
  // put your main code here, to run repeatedly:
  // poll all sensors first:
  int left_distance = LeftDistance.measureDistanceCm();
  int right_distance = RightDistance.measureDistanceCm();
  float mag_strength = MagAbsolute(mag);
  // now check conditions
  if(left_distance > DIST_THRESH and right_distance > DIST_THRESH and mag_strength < abs_mag_thresh) {
    // if FRONT CLEAR AND NO MINES
    delay(100); // delay to keep polling rate of sensors to less than 10Hz while no mines and no obstacles
    analogWrite(PWM_A, DRIVE_SPEED);
    analogWrite(PWM_B, DRIVE_SPEED);
  }

  if(left_distance <= DIST_THRESH and right_distance <= DIST_THRESH) {
    // IF FRONT BLOCKED
    Stop();
    // TURN ON SPOT
    int turn_direction = random(0, 2); // generate random turn direction.
    int turn_time = random(100, MAX_TURN_TIME);
    if(turn_direction < 1) {
      // turn left:
      TightLeft(turn_time);
    }
    if(turn_direction >= 1) {
      TightRight(turn_time);
    }
  }
  else if(left_distance <= DIST_THRESH) {
    // IF LEFT BLOCKED
    Stop();
    // TURN RIGHT
    int turn_time = random(100, MAX_TURN_TIME);
    TightRight(turn_time);
  }
  else if(right_distance <= DIST_THRESH) {
    // IF RIGHT BLOCKED
    Stop();
    // TURN LEFT:
    int turn_time = random(100, MAX_TURN_TIME);
    TightLeft(turn_time);
  }
  if(mag_strength >= abs_mag_thresh) {
    // IF MINE PRESENT
    Stop();
    ActivateMarker();
    Reverse(500); // reverse backwards from mine
    int turn_direction = random(0, 2);
    int turn_time = random(100, MAX_TURN_TIME);
    // turn random direction for random time
    if (turn_direction < 1) {//turn left
      TightLeft(turn_time);
    }
    if (turn_direction >= 1) { // turn right
      TightRight(turn_time);
    }
  }
}
