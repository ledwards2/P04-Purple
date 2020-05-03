#include <Wire.h>
#include <Adafruit_MLX90393.h> // Melexis MLX90393 Library by Kevin Townsend, for Adafruit Industries. Available at: https://github.com/adafruit/Adafruit_MLX90393_Library

#include <HCSR04.h> // HCSR04 Distance Sensor Library by Martin Sosic, Available at: https://github.com/Martinsos/arduino-lib-hc-sr04 

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



// Motor Driver Setup
int DIRECTION_A = 12;
int BRAKE_A = 9;
int PWM_A = 3;


int DIRECTION_B = 13;
int BRAKE_B = 8;
int PWM_B = 11;

int DRIVE_SPEED = 91;


//Threshold Values
int DIST_THRESH = 5; //cm

int abs_mag_thresh = 20; // UNITS????

int MAX_TURN_TIME = 2000;

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
  float x = ReadMag(sensor, 'x');
  float y = ReadMag(sensor, 'y');
  float z = ReadMag(sensor, 'z');
  float absolute = sqrt(sq(x) + sq(y) + sq(z));
  return absolute;
  
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
  mag.begin();
}

float ReadMag(Adafruit_MLX90393 sensor, char axis) {
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
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}

void ActivateMarker() {
  /* THIS HAS NOT BEEN IMPLEMENTED YET
   *  Function to activate marking system. Specs TBC
   */
}


void loop() {
  // put your main code here, to run repeatedly:
  if(LeftDistance.measureDistanceCm() > DIST_THRESH and RightDistance.measureDistanceCm() > DIST_THRESH and MagAbsolute(mag) < abs_mag_thresh) {
    analogWrite(PWM_A, DRIVE_SPEED);
    analogWrite(PWM_B, DRIVE_SPEED);
  }
  if(LeftDistance.measureDistanceCm() <= DIST_THRESH and RightDistance.measureDistanceCm() <= DIST_THRESH) {
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
  else if(LeftDistance.measureDistanceCm() <= DIST_THRESH) {
    Stop();
    // TURN RIGHT
    int turn_time = random(100, MAX_TURN_TIME);
    TightRight(turn_time);
  }
  else if(RightDistance.measureDistanceCm() <= DIST_THRESH) {
    Stop();
    // TURN LEFT:
    int turn_time = random(100, MAX_TURN_TIME);
    TightLeft(turn_time);
  }
  if(MagAbsolute(mag) >= abs_mag_thresh) {
    ActivateMarker();
  }
}
