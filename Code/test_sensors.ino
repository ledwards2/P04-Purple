/* Function to test sensors on robot.
 *  Sensors used: 2x HC-SR04 Ultrasonic Sensors
 *  1x Melexis MLX90393 High Range Magnetometer
 * Sensor output is displayed on computer through serial print.  
 * 
 */

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



void setup() {
  // put your setup code here, to run once:

  /* MLX90393 TEST CODE FROM Adafruit Industries MLX90393 Library, by Kevin Townsend. Available at: https://github.com/adafruit/Adafruit_MLX90393_Library */
  Serial.begin(9600);

  while(!Serial) {
    delay(10);
  }
  if(mag.begin()) {
    Serial.println("MLX90393 Found");
  }
  else {
    Serial.println("no MLX90393");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  /* This section taken from Adafruit_MLX90393_basicdemo.H */
  float x, y, z;

  if(mag.readData(&x, &y, &z)) {
    Serial.println("Magnetometer information:");
    Serial.print("X: "); Serial.print(x, 4); Serial.println(" uT");
    Serial.print("Y: "); Serial.print(y, 4); Serial.println(" uT");
    Serial.print("Z: "); Serial.print(z, 4); Serial.println(" uT");
  } 
  else {
    Serial.println("Unable to read XYZ data from the magnetometer.");
  }

  /* This section from HCSR04_simple.h, courtesy Martin Sosic */
  Serial.println("Distance Information - Left:"
  Serial.printLn(LeftDistance.measureDistanceCm());
  Serial.println("Distance Information - Right");
  Serial.println(RightDistance.measureDistanceCm());
  delay(5000);
  
}
