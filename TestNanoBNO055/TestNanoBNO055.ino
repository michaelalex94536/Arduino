/*
Just a simple check to visualize the 3-sensor raw data of the BNO055
using the Arduino Nano BLE 33 board.  Nine (9) signals are displayed. 
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Here are the Nano LED pins I think.
 #define RED 22     
 #define BLUE 24     
 #define GREEN 23

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  // put your setup code here, to run once:
  // The LEds are outputs of course
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);

  Serial.begin(115200);
  myIMU.begin();
  digitalWrite(RED, HIGH);
  delay(1000);
  myIMU.setExtCrystalUse(true);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, HIGH);

  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  Serial.print(acc.x());
  Serial.print(",");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print(acc.z());
  Serial.print(",");

  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.print(gyro.z());
  Serial.print(",");

  Serial.print(mag.x());
  Serial.print(",");
  Serial.print(mag.y());
  Serial.print(",");
  Serial.println(mag.z());

  delay(BNO055_SAMPLERATE_DELAY_MS);

}
