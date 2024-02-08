/*
Complimentary filter
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

float thetaM; // Tilt in "X" direction - same as pitch angle - measured
float phiM;   // Tilt in "Y" direction - same as roll - measured

float thetaG = 0;  // Pitch gyro measurement - Y axis rotation
float phiG = 0;    // Roll gyro measurement - X axis rotation

float thetaC = 0;   // Complementary filter pitch
float phiC = 0;    // Complementary filter roll

float dt;  // Time step

unsigned long millisRef;  // Time stamp for reference

float filtWt = 0.75;  // How much weight to give to the gyro value

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define pi (3.141592654)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  myIMU.setExtCrystalUse(true);
  millisRef = millis();
}

void loop() {
   // put your main code here, to run repeatedly:

  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  thetaM = -atan2(acc.x(), acc.z()) * (180.0/pi); // Need minus sign so nose up is +ve angle
  phiM = -atan2(acc.y(), acc.z()) * (180.0/pi);   // Need minus sign so left side up is +ve angle

  dt = (millis() - millisRef)/1000.0;
  millisRef = millis();

  thetaG = thetaG + gyro.y() * dt;
  phiG = phiG - gyro.x() * dt;
  
  /// Give most weight to the gyro - high pass the gyros effectively
  thetaC = (filtWt * thetaG) + (1.0 - filtWt) * thetaM;
  phiC = (filtWt * phiG) + (1.0 - filtWt) * phiM;

  Serial.print(",");
  Serial.print(thetaM);
  Serial.print(",");
  Serial.print(phiM);
  Serial.print(",");
  Serial.print(thetaG);
  Serial.print(",");
  Serial.print(phiG);

  Serial.print(",");
  Serial.print(thetaC);
  Serial.print(",");
  Serial.println(phiC);

  delay(BNO055_SAMPLERATE_DELAY_MS);

}
