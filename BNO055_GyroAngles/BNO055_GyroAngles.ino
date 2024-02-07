/*
LPF of pitch and roll angles
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

float thetaM; // Tilt in "X" direction - same as pitch angle - measured
float phiM;   // Tilt in "Y" direction - same as roll - measured
float thetaFiltOld = 0.0, thetaFiltNew;  //  Old and new filtered theta values
float phiFiltOld = 0.0, phiFiltNew;  //  Old and new filtered phi values

float thetaG = 0;  // Pitch gyro measurement - Y axis rotation
float phiG = 0;    // Roll gyro measurement - X axis rotation

float dt;  // Time step

unsigned long millisRef;  // Time stamp for reference

float filtWt = 0.95;  // How much weight to give to old value

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

// Here are the LPF angles
  thetaFiltNew = filtWt * thetaFiltOld + (1.0 - filtWt)*thetaM;
  phiFiltNew = filtWt * phiFiltOld + (1.0 - filtWt)*phiM;

  dt = (millis() - millisRef)/1000.0;
  millisRef = millis();

  thetaG = thetaG + gyro.y() * dt;
  phiG = phiG - gyro.x() * dt;

// Update the "old" filter values:
  thetaFiltOld = thetaFiltNew;
  phiFiltOld = phiFiltNew;

  Serial.print(",");
  Serial.print(thetaM);
  Serial.print(",");
  Serial.print(phiM);
  Serial.print(",");
  Serial.print(thetaFiltNew);
  Serial.print(",");
  Serial.print(phiFiltNew);
  Serial.print(",");
  Serial.print(thetaG);
  Serial.print(",");
  Serial.println(phiG);

  delay(BNO055_SAMPLERATE_DELAY_MS);

}
