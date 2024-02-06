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

float filtWt = 0.95;  // How much weight to give to old value

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define pi (3.141592654)

Adafruit_BNO055 myIMU = Adafruit_BNO055();
uint8_t sys_cal, gyro_cal, acc_cal, mag_cal;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  myIMU.setExtCrystalUse(true);
}

void loop() {
   // put your main code here, to run repeatedly:

  myIMU.getCalibration(&sys_cal, &gyro_cal, &acc_cal, &mag_cal);

  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  thetaM = -atan2(acc.x(), acc.z()) * (180.0/pi); // Need minus sign so nose up is +ve angle
  phiM = -atan2(acc.y(), acc.z()) * (180.0/pi);   // Need minus sign so left side up is +ve angle

// Here are the LPF angles
  thetaFiltNew = filtWt * thetaFiltOld + (1.0 - filtWt)*thetaM;
  phiFiltNew = filtWt * phiFiltOld + (1.0 - filtWt)*phiM;

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
  Serial.print(thetaFiltOld);
  Serial.print(",");
  Serial.println(phiFiltOld);

  delay(BNO055_SAMPLERATE_DELAY_MS);

}
