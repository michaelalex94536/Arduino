/*
Calibration of the BNO055 IMU using the Arduino Nano BLE 33

Accelerometer calibration really sucks, so we report the acceleration too. 
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();
uint8_t sys_cal, gyro_cal, acc_cal, mag_cal = 0;
sensor_t sensor;
long bnoID;

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

  if(myIMU.isFullyCalibrated()){

    Serial.println("\nCALIBRATION COMPLETE!!");

    adafruit_bno055_offsets_t newCalib;

    myIMU.getSensorOffsets(newCalib);

    myIMU.getSensor(&sensor);
    bnoID = sensor.sensor_id; 
    
  }

  Serial.print(acc.x());
  Serial.print(",");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print(acc.z());
  Serial.print(",");

  Serial.print(sys_cal);
  Serial.print(",");
  Serial.print(mag_cal);
  Serial.print(",");
  Serial.print(acc_cal);
  Serial.print(",");
  Serial.print(gyro_cal);
  Serial.println(",");

  delay(BNO055_SAMPLERATE_DELAY_MS);

}
