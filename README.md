##  Arduino projects (mostly)

####  Never was much of an Arduino fan, but it is the fastest way to go if you want to play with a new sensor.  (Once I get familiar with the sensor, I'll move the project to something like an STM32 microcontroller.)  While Arduino libraries are nice and get you up to speed quickly, so much stuff that I would like to be aware of gets hidden in them.  For example, what's the I2C address, what pins is the SPI using, what is the SPI clock rate, etc?  As time goes on, I plan to move as much as possible from the Arduino IDE to VS Code with the PlatformIO plug-in.  

###  BNO055-related projects

####  These projects are mostly about using the BNO055 9-axis Adafruit IMU Fusion breakout board with the Arduino Nano 33 BLE.   
</br>

#### The connections between the Nano 33 BLE and the Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout-BNO055 are shown here.  It's pretty simple - only 4 connections are required.  I tried to use 5V from the Nano to power the Vin pin of the 9-DOF board, but I didn't see any sensor outputs; meanwhile, 3.3 V worked fine.   


### Nano 33 BLE pin . . . . . . . . . . 9-DOF Breakout Board pin 
#### 3.3 V <------------------------------------------------------------> Vin 
#### GND   <------------------------------------------------------------> GND
#### I2C SCL - A5 <-------------------------------------------------> I2C SCL
#### I2C SDA - A4 <-------------------------------------------------> I2C SDA    
</br>
</br>

### Project listing and decription (sort of increasing in complexity)

#### 1) TestNanoBNO055
This project sends all 9-axis data from the three sensors to the serial port. 

#### 2) BNO055_Cal
This project continuously shows the calibration factors of the gyro, accelerometer, amgnetometer, and "system" as well as the acceleration in X, Y, and Z in m/sec^2.  A factor of 0 means the sensor is completely uncalibrated, and 3 means the sensor in question is completely calibrated.  You can twirl the board around and see how this affects calibration.  Calibration of the accelerometer is a bit tricky.  If you can get the accelerometer calibrated, you can lay the board flat and see if you measure gravity to be close to 9.8 m/sec^2.  
</br>
</br>

###  SH1107 OLED Display-related projects

####  These projects are mostly about using the Grove OLED Display 1.12 (SH1107) V3.0 with the ESP32 Dev module (ESPRESSIF ESP32-WROOM-32D) using SPI (NOT I2C!!!)    

#### The connections between the ESP32 dev module and the OLED Display SPI connections are shown here. If you want, you can solder two pads together on the back of the OLED PCB and use I2C instead of SPI. In fact this is the default and mostly commonly-used configuration.   


### ESP32 Dev Board pin [GPIO #]. . . . . . . . . . OLED Breakout Board pin label 
#### 3.3 V <------------------------------------------------------------> 5V 
#### GND   <------------------------------------------------------------> GND
#### MOSI [23] <------------------------------------------------------> SI
#### SCK [18] <-------------------------------------------------------> SCL
#### RX2 [16] <-------------------------------------------------------> DC
#### SS [5] <---------------------------------------------------------> CS
#### TX2 [17] <------------------------------------------------------> RES
</br>
</br>
