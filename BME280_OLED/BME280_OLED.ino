/* 

Here we connect a BME280 sensor and OLED display to the ESP32 Dev module using SPI

Temperature, humidity and air pressure are displayed on the OLED.  

The SPI CLK and MOSI lines are common to both peripherals, which have unique CS lines. 

*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define BME_CLK     18
#define BME_MISO    19
#define BME_MOSI    23
#define BME_CS       5
#define SEALEVELPRESSURE_HPA (1013.25)

#define OLED_MOSI   23
#define OLED_CLK    18
#define OLED_DC     16
#define OLED_CS     15  //5
#define OLED_RESET  17

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128

Adafruit_SH1107 OLED(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

Adafruit_BME280 BME(BME_CS, BME_MOSI, BME_MISO, BME_CLK); // software SPI

unsigned long delayTime;

void setup() {

  unsigned BME_status;
  unsigned OLED_status;

  Serial.begin(115200);
  delay(100);

  if(!Serial)
  {
    Serial.println(F("Serial port initialization failed"));
    return;
  }

  OLED_status = OLED.begin(SH110X_SETCOMPINS);
  delay(100);
  if(!OLED_status)
  {
    Serial.println(F("OLED initialization failed"));
    return;
  }

  BME_status = BME.begin(); 
  delay(100); 
  if (!BME_status) {
      Serial.println("Could not find a valid BME280 sensor - check your wiring!");
      return;
  }

  if (BME_status){
    Serial.println("BME280 found.");
  }

  if (OLED_status){
    Serial.println("OLED found.");
  }

  OLED.clearDisplay();

}



void loop() {
  // put your main code here, to run repeatedly:
OLEDDisplay();
}


void OLEDDisplay()
{
  OLED.clearDisplay();

  OLED.setTextSize(1);
  OLED.setTextColor(SH110X_WHITE);
  OLED.setCursor(15,18);
  OLED.println("Temperature [C]");
  OLED.setTextSize(2);
  OLED.setCursor(32, 28);
  //OLED.println("0.0/32.0");
  OLED.println(BME.readTemperature());

  OLED.setTextSize(1);
  OLED.setTextColor(SH110X_WHITE);
  OLED.setCursor(24,58);
  OLED.println("Humidity [%]");
  OLED.setTextSize(2);
  OLED.setCursor(32, 68);
  //OLED.println("50.0");
  OLED.println(BME.readHumidity());


  OLED.setTextSize(1);
  OLED.setTextColor(SH110X_WHITE);
  OLED.setCursor(22,98);
  OLED.println("Pressure [hPa]");
  OLED.setTextSize(2);
  OLED.setCursor(22, 108);
  //OLED.println("1013.25");
  OLED.println(BME.readPressure()/100.0);

  OLED.display();
  delay(1000);
}