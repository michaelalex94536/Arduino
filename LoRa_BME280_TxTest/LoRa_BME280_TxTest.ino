//
// LoRa_BME280_TX on the UNO
// Transmit BME280 sensor using LoRa and an Arduino
// The wiring here is Arduin Uno specific!!!!!  
//
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX

#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SERIAL_BAUD 9600

// Here is the signal wiring between the RFM9x module and the Uno. These pins need to be explicitly defined.
// Note that the RFM95_INT pin must be an Arduino hardware interrupt pin or it won't work.
#define RFM95_CS  6  // RFM9x CS <---> Uno digital pin 6
#define RFM95_RST 5  // RFM9x RST <---> Uno digital pin 5 
#define RFM95_INT 3  // RFM9x G0 <---> Uno digital pin 3  YOU MUST UES THIS PIN FOR THE UNO!!! 

// Set Tx frequency in MHz
#define RF95_FREQ 915.0

// Here is the SPI wiring between the RFM9x module and the Uno. These pins DO NOT need to be explicitly defined. 
// These are hardware SPI pins and can't be changed in this version.
// RFM9x  SCK <---> Uno digital pin 13
// RFM9x MISO <---> Uno digital pin 12
// RFM9x MOSI <---> Uno digital pin 11 

// Here is the SPI wiring between the BME280 module and the Uno. These pins DO NOT need to be explicitly defined. 
// These are software SPI pins and the use can change them.   
#define BME_MOSI 2   // BME MOSI <---> Uno digital pin 2
#define BME_MISO 8   // BME MISO <---> Uno digital pin 8
#define BME_SCK  4   // BME SCK  <---> Uno digital pin 4
#define BME_CS   7   // BME CS <---> Uno digital pin 7

// Create an instance of the sensor:
Adafruit_BME280 BME(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
// Adafruit_BME280 BME(BME_CS);

// Create an instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// This structure holds temperature, humidity, and pressure, respectively:
struct myResults
{
  float res1, res2, res3;
} results;

float temp = 0.0;



void setup() 
{
  unsigned BME_status;

  Serial.begin(SERIAL_BAUD);

  while(!Serial) {} // Wait


  BME_status = BME.begin(); 
  delay(100); 
  if (!BME_status) {
      Serial.println("Could not find a valid BME280 sensor - check your wiring!");
      return;
  }

  if (BME_status){
    Serial.println("BME280 found.");
    Serial.print("BME280 sensor ID: ");
    Serial.println(BME.sensorID(), 16);
  }

  delay(500);


  // Set the RFM9x reset pin high
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);


  Serial.println("Arduino LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    //while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);


}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{

Serial.println("Looping stars here\r\n");

  char buffer[7];

  GetData();
  temp = results.res1;
  Serial.print("Measured temperature of: ");
  Serial.print(temp);
  Serial.println(" degrees.");

// How to convert float to char:
  dtostrf(temp, 5, 2, buffer);
  Serial.println(buffer);

  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
 char radiopacket[20] = "Hello World #      ";

  itoa(packetnum++, radiopacket+13, 10); 
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("Sending..."); delay(10);
  //rf95.send((uint8_t *)radiopacket, 20);
  rf95.send((uint8_t *) radiopacket, sizeof(radiopacket));

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?\r\n");
  }
  
}

//////////////////////////////////////////////////////////////////
myResults GetData()
{

   Serial.print("Temperature [C]: ");
   results.res1 = BME.readTemperature();
   Serial.println(results.res1);
   temp = results.res1;
   delay(1000);

   Serial.print("Humidity [%]: ");
   results.res2 = BME.readHumidity();
   Serial.println(results.res2);
   delay(1000);

   Serial.print("Pressure [mbar]: ");
   results.res3 = BME.readPressure()/100.0;
   Serial.println(results.res3);
   delay(1000);

   return(results);
 /*  client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("Pa");
*/
}
