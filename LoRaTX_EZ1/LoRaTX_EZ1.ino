/* LoRaTX_EZ_1.ino 

Uses LoRa library by Sandeed Mistry to Tx data to a receiver. 

Hardware: Arduino Uno and Adafruit 900MHz RFM9x module.  

*/

#include <LoRa.h>
#include "SPI.h"

// Here are the LoRa module pins <---> Uno pins
const int csPin = 4;     // LoRa module chip select
const int rstPin = 2;    // LoRa module reset pin
const int irqPin = 3;    // LoRa module interrupt (G0) pin. Must be arduin HW interrupt pin


// Message counter
byte msgCount = 0;


void setup() {
  Serial.begin(9600); 
  while (!Serial);

// Configure the pins for the LoRa module
  LoRa.setPins(csPin, rstPin, irqPin);

  Serial.println("Started LoRa Tx test.");

  if (!LoRa.begin(915000000)) {
    Serial.println("LoRa Tx initialization failed!");
    while(1);
}

}

void loop() {
  
  Serial.print("Sending packet: ");
  Serial.println(msgCount); 

  // Send packet
  LoRa.beginPacket();
  LoRa.print("Packet ");
  LoRa.print(msgCount);
  LoRa.endPacket();

  // Increment message counter 
  msgCount++;

  // Add a delay
  delay(5000);

}
