/* LoRaRX_EZ_1.ino 

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

  Serial.println("Started LoRa Rx test.");

  if (!LoRa.begin(915000000)) {
    Serial.println("LoRa Tx initialization failed!");
    while(1);
}

}

void loop() {
  
// Try to parse an incoming packet
  int packetSize = LoRa.parsePacket();
  if(packetSize){
    // Received a packet
    Serial.print("Received '");

    // Read packet
    while(LoRa.available()){
      Serial.print((char) LoRa.read());
    }

    // Print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" and SNR ");
    Serial.println(LoRa.packetSnr());
  
  }


}
