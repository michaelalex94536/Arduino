/* LoRaRX_EZ_2.ino 

Uses LoRa library by Sandeed Mistry to Rx data from a transmitter. 

In this version we use the Tx bushbutton to turn on the Rx LED  - one way only.
Be sure the message you receive that is supposed to denote the button press is exactly the same as 
the message the Tx side of things is sending!

There is a pushbutton between Uno Pin 8 and ground. 

There is an LED and resistor between Uno Pin 5 and ground. 

Hardware: Arduino Uno and Adafruit 915MHz RFM9x module.  

*/

#include <LoRa.h>
#include "SPI.h"

// Here are the LoRa module pins <---> Uno pins
const int csPin = 4;     // LoRa module chip select
const int rstPin = 2;    // LoRa module reset pin
const int irqPin = 3;    // LoRa module interrupt (G0) pin. Must be arduin HW interrupt pin


/* We use the Uno software SPI pins
 LoRa module  SCK Pin <---> Uno Pin 13
 LoRa module MISO Pin <---> Uno Pin 12
 LoRa module MOSI Pin <---> Uno Pin 11

There is a pushbutton between Uno Pin 8 and ground
 Pushbutton <---> Uno Pin 8

*/


// LED - 330 Ohm resistor connection
const int ledPin = 5;

// Receive message variables
String contents = "";
String btnPress = "Button Pressed";
bool rcvButtonState;

// Message counter
byte msgCount = 0;


void setup() {

  // Set LED as output
  pinMode(ledPin, OUTPUT);

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

  // Received packet
  if(packetSize){

    Serial.print("Received packet '");
  
    // Read packet
    while(LoRa.available()){
      contents += (char) LoRa.read();
    }

    // Print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    Serial.println(contents);

    // Toggle button state
    if (contents.equals(btnPress)){
      rcvButtonState = !rcvButtonState;
    }

    //Drive LED
    if (rcvButtonState == true) {
      digitalWrite(ledPin, HIGH);
      Serial.println("LED ON");
      } else {
        digitalWrite(ledPin, LOW);
        Serial.println("LED OFF");
      }    
  
      // Clear contents
      contents = "";

  }


}
