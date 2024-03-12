/* LoRaTX_EZ_2.ino 

Uses LoRa library by Sandeed Mistry to Tx data to a receiver. 

In this version we use the Tx bushbutton to turn on the Rx LED - one way only.

Hardware: Arduino Uno and Adafruit 915MHz RFM9x module.  

There is a pushbutton between Uno Pin 8 and ground. 

There is an LED and resistor between Uno Pin 5 and ground. 

*/

#include <LoRa.h>
#include "SPI.h"

// Here are the LoRa module pins <---> Uno pins
const int csPin = 4;     // LoRa module chip select
const int rstPin = 2;    // LoRa module reset pin
const int irqPin = 3;    // LoRa module interrupt (G0) pin. Must be arduin HW interrupt pin due to the library beung used. 

/* We use the Uno software SPI pins
 LoRa module  SCK Pin <---> Uno Pin 13
 LoRa module MISO Pin <---> Uno Pin 12
 LoRa module MOSI Pin <---> Uno Pin 11

There is a pushbutton between Uno Pin 8 and ground
 Pushbutton <---> Uno Pin 8

*/

// LED - 330 Ohm resistor connection
const int ledPin = 5;


// Message counter
byte msgCount = 0;

// Variables for the momentary pushbutton on Pin 8.
int buttonPin = 8;
int TxButtonState;

void setup() {
  Serial.begin(9600); 
  while (!Serial);

  // Make sure the pushbutton pin is an input
  pinMode(buttonPin, INPUT_PULLUP);

// Configure the pins for the LoRa module
  LoRa.setPins(csPin, rstPin, irqPin);

  Serial.println("Started LoRa Tx test.");

  if (!LoRa.begin(915000000)) {
    Serial.println("LoRa Tx initialization failed!");
    while(1);
  }

}

void loop() {

  // Get pushbutton state
  TxButtonState = digitalRead(buttonPin);
  
  // Send packet if button was pressed
  if (TxButtonState == LOW) {
    // Send packet
    LoRa.beginPacket();
    LoRa.print("Button Pressed");
    LoRa.endPacket();
    msgCount++;
    Serial.print("Tx button pushed!       ");
    Serial.print("Sending packet: ");
    Serial.println(msgCount);
    delay(1000);
  }

}
