/* LoRaTX_EZ_3.ino 

Uses LoRa library by Sandeed Mistry to Tx data between two LoRa modules. 

Demonstrates two-way data transfer between 2 LoRa modules
Each UNO toggles the LED on the OTHER Uno if the addresses are correct. 

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

// Outgoing message variable name:
String outMessage;

// Message counter
byte msgCount = 0;

// Received message variables
String contents = "";
String buttonPress = "Tx button pressed";
bool rcvButtonState;

// Source and destination addresses - be sure to reverse these when programming the second module...
byte localAddress = 0xAA;  // Address of the sending device - this one [0xCC]
byte destination = 0xCC;  // Address of destination/remote device [0xAA]


// Variables for the momentary pushbutton on Pin 8.
int buttonPin = 8;
int TxButtonState;

void setup() {
  Serial.begin(9600); 
  while (!Serial);

  // Make sure the pushbutton pin is an input
  pinMode(buttonPin, INPUT_PULLUP);

  // COnfigure LED pin
  pinMode(ledPin, OUTPUT);

// Configure the pins for the LoRa module
  LoRa.setPins(csPin, rstPin, irqPin);


  Serial.println("Started LoRa Duplex test with callback.");
  Serial.print("Device address: "); 
  Serial.println(localAddress, HEX);

  if (!LoRa.begin(915000000)) {
    Serial.println("LoRa Tx initialization failed!");
    while(1);
  }
  // Specify the receive callback function
  LoRa.onReceive(onReceive);

  // Place module in receive mode
  LoRa.receive();

  Serial.println("LoRa init succeeded.");

}

void loop() {

  // Get pushbutton state
  TxButtonState = digitalRead(buttonPin);
  
  // Send packet if button was pressed
  if (TxButtonState == LOW) {
 
    // Compose and send message
    outMessage = buttonPress;
    sendMessage(outMessage);
    delay(500);

    // Return LoRa module to receive mode
    LoRa.receive();
  }

}

// Send LoRa packet
void sendMessage(String outgoing){
  LoRa.beginPacket();                   // Start packet
  LoRa.write(destination);              // Destination address
  LoRa.write(localAddress);             // Sender's address
  LoRa.write(msgCount);                 // Add message ID
  LoRa.write(outgoing.length());        // Add payload length
  LoRa.print(outgoing);                 // Add payload
  LoRa.endPacket();                     // Finish up packet and send it
  msgCount++;                           // Increment message ID
}


// Receive callback
void onReceive(int packetSize){
  if (packetSize == 0) return;        // If no packet, return

  // Read packet header bytes
  int recipient = LoRa.read();        // Recipient address
  byte sender = LoRa.read();          // Sender address
  byte incomingMsgId = LoRa.read();   // Incoming message ID
  byte incomingLength = LoRa.read();  // Incoming message length

  String incoming = "";               // Packet payload

  while (LoRa.available()) {          // Can't use readstring in a callback
    incoming += (char)LoRa.read();    // Addbytes one-by-one
  }

  if (incomingLength != incoming.length()){
    Serial.println("Error: lengths don't match");
    return;
  }

   // If the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;  // skip rest of function
  }
 
  // If message is for this device, or broadcast, print details:
  Serial.println();
  Serial.println();
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
 
 
  // Toggle button state
  if (incoming.equals(buttonPress)) {
    rcvButtonState = !rcvButtonState;
  }
 
  // Drive LED
  if (rcvButtonState == true) {
    digitalWrite(ledPin, HIGH);
    Serial.println("LED turned  ON");
  } else {
    digitalWrite(ledPin, LOW);
    Serial.println("LED turned OFF");
  }






}






























