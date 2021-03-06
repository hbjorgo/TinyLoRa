// Hello LoRa - ABP TTN Packet Sender (Multi-Channel)
// Tutorial Link: https://learn.adafruit.com/the-things-network-for-feather/using-a-feather-32u4
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Copyright 2015, 2016 Ideetron B.V.
//
// Modified by Brent Rubell for Adafruit Industries, 2018
/************************** Configuration ***********************************/
#include "TinyLoRa.h"
#include <SPI.h>

// Visit your thethingsnetwork.org device console
// to create an account, or if you need your session keys.

// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x96, 0x25, 0x1D, 0x5B, 0xD2, 0xFE, 0x6B, 0xA8, 0x23, 0x8F, 0x46, 0xC5, 0xDC, 0x90, 0xD5, 0x93 };
// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x5C, 0x2C, 0x27, 0x11, 0x80, 0x54, 0x63, 0xC1, 0xB1, 0x35, 0x41, 0x00, 0xD9, 0xE2, 0x3E, 0xBF };
// Device Address (MSB)
uint8_t DevAddr[4] = { 0x26, 0x01, 0x3E, 0x8E };


uint8_t DevEui[8] = { 0x00, 0x4B, 0x38, 0x0B, 0x9E, 0x86, 0x3B, 0x5D }; // Dev EUI
uint8_t JoinEui[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0xD2, 0x31 }; // App EUI
uint8_t AppKey[16] = { 0x89, 0x66, 0x6D, 0x50, 0x72, 0x0D, 0xCE, 0x1C, 0x4A, 0x14, 0xA3, 0x28, 0x1C, 0x06, 0x14, 0xF6 }; // App Key
uint8_t NwkKey[16] = {};

/************************** Example Begins Here ***********************************/
// Data Packet to Send to TTN
unsigned char loraData[11] = {"A"};

// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 20;

// Pinout for Adafruit Feather 32u4 LoRa
//TinyLoRa lora = TinyLoRa(7, 8, 4);

// Pinout for Adafruit Feather M0 LoRa
TinyLoRa lora = TinyLoRa(3, 8, 4);

void setup()
{
  delay(2000);
  Serial.begin(9600);
  while (! Serial);
  
  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize LoRa
  Serial.print("Starting LoRa...");
  // define multi-channel sending
  lora.setChannel(MULTI);
  // set datarate
  lora.setDatarate(SF7BW125);
  if(!lora.begin())
  {
    Serial.println("Failed");
    Serial.println("Check your radio");
    while(true);
  }

  // Optional set transmit power. If not set default is +17 dBm.
  // Valid options are: -80, 1 to 17, 20 (dBm).
  // For safe operation in 20dBm: your antenna must be 3:1 VWSR or better
  // and respect the 1% duty cycle.

  // lora.setPower(17);

  Serial.println("OK");
}

void loop()
{
  Serial.println("Sending LoRa Data...");
  lora.sendData(loraData, sizeof(loraData), lora.frameCounter);
  // Optionally set the Frame Port (1 to 255)
  // uint8_t framePort = 1;
  // lora.sendData(loraData, sizeof(loraData), lora.frameCounter, framePort);
  Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
  lora.frameCounter++;

  // blink LED to indicate packet sent
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("delaying...");
  delay(sendInterval * 1000);
}
