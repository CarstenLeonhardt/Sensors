/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 - Henrik Ekblad
   Contribution by: Derek Macias

   DESCRIPTION
   Example showing how to create an atuator for a servo.
   Connect red to +5V, Black or brown to GND and the last cable to Digital pin 3.
   The servo consumes much power and should probably have its own powersource.'
   The arduino might spontanally restart if too much power is used (happend
   to me when servo tried to pass the extreme positions = full load).
   http://www.mysensors.org/build/servo
*/
/*
 * HW: Arduino Promini 8mhz 3.3V + neopixel RGB ring
 * Gree if alarm is disarmed, Red if Armed
 */


// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

//43 Entrance/Hall
//44 Basement
//45 desktop
//49 desktop2
#define MY_NODE_ID 43 // Sets a static id for a node


#include <SPI.h>
#include <MySensors.h>
#include <Adafruit_NeoPixel.h>

static const uint64_t UPDATE_INTERVAL = 60000; //60000;


//Neopixxel led ring PIN
#define PIN 6
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      12
#define CHILD_ID_LED 2

#define FADE_DELAY 50//75// 100  // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim) LED
uint32_t prevMillis;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
//Neopixel animation delay
int delayval = 150; // delay for half a second

MyMessage lightMsg(CHILD_ID_LED, V_STATUS); // NeoPixel msg


void setup()
{
  Serial.println("setup()");

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(10); // limit the brightness
  Serial.println("change color");
  SetColor(204,204,0,0);
  pixels.show(); // set pixel output
  
  // Request last servo state at startup
  request(CHILD_ID_LED, V_STATUS);
}

void presentation()
{
  Serial.println("presentation()");
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Alarm Armed", "1.0");

  Serial.println("S_BINARY");
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_LED, S_BINARY,"Armed", false);
}

void loop()
{
  if (millis() - prevMillis >= UPDATE_INTERVAL) 
  {
    prevMillis += UPDATE_INTERVAL;
    Serial.println("send heartbeat to controller");
    sendHeartbeat();
  }
}

void SetColor(unsigned char r,unsigned char g,unsigned char b, unsigned int delayv)
{
  for(int i=0; i<NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(r,g,b)); // Moderately bright green color.
    pixels.setPixelColor(i+(NUMPIXELS/2), pixels.Color(r,g,b)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayv); // Delay for a period of time (in milliseconds).
  }
}


void receive(const MyMessage &message)
{
      Serial.print("Receive message: ");

  if (message.type == V_STATUS)
  {
    Serial.print("Receive light message: ");
    Serial.print("V_STATUS: ");

 
    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );
    Serial.print("Value: ");
    Serial.println(requestedLevel);
      if (message.sensor == CHILD_ID_LED) //is the message for this node
    {
      Serial.print( "Changing level to " );
      Serial.print( requestedLevel );
      

      if (requestedLevel)
      {
        Serial.println( " - Red Color");
        SetColor(155,0,0,delayval);//Red
      }
      else
      {
        Serial.println( " - Green Color");
        SetColor(0, 155,0,delayval); // Green
      }
    }
  }
  else
  {
      Serial.print("unknown type:: ");
      Serial.println(message.type);
  }
}
