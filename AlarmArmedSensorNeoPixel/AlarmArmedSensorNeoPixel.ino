/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0: Henrik EKblad
 * Version 1.1 - 2016-07-20: Converted to MySensors v2.0 and added various improvements - Torben Woltjen (mozzbozz)
 * 
 * DESCRIPTION
 * This sketch provides an example of how to implement a humidity/temperature
 * sensor using a DHT11/DHT-22.
 *  
 * For more information, please visit:
 * http://www.mysensors.org/build/humidity
 * 
 */

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached 
#define MY_RADIO_RF24

//43 Entrance/Hall
//44 Basement
//45 desktop
//49 desktop2
#define MY_NODE_ID 65 // Sets a static id for a node

//#define MY_RF24_PA_LEVEL RF24_PA_HIGH // High tx power
//#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power

#define MY_REPEATER_FEATURE //enable package repeater

//#include <SPI.h>
#include <MySensors.h>  
//#include <DHT.h>
#include <Adafruit_NeoPixel.h>

// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 3

//Neopixxel led ring PIN
#define PIN 6

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      12

#define FADE_DELAY 10  // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim) LED


// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000; //60000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]

#define CHILD_ID_LED 2


static int16_t currentLevel = 0;  // Current dim level... //LED
uint32_t prevMillis;
uint32_t dataAge = 0;
unsigned char requestcnt = 0;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Neopixel animation delay
int delayval = 150; // delay for half a second

MyMessage lightMsg(CHILD_ID_LED, V_STATUS); // LED


void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("Alarm Armed", "1.2");
  //delay(100);
  Serial.println("presentation()");
  // Register all sensors to gw (they will be created as child devices)

  Serial.println("S_LIGHT");
  present( CHILD_ID_LED, S_BINARY,"Neopixel light", false );
}


void setup()
{ 
  Serial.println("setup()");
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(10); // limit the brightness
  Serial.println("change color");
  SetColor(204,204,0,0);
  pixels.show(); // set pixel output
  
  Serial.println("Request led state setup()");
  request( CHILD_ID_LED, V_LIGHT ); //LED
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

//led
void receive(const MyMessage &message)
{
      Serial.print("Receive message: ");

  if (message.type == V_STATUS)
  {
    Serial.print("Receive light message: ");
    Serial.println("V_STATUS");

 
    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );

    // Adjust incoming level if this is a V_STATUS variable update [0 == off, 1 == on]
    requestedLevel *= ( message.type == V_STATUS ? 100 : 1 );

    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;

    if (message.sensor == CHILD_ID_LED) //is the message for this node
    {
      //Serial.println( "Yellow LED" );
      Serial.print( "Changing level to " );
      Serial.println( requestedLevel );

      if (requestedLevel > 10)
        SetColor(155,0,0,delayval);
      else
        SetColor(0, 155,0,delayval);
      Serial.print(dataAge);
      dataAge = 0; //reset data age counter
      Serial.print(" :Reset Data age cnt: ");
      Serial.println(dataAge);
      requestcnt = 0; //reset counter  
    }
  }
  else
  {
      Serial.print("unknown type:: ");
      Serial.println(message.type);
  }
}

void loop()      
{  
  if (millis() - prevMillis >= UPDATE_INTERVAL) 
  {
    //Serial.print("Slow Loop: millis: ");
    //Serial.println(prevMillis);
    prevMillis += UPDATE_INTERVAL;

    //send heartbeat to controller
    //sendHeatbeat(); //2.3.1
    Serial.println("send heartbeat to controller");
    sendHeartbeat();
    //Serial.println("send light value to controller");
    //send(lightMsg.set(1,1));

    
    if (requestcnt < 6)
    {
      requestcnt++;
      Serial.print("Increment requestcnt: ");
      Serial.println(requestcnt);
    }
    else
    {
      Serial.println("Request led state: ");
      request( CHILD_ID_LED, V_LIGHT ); //LED
      requestcnt = 0;
    }

 
     
    dataAge++;
    Serial.print("Increment Data age cnt: ");
    Serial.println(dataAge);
    if (dataAge > 10)
    {
      Serial.println("data too old, go blue");
      //SetColor(0,0, 155,0); //set blue color
    }
 
  }
}
