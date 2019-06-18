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
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RS485

#define MY_NODE_ID 44 //43 // Sets a static id for a node

#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power
 
#include <SPI.h>
#include <MySensors.h>  
#include <DHT.h>
//#include <Adafruit_NeoPixel.h>
#include <OLED_I2C.h>



// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 3

#define LED_PIN 5      // Arduino pin attached to MOSFET Gate pin //analog pwm pin

//Neopixxel led ring PIN
#define PIN            6

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
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_LED 2


float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
static int16_t currentLevel = 0;  // Current dim level... //LED
uint32_t prevMillis;

//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Neopixel animation delay
int delayval = 150; // delay for half a second

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage lightMsg(CHILD_ID_LED, V_LIGHT); // LED

DHT dht;

OLED  myOLED(SDA, SCL, 8);

extern uint8_t SmallFont[];
//extern uint8_t MediumNumbers[];
extern uint8_t BigNumbers[];
extern uint8_t Line1[4]="ABE";

void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("TemperatureAndHumidity", "1.1");
  
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);

  present( CHILD_ID_LED, S_LIGHT );
}


void setup()
{ 
  pinMode(LED_PIN, OUTPUT);       // sets the pin as output 
  //pinMode(LED_PIN2, OUTPUT);       // sets the pin as output  led2
  
//  pixels.begin(); // This initializes the NeoPixel library.
//  pixels.setBrightness(10); // limit the brightness
//  SetColor(0,0, 155,0);
//  pixels.show(); // turn off all pixels
  
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) 
  {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());

  Serial.println("Request led state");
  request( CHILD_ID_LED, V_LIGHT ); //LED
  myOLED.begin();
//  myOLED.print("OLED_I2C Scrolling Text Demo ");
//  myOLED.update();
  
}

//void SetColor(unsigned char r,unsigned char g,unsigned char b, unsigned int delayv)
//{
//  for(int i=0; i<NUMPIXELS; i++)
//  {
//    pixels.setPixelColor(i, pixels.Color(r,g,b)); // Moderately bright green color.
//    pixels.setPixelColor(i+(NUMPIXELS/2), pixels.Color(r,g,b)); // Moderately bright green color.
//    pixels.show(); // This sends the updated pixel color to the hardware.
//    delay(delayv); // Delay for a period of time (in milliseconds).
//  }
//}

//led
void receive(const MyMessage &message)
{
  if (message.type == V_LIGHT || message.type == V_DIMMER)
  {
    Serial.println("Receive light message");

    Serial.print( "Message Sensor: " );
    Serial.println( message.sensor );
    
    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );

    // Adjust incoming level if this is a V_LIGHT variable update [0 == off, 1 == on]
    requestedLevel *= ( message.type == V_LIGHT ? 100 : 1 );

    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;

    if (message.sensor == CHILD_ID_LED) //is the message for this node
    {
      Serial.println( "Yellow LED" );
      Serial.print( "Changing level to " );
      Serial.println( requestedLevel );
      fadeToLevel(requestedLevel);

//      if (requestedLevel > 10)
//        SetColor(155,0,0,delayval);
//      else
//        SetColor(0, 155,0,delayval);
    }
  }
}

//led

/***
 *  This method provides a graceful fade up/down effect
 */
void fadeToLevel( int toLevel )
{

  int delta = ( toLevel - currentLevel ) < 0 ? -1 : 1;

  while ( currentLevel != toLevel ) 
  {
    currentLevel += delta;
    analogWrite( LED_PIN, (int)(currentLevel / 100. * 255) );

    delay( FADE_DELAY );
  }
}

///https://forum.mysensors.org/topic/1424/resend-if-st-fail/9

void resend(MyMessage &msg, int repeats)
{
  int repeat = 1;
  int repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) 
  {
    if (send(msg)) 
    {
      sendOK = true;
    } 
    else 
    {
      sendOK = false;
      Serial.print("TX ERROR: ");
      Serial.println(repeat);
      repeatdelay += 250;
    } 
    repeat++; 
    delay(repeatdelay);
  }
}


void loop()      
{  
  if (millis() - prevMillis >= UPDATE_INTERVAL) 
  {
//    Serial.print("Slow Loop: millis: ");
//    Serial.println(prevMillis);
    prevMillis += UPDATE_INTERVAL;
  
    // Force reading sensor, so it works also after sleep()
    dht.readSensor(true);
//    Serial.print("Dht.getStatusString: ");
//    Serial.println(dht.getStatusString());
    
    
    // Get temperature from DHT library
    float temperature = dht.getTemperature();

    if (isnan(temperature)) 
    {
      sleep(dht.getMinimumSamplingPeriod()); //wait a bit
      temperature = dht.getTemperature();
    }

    Serial.print("T: ");
    Serial.println(temperature);
    
    if (isnan(temperature)) 
    {
      Serial.println("Failed reading temperature from DHT!");
    } 
    else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) 
    {
      // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
      lastTemp = temperature;
      // Reset no updates counter
      nNoUpdatesTemp = 0;
      temperature += SENSOR_TEMP_OFFSET;
      send(msgTemp.set(temperature, 1));
  
      #ifdef MY_DEBUG
      Serial.print("T: ");
      Serial.println(temperature);
      #endif
      myOLED.setFont(SmallFont);
      myOLED.print("T: ", LEFT, 1);
      myOLED.update();
      myOLED.setFont(BigNumbers);
      myOLED.printNumF(temperature, 1, RIGHT, 0);
      myOLED.update();
    } 
    else 
    {
      // Increase no update counter if the temperature stayed the same
      nNoUpdatesTemp++;
    }
  
    // Get humidity from DHT library
    float humidity = dht.getHumidity();

     if (isnan(humidity)) 
    {
      sleep(dht.getMinimumSamplingPeriod()); //wait a bit
      humidity = dht.getHumidity();
    }
    if (isnan(humidity)) 
    {
      Serial.println("Failed reading humidity from DHT");
    } 
    else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) 
    {
      // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
      lastHum = humidity;
      // Reset no updates counter
      nNoUpdatesHum = 0;
      send(msgHum.set(humidity, 1));
      
      #ifdef MY_DEBUG
      Serial.print("H: ");
      Serial.println(humidity);
      #endif
      myOLED.setFont(SmallFont);
      myOLED.print("H: ", LEFT, 40);
            myOLED.update();
      myOLED.setFont(BigNumbers);
      myOLED.printNumF(humidity, 1, RIGHT, 40);
      myOLED.update();

    } 
    else 
    {
      // Increase no update counter if the humidity stayed the same
      nNoUpdatesHum++;
    }
  }
}
