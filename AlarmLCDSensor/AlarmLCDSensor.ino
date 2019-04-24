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

#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display


// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 3

#define LED_PIN 5      // Arduino pin attached to MOSFET Gate pin //analog pwm pin
//#define LED_PIN2 4 //led2      // Arduino pin attached to MOSFET Gate pin  digital pin

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000; //60000;
static const uint64_t MAIN_INTERVAL = 60000 / UPDATE_INTERVAL; //60000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_LED 2
//#define CHILD_ID_LED2 3


float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
int mainLoop = MAIN_INTERVAL;
bool metric = true;
static int16_t currentLevel = 0;  // Current dim level... //LED
uint32_t prevMillis;


#define FADE_DELAY 10  // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim) LED

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage lightMsg(CHILD_ID_LED, V_LIGHT); // LED
//MyMessage lightMsg2(CHILD_ID_LED2, V_LIGHT); // LED2

DHT dht;


void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("TemperatureAndHumidity", "1.1");
  
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);

  present( CHILD_ID_LED, S_LIGHT );
  //present( CHILD_ID_LED2, S_LIGHT );
  
  metric = getControllerConfig().isMetric;
}


void setup()
{ 
   int error;

  pinMode(LED_PIN, OUTPUT);       // sets the pin as output 
  //pinMode(LED_PIN2, OUTPUT);       // sets the pin as output  led2

  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  lcd.begin(16, 2); // initialize the lcd
  lcd.setBacklight(255);
  lcd.home(); 
  lcd.clear();

  
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
  //request( CHILD_ID_LED2, V_LIGHT ); //LED
}

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

      lcd.setCursor(0, 0);
      lcd.print("Alarm:          ");
      lcd.setCursor(7, 0);
      lcd.print(requestedLevel ? "On" : "Off" );
    }
   
    // if ((message.sensor == CHILD_ID_LED2)) //is the message for this node
    // {
    //   Serial.println( "Red LED" );
    //   Serial.print( "Changing level to " );
    //   Serial.println( requestedLevel );
    //   digitalWrite(LED_PIN2, requestedLevel > 0 ? HIGH : LOW);
    // }
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

void loop()      
{ 
  if (millis() - prevMillis >= UPDATE_INTERVAL) 
  {
    Serial.print("Slow Loop: millis: ");
    Serial.println(prevMillis);
    prevMillis += UPDATE_INTERVAL;
  
    Serial.print("Get temps: ");
    Serial.println(mainLoop);
    // Force reading sensor, so it works also after sleep()
    dht.readSensor(true);
    //Serial.println(dht.getStatusString());
    
    
    // Get temperature from DHT library
    float temperature = dht.getTemperature();
    if (isnan(temperature)) 
    {
      Serial.println("Failed reading temperature from DHT!");
      lcd.setCursor(0, 1);
      lcd.print("No DHT reading");
    } 
    else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) 
    {
      // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
      lastTemp = temperature;
      if (!metric) 
      {
        temperature = dht.toFahrenheit(temperature);
      }
      // Reset no updates counter
      nNoUpdatesTemp = 0;
      temperature += SENSOR_TEMP_OFFSET;
      send(msgTemp.set(temperature, 1));
  
      #ifdef MY_DEBUG
      Serial.print("T: ");
      Serial.println(temperature);
      #endif
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

    lcd.setCursor(0, 1);
    lcd.print("H:");
    lcd.print(humidity);
    lcd.print(" T:");
    lcd.print(temperature);
      
    } 
    else 
    {
      // Increase no update counter if the humidity stayed the same
      nNoUpdatesHum++;
    }
  }
  // Sleep for a while to save energy
 // sleep(UPDATE_INTERVAL); 
}
