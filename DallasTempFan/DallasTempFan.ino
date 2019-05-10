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
 * DESCRIPTION
 *
 * Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
 * http://www.mysensors.org/build/temp
 */
//https://www.mysensors.org/download/sensor_api_20
//v2.1 Anpassung auf API 2.0 von JH
#define MY_RADIO_NRF24
#define MY_DEBUG    // Enables debug messages in the serial log 
//#define MY_DEBUG_VERBOSE_RF24  //verbose debug
#define MY_NODE_ID 58//54 // Sets a static id for a node
//#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power
#include <MySensors.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>


#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define CHILD_ID_TEMP 1
#define CHILD_ID_RELAY 0
#define RELAY_PIN 4      // Arduino pin attached to MOSFET Gate pin //analog pwm pin
#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16
unsigned long SLEEP_TIME = 60000;//3000; // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

uint32_t prevMillis;
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
boolean receivedConfig = false;
boolean metric = true; 
boolean relay = HIGH; 
// Initialize temperature message
MyMessage msg(CHILD_ID_TEMP,V_TEMP);
MyMessage lightMsg(CHILD_ID_RELAY, V_LIGHT); // Relay for fan
void setup()  
{ 
  sensors.setResolution(TEMP_12_BIT); // Genauigkeit auf 12-Bit setzen
  Serial.print( "Relay Setup");
  pinMode(RELAY_PIN, OUTPUT);       // sets the pin as output 
  digitalWrite(RELAY_PIN,relay); //turn off relay 
  Serial.println( "Done");
}

void presentation()
{
  // Startup up the OneWire library
  sensors.begin();
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);
  
  // Startup and initialize MySensors library. Set callback for incoming messages. 

  
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Temperature Sensor", "2.1");

  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();
  Serial.print( "Number of DS18b20 found: ");
  Serial.println(numSensors);
  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) 
  {   
     present(i+1, S_TEMP);
  }
  present( CHILD_ID_RELAY, S_LIGHT );
}

//led
void receive(const MyMessage &message)
{
  if (message.type == V_LIGHT || message.type == V_DIMMER)
  {
    //Serial.println("Receive light message");

    //Serial.print( "Message Sensor: " );
    //Serial.println( message.sensor );
    
    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );

    // Adjust incoming level if this is a V_LIGHT variable update [0 == off, 1 == on]
    requestedLevel *= ( message.type == V_LIGHT ? 1 : 0 );

    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;

    //if (message.sensor == CHILD_ID_RELAY) //is the message for this node
    {
      Serial.print( "Relay: " );
      Serial.println( requestedLevel );
      relay = (requestedLevel == 0 ? 1:0);
      digitalWrite( RELAY_PIN, relay);
    }
  }
}

void loop()     
{           
  //Serial.println( "loop" );
 if (millis() - prevMillis >= SLEEP_TIME) 
 {
    //Serial.print("Slow Loop: millis: ");
    //Serial.println(prevMillis);
    prevMillis += SLEEP_TIME;
  
    // Fetch temperatures from Dallas sensors
    sensors.requestTemperatures();
  
    // query conversion time and sleep until conversion completed
    //int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
    // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
    //sleep(conversionTime);
  
    // Read temperatures and send them to controller 
    for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) 
    {
     // Fetch and round temperature to one decimal
     float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
  
   
      // Only send data if temperature has changed and no error
      #if COMPARE_TEMP == 1
      if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) 
      {
      #else
      if (temperature != -127.00 && temperature != 85.00) 
      {
      #endif
        // Send in the new temperature
        send(msg.setSensor(i).set(temperature,2));
        // Save new temperatures for next compare
        lastTemperature[i]=temperature;
      }
    }
     //Calculate temp diff between the first two probes
    float tempdiff = abs(lastTemperature[0] - lastTemperature[1]);  
    if (tempdiff > 1.5)
    {
      relay = LOW;
    }
    else if (tempdiff < 0.5)
    {
      relay = HIGH;
    }
    digitalWrite( RELAY_PIN, relay); //turn on
  }

  
  //sleep(SLEEP_TIME);
}

