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
 * Simple binary switch example 
 * Connect button or door/window reed switch between 
 * digitial I/O pin 3 (BUTTON_PIN below) and GND.
 * http://www.mysensors.org/build/binary
 */


// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
#define MY_NODE_ID 23 // Sets a static id for a node //23 BasementDoorLocked
#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power

#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>


#define CHILD_ID 3
#define BUTTON_PIN  3  // Arduino Digital I/O pin for button/reed switch
#define INDUCTIVE_PIN  1  // Arduino Analog input pin for button/reed switch

Bounce debouncer = Bounce(); 
int oldValue=-1;

float metalDetected;
int monitoring;
int metalDetection = 1;


// Change to V_LIGHT if you use S_LIGHT in presentation below
//MyMessage msg(CHILD_ID, V_TRIPPED); //original
MyMessage msg(CHILD_ID, V_ARMED);


void setup()  
{  
  // Setup the button
  pinMode(BUTTON_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN,HIGH);
  
  // After setting up the button, setup debouncer
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5);
  
}

void presentation() 
{
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  present(CHILD_ID, S_DOOR);  
}


//  Check if digital input has changed and send in new value
void loop() 
{
  debouncer.update();
  // Get the update value
  int value = debouncer.read();
 
  if (value != oldValue) 
  {
     Serial.print("Pin state changed, was: ");
     Serial.print(oldValue);
     Serial.print(", is now: ");
     Serial.println(value);
   
     // Send in the new value
     send(msg.set(value==HIGH ? 0 : 1));
     oldValue = value;
  }

  //monitoring = analogRead(metalDetection);
  monitoring = analogRead(A1);
  metalDetected = (float) monitoring*100/1024.0;
//  Serial.print("14CORE METAL DETECTOR TEST");
//  delay(500);
//  Serial.print("Initializing Proximity Sensor");
//  delay(500);
//  Serial.print("Please wait...");
//  delay(1000);
  Serial.print("Metal is Proximited = ");
  Serial.print(metalDetected);
  Serial.print("% - ");
  Serial.print(monitoring);
  Serial.println(" Adc");
  if (monitoring > 250)
    Serial.println("Metal is Detected");
  delay(1000);

  
} 

