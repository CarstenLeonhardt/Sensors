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
// Promini 3.3V switch on 2 and 3, purple pcb, battery powered

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
#define MY_NODE_ID 49 // Sets a static id for a node 

#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>

#define CHILD_ID_MOMENTARY 3
#define BUTTON_PIN_MOMENTARY  3  // Arduino Digital I/O pin for button/reed switch

#define CHILD_ID_SW 2
#define BUTTON_PIN_SW 2  // Arduino Digital I/O pin for button/reed switch

Bounce debouncerA = Bounce(); 
int oldValueA=-1;
int ValueA = 0;

Bounce debouncerB = Bounce(); 
int oldValueB=-1;
int ValueB = 0;

MyMessage msgA(CHILD_ID_MOMENTARY, V_TRIPPED);
MyMessage msgB(CHILD_ID_SW, V_TRIPPED);
// Change to V_LIGHT if you use S_LIGHT in presentation below


//Retransmit present if it was not acked
void represent(uint8_t childSensorId, uint8_t sensorType, const char *description, int repeats)
{
  int repeat = 1;
  int repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) 
  {
    if (present(childSensorId, sensorType, description, false)) 
    {
      sendOK = true;
    } 
    else 
    {
      sendOK = false;
      Serial.print(F("Unable to transmit present(), retry count: "));
      Serial.println(repeat);
      repeatdelay += 250;
    } 
    repeat++; 
    delay(repeatdelay);
  }
}

void setup()  
{  
  // Setup the button
  pinMode(BUTTON_PIN_MOMENTARY,INPUT);
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN_MOMENTARY,HIGH);
  
  // After setting up the button, setup debouncer
  debouncerA.attach(BUTTON_PIN_MOMENTARY);
  debouncerA.interval(5);
  
  pinMode(BUTTON_PIN_SW,INPUT);
  digitalWrite(BUTTON_PIN_SW,HIGH);
  debouncerB.attach(BUTTON_PIN_SW);
  debouncerB.interval(5);
  
}

void presentation() 
{

  sendSketchInfo("Light Switch Status", "1.0");
  Serial.println(F("Presentation() "));
  delay(1000);
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  represent(CHILD_ID_MOMENTARY, S_DOOR, "lightMomentary", 10);  

  represent(CHILD_ID_SW, S_DOOR, "lightSw", 10);  
  
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
}


//Retransmit message if it was not acked
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
      Serial.print(F("Unable to transmit, retry count: "));
      Serial.println(repeat);
      repeatdelay += 250;
    } 
    repeat++; 
    delay(repeatdelay);
  }
}


int GetPinState(Bounce &debouncer, int &oldValue, MyMessage &msg, int repeats, String pinname, bool Verbose)
{
  int value = 0;
  int retval = oldValue==HIGH ? 0 : 1;
  debouncer.update();
  // Get the update value
  value = debouncer.read();

  if (value != oldValue) 
  {
    if (Verbose)
    {
      Serial.print(F("Pin "));
      Serial.print(pinname);
      Serial.print(F(" state changed, was: "));
      Serial.print(oldValue);
      Serial.print(F(", is now: "));
      Serial.print(value);
    }
    // Send in the new value
    retval = value==HIGH ? 0 : 1;
    if (Verbose)
    {
      Serial.print(F(" - State is now: "));
      Serial.println(retval);
    }
    resend((msg.set(retval)), repeats);
    oldValue = value;
  }
  return retval;
}


//  Check if digital input has changed and send in new value
void loop() 
{

   ValueA = GetPinState(debouncerA, oldValueA, msgA, 10, F("Momentary"), true);
   ValueB = GetPinState(debouncerB, oldValueB, msgB, 10, F("Switch"), true);

} 
