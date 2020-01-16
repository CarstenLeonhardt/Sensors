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

#define CHILD_ID_MOMENTARY 3
#define BUTTON_PIN_MOMENTARY  3  // Arduino Digital I/O pin for button/reed switch

#define CHILD_ID_SW 2
#define BUTTON_PIN_SW 2  // Arduino Digital I/O pin for button/reed switch

uint8_t value1;
uint8_t value2;
uint8_t sentValue1=2;
uint8_t sentValue2=2;


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
  pinMode(BUTTON_PIN_MOMENTARY,INPUT_PULLUP);
  pinMode(BUTTON_PIN_SW,INPUT_PULLUP);
}

void presentation() 
{

  sendSketchInfo("Light Switch Status", "1.1");
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



uint8_t GetPinValue(uint8_t pin, uint8_t &sentValue, MyMessage &msg, int repeats, String pinname, bool Verbose )
{
  uint8_t value;
  uint8_t retval;

  value = digitalRead(pin);

  retval = value==HIGH ? 0 : 1;
  if (value != sentValue) 
  {
    if (Verbose)
    {
      Serial.print(F("Pin "));
      Serial.print(pinname);
      Serial.print(F(" state changed, was: "));
      Serial.print(sentValue==LOW);
      Serial.print(F(", is now: "));
      Serial.println(value==LOW);
    }
    // Value has changed from last tranmission, send the updated value
    resend((msg.set(retval)), repeats);
    sentValue = value;
  }
  return retval;
}


//  Check if digital input has changed and send in new value
void loop() 
{
  // Short delay to allow buttons to properly settle
  sleep(5);
  
  value1 = GetPinValue(BUTTON_PIN_MOMENTARY, sentValue1, msgA, 10, F("Momentary"), true);
  value2 = GetPinValue(BUTTON_PIN_SW, sentValue2, msgB, 10, F("Switch"), true);
  
  // Sleep until something happens with the sensor
  Serial.print(F("Going to sleep"));
  sleep(BUTTON_PIN_MOMENTARY-2, CHANGE, BUTTON_PIN_SW-2, CHANGE, 0);
} 
