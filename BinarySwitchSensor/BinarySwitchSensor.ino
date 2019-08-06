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
#define MY_NODE_ID 22 // Sets a static id for a node //22 AlarmEnabledSensor
#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power

//Enable Repeater function
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>


#define CHILD_ARMED_ID 3
#define CHILD_SIREN_ID 4
#define ALARM_ARMED_PIN  3  // Alarm Armed Activated
#define ALARM_SIREN_PIN  4  // Alarm Siren Activated

//Send direct node messages
#define ALARM_NODE_1_ID 49 //desktop node
#define CHILD_ID_LED 2


Bounce debouncerA = Bounce(); 
Bounce debouncerS = Bounce(); 
int oldValueA=-1;
int oldValueS=-1;
uint32_t prevMillis;
int ValueA = 0;
int ValueS = 0;

// Sleep time between sensor updates (in milliseconds)
static const uint64_t UPDATE_INTERVAL = 60000;

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage msgA(CHILD_ARMED_ID, V_ARMED);
MyMessage msgS(CHILD_SIREN_ID, V_TRIPPED);


void setup()  
{  
  // Setup the button
  pinMode(ALARM_ARMED_PIN,INPUT);
  pinMode(ALARM_SIREN_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(ALARM_ARMED_PIN,HIGH);
  digitalWrite(ALARM_SIREN_PIN,HIGH);
  
  // After setting up the button, setup debouncer
  debouncerA.attach(ALARM_ARMED_PIN);
  debouncerA.interval(5);
  debouncerS.attach(ALARM_SIREN_PIN);
  debouncerS.interval(5);
  
}

void presentation() 
{
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  present(CHILD_ARMED_ID, S_DOOR);  
  present(CHILD_SIREN_ID, S_DOOR);  
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
      Serial.print("Unable to transmit, retry count: ");
      Serial.println(repeat);
      repeatdelay += 250;
    } 
    repeat++; 
    delay(repeatdelay);
  }
}

//  Check if digital input has changed and send in new value
void loop() 
{
  debouncerA.update();
  // Get the update value
  int value = debouncerA.read();
 
  if (value != oldValueA) 
  {
     Serial.print("Pin ARMED state changed, was: ");
     Serial.print(oldValueA);
     Serial.print(", is now: ");
     Serial.println(value);
   
     // Send in the new value
     ValueA = value==HIGH ? 0 : 1;
     Serial.print("ValueA is now: ");
     Serial.println(ValueA);
    
     resend((msgA.set(ValueA)), 10);
//     resend((msgA.set(value==HIGH ? 0 : 1)), 10);
     //send(msgA.set(value==HIGH ? 0 : 1));
     //Send data direct to sister node
//     Serial.println("Send data to sisteer node: ");
//     send(msgA.setDestination(ALARM_NODE_1_ID).setSensor(CHILD_ID_LED).set(value==HIGH ? 0 : 1));
     oldValueA = value;
  }

  debouncerS.update();
  // Get the update value
  value = debouncerS.read();
 
  if (value != oldValueS) 
  {
     delay(200);
     Serial.print("Pin SIREN state changed, was: ");
     Serial.print(oldValueS);
     Serial.print(", is now: ");
     Serial.println(value);
   
     // Send in the new value
     //send(msgS.set(value==HIGH ? 0 : 1));
     ValueS = value==HIGH ? 0 : 1;
     Serial.print("ValueS is now: ");
     Serial.println(ValueS);

     //resend((msgS.set(value==HIGH ? 0 : 1)), 10);
     resend((msgS.set(ValueS)), 10);
     oldValueS = value;
  }


  //Send digital input state every minute
  if (millis() - prevMillis >= UPDATE_INTERVAL) 
  {
      prevMillis += UPDATE_INTERVAL;
      Serial.print("Retransmit value Armed: ");
      Serial.println(ValueA);
      resend((msgS.set(ValueA)), 10);
      //resend((msgA.set(oldValueA==HIGH ? 0 : 1)), 10);
      //send(msgA.set(oldValueA==HIGH ? 0 : 1));

      delay(200);

      Serial.print("Retransmit value Siren: ");
      Serial.println(ValueS);
      resend((msgS.set(ValueS)), 10);
      //resend((msgS.set(oldValueS==HIGH ? 0 : 1)), 10);
      //send(msgS.set(oldValueS==HIGH ? 0 : 1));
  
    //  delay(200);
      
      //Send data direct to sister node
//      Serial.println("Send data to sisteer node: ");
//      send(msgA.setDestination(ALARM_NODE_1_ID).setSensor(CHILD_ID_LED).set(oldValueA==HIGH ? 0 : 1));
  }
} 
