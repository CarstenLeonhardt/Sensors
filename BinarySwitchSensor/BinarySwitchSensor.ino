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
 
/*
 * HW: Arduino promini 8mhz
 * 3 * digital input from Alarmsystem Armed / Siren active / Alarm completly armed
 */

// Enable debug prints to serial monitor
//#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power

#if defined(ARDUINO_AVR_NANO)
  //Only needed for Keywish NanoExpansionBoard with screw terminals
  #define MY_RF24_CE_PIN 10
  #define MY_RF24_CS_PIN 9
  #define MY_NODE_ID 24 // Sets a static id for a node 
  #define CHILD_ARMED_ID 1
  #define CHILD_SIREN_ID 2
  #define CHILD_ARMED_AWAY_ID 3 //Away
  #define CHILD_ARMED_HOME_ID 4 //Home
#elif defined(ARDUINO_AVR_PRO)
  // Pro Mini assignments
  #define CHILD_ARMED_ID 3
  #define CHILD_SIREN_ID 4
  #define CHILD_ARMED_AWAY_ID 5 //Away
  #define CHILD_ARMED_HOME_ID 6 //Home
  #define MY_NODE_ID 22 // Sets a static id for a node //22 AlarmEnabled
#else
  #error Unsupported board selection.
#endif



//Enable Repeater function
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>



#define ALARM_ARMED_PIN  3  // Alarm Armed Activated
#define ALARM_SIREN_PIN  4  // Alarm Siren Activated
#define ALARM_ARMED_COMPLETE_PIN  5  // Alarm Completly Armed Activated


Bounce debouncerA = Bounce(); 
Bounce debouncerS = Bounce(); 
Bounce debouncerC = Bounce(); 

int oldValueA=-1;
int oldValueS=-1;
int oldValueC=-1;
int oldValueP=-1;

int ValueA = 0;
int ValueS = 0;
int ValueC = 0;
int ValueP = 0;
uint32_t prevMillis;

// Sleep time between sensor updates (in milliseconds)
static const uint64_t UPDATE_INTERVAL = 60000;

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage msgA(CHILD_ARMED_ID, V_ARMED);
MyMessage msgS(CHILD_SIREN_ID, V_TRIPPED);
MyMessage msgC(CHILD_ARMED_AWAY_ID, V_ARMED);
MyMessage msgP(CHILD_ARMED_HOME_ID, V_ARMED);


void setup()  
{  
  // Setup the button
  pinMode(ALARM_ARMED_PIN,INPUT);
  pinMode(ALARM_SIREN_PIN,INPUT);
  pinMode(ALARM_ARMED_COMPLETE_PIN,INPUT);
  
  // Activate internal pull-up
  digitalWrite(ALARM_ARMED_PIN,HIGH);
  digitalWrite(ALARM_SIREN_PIN,HIGH);
  digitalWrite(ALARM_ARMED_COMPLETE_PIN,HIGH);
  
  // After setting up the button, setup debouncer
  debouncerA.attach(ALARM_ARMED_PIN);
  debouncerA.interval(100);
  debouncerS.attach(ALARM_SIREN_PIN);
  debouncerS.interval(100);  
  debouncerC.attach(ALARM_ARMED_COMPLETE_PIN);
  debouncerC.interval(100);
  
}
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

void presentation() 
{
  // Register binary input sensor to gw (they will be created as child devices)
  sendSketchInfo("FrontiAlarm Status", "1.7");
  Serial.println(F("Presentation() "));
  delay(1000);
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  represent(CHILD_ARMED_AWAY_ID, S_DOOR, "ArmedComplete", 10);  
  represent(CHILD_ARMED_HOME_ID, S_DOOR, "ArmedPartly", 10);  
  represent(CHILD_ARMED_ID, S_DOOR, "Armed", 10);  
  represent(CHILD_SIREN_ID, S_DOOR, "Siren", 10);  
  Serial.println(F("Presentation() Done "));
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

int GetArmedHome(int Armed, int ArmedAway, int &oldValue, MyMessage &msg, int repeats, String pinname, bool Verbose)
{
  int retval = 0;
   // Report if the alarm is Armed Home
  if (Armed == 0)
  {
    if  (ArmedAway == 0) 
    { //Armed Away activated alarm
      retval = 0;
    }
    else if (ArmedAway == 1) 
    { //Armed Home activated alarm
      retval = 1;
    }
  }
  else
  {
      retval = 0;
  }
  
  if (retval != oldValue)
  {  
    oldValue = retval;
    if (Verbose)
    {
      Serial.print(pinname);
      Serial.print(F(" Alarm Home Armed, was: "));
      Serial.print(oldValue);
      Serial.print(F(", is now: "));
      Serial.println(retval);
    }
    resend((msg.set(retval)), repeats);
  }
  return retval;
}

void ForcedUpdate(int txvalue, MyMessage &msg, int repeats, String pinname, bool Verbose)
{
  if (Verbose)
  {
    Serial.print(F("Retransmit value ")); 
    Serial.print(pinname);
    Serial.print(F(" : "));
    Serial.println(txvalue);
  }
  resend((msg.set(txvalue)), repeats);
  delay(200);
}

//  Check if digital input has changed and send in new value
void loop() 
{
  
  // Alarm Armed
  ValueA = GetPinState(debouncerA, oldValueA, msgA, 10, F("ARMED"), true);

  // Siren active
  ValueS = GetPinState(debouncerS, oldValueS, msgS, 10, F("SIREN"), true);

  // Alarm completly armed  - Armed Away
  ValueC = GetPinState(debouncerC, oldValueC, msgC, 10, F("ArmedAway"), true);

  // Report if the alarm is Partly Armed - Armed Home
  ValueP = GetArmedHome(oldValueA, oldValueC, oldValueP, msgP, 10, F("ArmedHome"), true);

  //Send digital input state every minute
  if (millis() - prevMillis >= UPDATE_INTERVAL) 
  {
      //Serial.print("ValueA is : ");
      //Serial.println(ValueA);
      //Serial.print("ValueC is : ");
      //Serial.println(ValueC);
      //Serial.print("ValueS is : ");
      //Serial.println(ValueS);
      //Serial.print("ValueP is : ");
      //Serial.println(ValueP);
      prevMillis += UPDATE_INTERVAL;
      //Serial.println(F("ForcedUpdate Armed"));
      ForcedUpdate(ValueA, msgA, 10, "Armed" , true);
      //Serial.println(F("ForcedUpdate Away"));
      ForcedUpdate(ValueC, msgC, 10, "Alarm Away Armed" , true);
      //Serial.println(F("ForcedUpdate Siren"));
      ForcedUpdate(ValueS, msgS, 10, "Siren" , true);
      //Serial.println(F("ForcedUpdate Home"));
      ForcedUpdate(ValueP, msgP, 10, "Alarm Home Armed" , true);
  }
} 
