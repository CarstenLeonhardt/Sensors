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

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <SPI.h>
#include <MySensors.h>
#include <Servo.h>

#define SERVO_DIGITAL_OUT_PIN 3
#define SERVO_MIN 8 // Fine tune your servos min. 0-180
#define SERVO_MAX 55 //120  // Fine tune your servos max. 0-180
#define DETACH_DELAY 1500 // Tune this to let your movement finish before detaching the servo
#define CHILD_ID 10   // Id of the sensor child
#define FADE_DELAY 50//75// 100  // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim) LED

MyMessage msg(CHILD_ID, V_DIMMER);
Servo myservo;  // create servo object to control a servo
// a maximum of eight servo objects can be created Sensor gw(9,10);
unsigned long timeOfLastChange = 0;
bool attachedServo = false;
static int16_t currentLevel = SERVO_MIN;  // Current dim level... //LED


void setup()
{
  // Request last servo state at startup
  request(CHILD_ID, V_DIMMER);

  //Set servo to 0 degress
  myservo.write(currentLevel);
  myservo.attach(SERVO_DIGITAL_OUT_PIN);
  attachedServo = true;

}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Servo", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID, S_COVER);
}

void loop()
{
  if (attachedServo && millis() - timeOfLastChange > DETACH_DELAY)
  {
    Serial.println("Servo Detach");
    myservo.detach();
    attachedServo = false;
  }
}

/***
    This method provides a graceful fade up/down effect
*/
void fadeToLevel( int toLevel )
{

  int delta = ( toLevel - currentLevel ) < 0 ? -1 : 1;
  Serial.print("Start fading from: ");
  Serial.print(currentLevel);
  Serial.print(" to: ");
  Serial.println(toLevel);
  while ( currentLevel != toLevel )
  {
    currentLevel += delta;
    myservo.write((int)(currentLevel));

    //analogWrite( LED_PIN, (int)(currentLevel / 100. * 255) );
    //Serial.println(currentLevel);
    delay( FADE_DELAY );
  }
  Serial.println("Done fading");
}

void receive(const MyMessage &message)
{
  Serial.println("Servo Attach");
  myservo.attach(SERVO_DIGITAL_OUT_PIN);
  attachedServo = true;

  if (message.type == V_DIMMER)
  { // This could be M_ACK_VARIABLE or M_SET_VARIABLE
    int val = message.getInt();
    //myservo.write(SERVO_MAX + (SERVO_MIN-SERVO_MAX)/100 * val); // sets the servo position 0-180
    //fadeToLevel(SERVO_MAX + (SERVO_MIN-SERVO_MAX)/100 * val);
    // Write some debug info
    Serial.print("Servo changed. new state: ");
    Serial.println(val);
  }
  else if (message.type == V_UP)
  {
    Serial.println("Servo UP command");
    //myservo.write(SERVO_MIN);
    fadeToLevel(SERVO_MIN);
    send(msg.set(100));
  }
  else if (message.type == V_DOWN)
  {
    Serial.println("Servo DOWN command");
    //myservo.write(SERVO_MAX);
    fadeToLevel(SERVO_MAX);
    send(msg.set(0));
  }
  else if (message.type == V_STOP)
  {
    Serial.println("Servo STOP command");
    myservo.detach();
    attachedServo = false;

  }
  timeOfLastChange = millis();
}



