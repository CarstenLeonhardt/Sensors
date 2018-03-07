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
   Version 1.0 - February 15, 2014 - Bruce Lacey
   Version 1.1 - August 13, 2014 - Converted to 1.4 (hek)
   Version 1.2 - 21-11-2017 - Used seperate inputs, for each output, adjusted timeout time

   DESCRIPTION
   This sketch provides a Dimmable LED Light using PWM and based Henrik Ekblad
   <henrik.ekblad@gmail.com> Vera Arduino Sensor project.
   Developed by Bruce Lacey, inspired by Hek's MySensor's example sketches.

   The circuit uses a MOSFET for Pulse-Wave-Modulation to dim the attached LED or LED strip.
   The MOSFET Gate pin is connected to Arduino pin 3 (LED_PIN), the MOSFET Drain pin is connected
   to the LED negative terminal and the MOSFET Source pin is connected to ground.

   This sketch is extensible to support more than one MOSFET/PWM dimmer per circuit.
   http://www.mysensors.org/build/dimmer
*/


/// LED Controller 1. floor cabinets



// Enable mysensor support
//#define MYSENSOR 1

// Enable serial port logging
#define LED_DIMMING_LOG_ENABLE 1// Log LED levels
#define DIGITAL_IN_LOG_ENABLE 1 // Log digital input state
//#define MY_DEBUG 1

#ifdef MYSENSOR
// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
// Sets a static id for a node
#define MY_NODE_ID 45
/////needs testing with radion on another pin
#define MY_RF24_CE_PIN 9//4    // Radio specific settings for RF24
// Include the MySensorLib
#include <MySensors.h>
//Name
#define SN "DimmableLED"
// Software Version
#define SV "1.1"
#endif

#define IN_PIN1 14     // A0 Arduino pin attached to magnetic sensor
#define IN_PIN2 15     // A1 Arduino pin attached to magnetic sensor
#define IN_PIN3 16     // A2 Arduino pin attached to magnetic sensor
#define IN_PIN4 17     // A3 Arduino pin attached to magnetic sensor

#define LED_PIN1 3     // Arduino pin attached to MOSFET Gate pin
#define LED_PIN2 5     // Arduino pin attached to MOSFET Gate pin
#define LED_PIN3 6     // Arduino pin attached to MOSFET Gate pin
#define LED_PIN4 9     //grr., used by mysensor for radio Arduino pin attached to MOSFET Gate pin

#define LED_COUNT 4    // number of LEDS/inputs

#define FADE_DELAY 5//10  // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim)

#define LED_MAX  100//255   // LED max light level
#define LED_MIN  0     // LED min light level

#define LED__STEP_UP  5   // LED steps whe dimming up
#define LED__STEP_DOWN  10 // LED steps whe dimming down


//led service interval
#define LED_LOOP_INTERVAL 100

// LED max on time ms
#define TIME_ON_MAX  10000 //1000 = 1m35s

#ifdef MYSENSOR
MyMessage dimmerMsg(0, V_DIMMER);
MyMessage lightMsg(0, V_LIGHT);
#endif

unsigned char ledPins [LED_COUNT] = {LED_PIN1, LED_PIN2, LED_PIN3, LED_PIN4}; // {LED_PIN1,LED_PIN2,LED_PIN3}; //
unsigned char inputPins [LED_COUNT] =  {IN_PIN1, IN_PIN2, IN_PIN3, IN_PIN4}; //{IN_PIN1,IN_PIN1,IN_PIN1};


enum states {
  S_OFF,
  S_ON,
  S_DimUp,
  S_DimDown,
  S_TimeOut,
  S_TimeOutDoneWaitForOff
};


struct Cabinet
{
  unsigned char ledPin;
  unsigned char inputPin;
  int16_t currentLevel = 0;
  int16_t ToLevel = 0;
  int timeOn = 0;
  unsigned char state = S_OFF;
};

Cabinet cab[LED_COUNT];

unsigned long currentMillis;
unsigned long previousMillis = 0;

/***
   Dimmable LED initialization method
*/
void setup()
{
  unsigned char i;

#ifdef MYSENSOR
  // Pull the gateway's current dim level - restore light level upon sendor node power-up
  request( 0, V_DIMMER );
#endif
  // Setup pins for led and door sensors
  for (i = 0; i < LED_COUNT; i++)
  {
    cab[i].inputPin  = inputPins[i];
    cab[i].ledPin = ledPins[i];
    pinMode(cab[i].inputPin,  INPUT_PULLUP); //set as input with pullup resister
    pinMode(cab[i].ledPin, OUTPUT);       // sets the pin as output
  }
}

void presentation()
{
#ifdef MYSENSOR
  // Register the LED Dimmable Light with the gateway
  present( 0, S_DIMMER );

  sendSketchInfo(SN, SV);
#endif
}

/***
    Dimmable LED main processing loop
*/
void loop()
{
  currentMillis = millis();
  //Serial.println( "Service leds " );
  ServiceInputs();

  // only serve the led sometimes every xx ms
  if (currentMillis - previousMillis >= LED_LOOP_INTERVAL)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    ServiceLeds();
  }
}


/*
   Service the Digital inputs

*/
void ServiceInputs(void)
{
  unsigned char i;
  for (i = 0; i < LED_COUNT; i++)
  {
    // Handle LED ON/OFF if Dorsensor state changes
    if ( (digitalRead(cab[i].inputPin) == HIGH) &&  (cab[i].state == S_OFF))
    {
#ifdef DIGITAL_IN_LOG_ENABLE
      Serial.print( "Door ");
      Serial.print( i );
      Serial.println( " open");
#endif
      cab[i].state = S_DimUp; //changing to state on
      cab[i].timeOn = TIME_ON_MAX; //Start the time
      cab[i].ToLevel = LED_MIN;
    }
    else if ( (digitalRead(cab[i].inputPin) == LOW) &&  ( (cab[i].state == S_ON) || (cab[i].state == S_TimeOutDoneWaitForOff)) )
    {
#ifdef DIGITAL_IN_LOG_ENABLE
      Serial.print( "Door ");
      Serial.print( i );
      Serial.print( " closed");
#endif
      if (cab[i].state == S_TimeOutDoneWaitForOff)
      {
#ifdef DIGITAL_IN_LOG_ENABLE
        Serial.print( " - timeout Off");
#endif
        cab[i].state = S_OFF; //changing to state off
      }
      else
      {
        cab[i].state = S_DimDown; //changing to state off
        cab[i].timeOn = 0; //Stop the time
        cab[i].ToLevel = LED_MAX;
      }
#ifdef DIGITAL_IN_LOG_ENABLE
      Serial.println( " "); // finish the line
#endif

    }
  }
}

/*
   Service the LED outputs

*/
void ServiceLeds(void)
{
  unsigned char i;
  for (i = 0; i < LED_COUNT; i++)
  {

    //If the time is up, turn off leds
    if ((cab[i].state == S_ON) && (cab[i].timeOn >= LED_LOOP_INTERVAL))
    {
      cab[i].timeOn -= 1;
    }
    else if ((cab[i].state == S_ON) && (cab[i].timeOn < LED_LOOP_INTERVAL))
    {
      cab[i].state = S_TimeOut;
      cab[i].timeOn = 0;
#ifdef LED_DIMMING_LOG_ENABLE
      Serial.print( "Channel: " );
      Serial.print( i );
      Serial.println( " - timeout, turn off " );
#endif
    }


    // Handle LED ON/OFF if Dorsensor state changes
    if ( cab[i].state == S_DimUp)
    {
      cab[i].ToLevel = LED_MAX;
#ifdef LED_DIMMING_LOG_ENABLE
      Serial.print( "Channel: " );
      Serial.print( i );
      Serial.print( " - Changing level to " );
      Serial.print( cab[i].ToLevel );
      Serial.print( ", from " );
      Serial.println( cab[i].currentLevel );
#endif
      fadeToLevel( cab[i].ToLevel ,  cab[i].ledPin, &cab[i].currentLevel);
      cab[i].state = S_ON;
      /*
        if (cab[i].ToLevel >= LED_MAX)
        cab[i].state = S_ON;
        else
        cab[i].ToLevel += LED__STEP_UP;
      */
    }
    else if ((cab[i].state == S_DimDown) || (cab[i].state == S_TimeOut))
    {
      cab[i].ToLevel = LED_MIN;
#ifdef LED_DIMMING_LOG_ENABLE
      Serial.print( "Channel: " );
      Serial.print( i );
      Serial.print( " - Changing level to " );
      Serial.print( cab[i].ToLevel );
      Serial.print( ", from " );
      Serial.println( cab[i].currentLevel );
#endif
      fadeToLevel( cab[i].ToLevel , cab[i].ledPin,  &cab[i].currentLevel);
      if (cab[i].state == S_DimDown)
      {
        cab[i].state = S_OFF;
      }
      else if (cab[i].state == S_TimeOut)
      {
        cab[i].state = S_TimeOutDoneWaitForOff;
      }

      /*
        if (cab[i].ToLevel <= LED_MIN)
        cab[i].state = S_OFF;
        else
        cab[i].ToLevel -= LED__STEP_DOWN;
      */
    }
  }
}

#ifdef MYSENSOR
void receive(const MyMessage &message)
{
  if (message.type == V_LIGHT || message.type == V_DIMMER) {

    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );

    // Adjust incoming level if this is a V_LIGHT variable update [0 == off, 1 == on]
    requestedLevel *= ( message.type == V_LIGHT ? 100 : 1 );

    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;

    Serial.print( "Changing level to " );
    Serial.print( requestedLevel );
    Serial.print( ", from " );
    //Serial.println( currentLevel );



    // Inform the gateway of the current DimmableLED's SwitchPower1 and LoadLevelStatus value...
    //    send(lightMsg.set(currentLevel > 0));

    // hek comment: Is this really nessesary?
    ///  send( dimmerMsg.set(currentLevel) );


  }
}
#endif
/***
    This method provides a graceful fade up/down effect
*/
void fadeToLevel( int toLevel, char channel, int* curLevel )
{
  int currentLevel = *curLevel;

  int delta = ( toLevel - currentLevel ) < 0 ? -1 : 1;


  while ( currentLevel != toLevel )
  {
    currentLevel += delta;
    analogWrite( channel, (int)(currentLevel / 100. * 255) );
    delay( FADE_DELAY );
  }
  *curLevel = currentLevel;

}
