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
#define MY_NODE_ID 59//54 // Sets a static id for a node
#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power

// Child ids
#define CHILD_ID_RELAY 0
#define CHILD_ID_FANSPEED 1
#define CHILD_ID_TEMP 2 // and onwards
#define CHILD_ID_TEMPDIFF 4 // temperature difference
#define GW_ID 0 //Gateway node adr


// Pin Assingment
#define PWM_PIN 3
//#define TACHO_PIN 2    // Speed feedback
#define RELAY_PIN 5    // Arduino pin attached to MOSFET Gate pin //analog pwm pin
#define ONE_WIRE_BUS 2 // Pin where dallase sensor is connected 

#define MAX_ATTACHED_DS18B20 2

#define SLEEP_TIME 3000 //60000;//3000; // Sleep time between reads (in milliseconds)

#define PWM_DUTY_MAX 79 // PWM output max DUETY value
#define TEMP_DIFF_FACTOR 10 // Temperature diffence to percentage conversion factor

#define TEMP_DIFF_RELAY_ON  0.5 // Temperature diffence to turn on output relay
#define TEMP_DIFF_RELAY_OFF 0.3 // Temperature diffence to turn off output relay

#define RUNTIME 120000
#define STOPTIME 120000

#define RELAY_ON  LOW //turn on relay value
#define RELAY_OFF HIGH //turn on relay value


#include <MySensors.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>


OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
uint32_t prevMillis;
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
boolean receivedConfig = false;
boolean relay = RELAY_OFF;
int pct = 0;
int oldpct = 0;
boolean controllerRelay = true;
uint32_t runtime = RUNTIME;
uint32_t stoptime = STOPTIME;


// Initialize temperature message
MyMessage msg(CHILD_ID_TEMP,V_TEMP); //temperature
MyMessage lightMsg(CHILD_ID_RELAY, V_LIGHT); // Relay for fan
MyMessage fanSpeedMsg(CHILD_ID_FANSPEED, V_PERCENTAGE); //Fanspeed in pct


void setup()  
{ 
  sensors.setResolution(TEMP_12_BIT); // Genauigkeit auf 12-Bit setzen
  //Serial.print( "Relay Setup");
  pinMode(RELAY_PIN, OUTPUT);       // sets the pin as output 
  digitalWrite(RELAY_PIN,relay); //turn off relay 
  //Serial.println( "Done");
  pinMode(PWM_PIN, OUTPUT);
  pwm25kHzBegin();
}


void presentation()
{
  // Startup up the OneWire library
  sensors.begin();
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);
  
  // Startup and initialize MySensors library. Set callback for incoming messages. 
  
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Temperature controlled Fan", "2.1");
  // Present the Fan Relay On/Off to controller

  
  Serial.println( "present( CHILD_ID_RELAY, S_LIGHT );");
  present( CHILD_ID_RELAY, S_LIGHT );

  Serial.println( "present( CHILD_ID_FANSPEED, S_DIMMER );");
  present( CHILD_ID_FANSPEED, S_DIMMER );

  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();
  Serial.print( "Number of DS18b20 found: ");
  Serial.println(numSensors);
  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) 
  {   
    Serial.print( "present(");
    Serial.print(i+CHILD_ID_TEMP);
    Serial.println( ", S_TEMP);");
     present(numSensors+CHILD_ID_TEMP, S_TEMP);
  }
  present(CHILD_ID_TEMPDIFF, S_TEMP);
  Serial.println( "void request Controller Relay state");
  //Request Master Relay state from controller
  request (CHILD_ID_RELAY, V_LIGHT,GW_ID);
}


/*
 * Receive messages from controller
 */
void receive(const MyMessage &message)
{
 // Serial.print( "Received message" );

  if (message.type == V_LIGHT || message.type == V_DIMMER)
  {
    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );

    // Adjust incoming level if this is a V_LIGHT variable update [0 == off, 1 == on]
    requestedLevel *= ( message.type == V_LIGHT ? 1 : 0 );

    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;

    if (message.sensor == CHILD_ID_RELAY) //is the message for this node
    {
     // Serial.print( "Controller relay: " );
     // Serial.println( requestedLevel );
      controllerRelay = (requestedLevel == 0 ? 0:1);
      digitalWrite( RELAY_PIN, controllerRelay);
    }
  }
}

void loop()     
{         
  byte duty = 0;

//  relay = RELAY_ON;
//  digitalWrite(RELAY_PIN,relay); //turn on relay 
//  float tempdiff1 = abs(21.2 - 19.6);
//  Serial.print( "Tempdiff: " );
//  Serial.print( tempdiff1 );  
//  int pct = (tempdiff1 * TEMP_DIFF_FACTOR);
//  Serial.print( " - Percentage: " );
//  Serial.println( pct );  
//  duty = pctToDuty((unsigned char)pct);
//  pwmDuty(duty); // range = 0-79 = 1.25-100%)
//   
    
  //Serial.println( "loop" );
  if (millis() - prevMillis >= SLEEP_TIME) 
  {
    //Serial.print("Slow Loop: millis: ");
    //Serial.println(prevMillis);
    prevMillis += SLEEP_TIME;
  
    // Fetch temperatures from Dallas sensors
    sensors.requestTemperatures();
  
    // Read temperatures and send them to controller 
    for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) 
    {
     // Fetch and round temperature to one decimal
     float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
     
      // Only send data if temperature has changed and no error
      if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) 
      {
        // Send in the new temperature
        send(msg.setSensor(i).set(temperature,2));
        // Save new temperatures for next compare
        lastTemperature[i]=temperature;
      }
    }//forloop end
    
    //Calculate temp diff between the first two probes
    float tempdiff = abs(lastTemperature[0] - lastTemperature[1]);  
    // Convert it to percentage for the fan control
    pct = (tempdiff * TEMP_DIFF_FACTOR);
    if (relay == RELAY_OFF)
      pct = 0;

    if (tempdiff > TEMP_DIFF_RELAY_ON) 
    {
//      Serial.print( "TEMP_DIFF_RELAY_ON: " );
//      Serial.print("Stoptime: ");
//      Serial.print(stoptime);

      if (stoptime >= STOPTIME)
      {
        stoptime = 0;
        relay = RELAY_ON;
//        Serial.print( " - stoptime >= STOPTIME");
//        Serial.println( " - Turn on relay" );
      }
//      else
//      {
//        Serial.println( " - stoptime < STOPTIME");
//      }
    }
    
    if (tempdiff < TEMP_DIFF_RELAY_OFF)
    {
//      Serial.print( "TEMP_DIFF_RELAY_OFF: " );
//      Serial.print(" runtime: ");
//      Serial.print(runtime);

      if (runtime >= RUNTIME) 
      {
        runtime = 0;
        relay = RELAY_OFF;
//        Serial.print( " - runtime >= RUNTIME");
//        Serial.println( " - Turn off relay" );
      }
//      else
//      {
//        Serial.println( " - runtime < RUNTIME");
//      }
    }

    if (oldpct != pct)
    {
      oldpct = pct;
      Serial.print( "Calculate temp diff ");
      Serial.print(tempdiff);
      Serial.print(" - pct: ");
      Serial.print(pct);    


      Serial.print(" - Stoptime: ");
      Serial.print(stoptime);
      Serial.print(" - Runtime: ");
      Serial.println(runtime);
      send(fanSpeedMsg.set(pct));   //Send the fanspeed to the controller
      send(msg.setSensor(CHILD_ID_TEMPDIFF).set(tempdiff,2)); //Send the temperature difference to the controller
      duty = pctToDuty((unsigned char)pct);
      //Set the duty cycle to the fan
      pwmDuty(duty);
    }
    

    if (relay == RELAY_ON)
    {
      runtime += SLEEP_TIME;      
    }
    else
    {
      stoptime += SLEEP_TIME;      
    }
  }//SlowLoop End
  
    // only operate relay if controller allowed it
    if (controllerRelay == false)
    {
      //Serial.println( "Controller blocks relay" );
      digitalWrite( RELAY_PIN, RELAY_OFF);
    }
    else
    {
      digitalWrite( RELAY_PIN, relay); //Write to relay 
    }
  //sleep(SLEEP_TIME);
}

/*
 * Setup pwm to use 25khz carrier wave for the 4pin PWM fan control
 * https://forum.arduino.cc/index.php?topic=415167.0
 */
void pwm25kHzBegin() 
{
  TCCR2A = 0;                               // TC2 Control Register A
  TCCR2B = 0;                               // TC2 Control Register B
  TIMSK2 = 0;                               // TC2 Interrupt Mask Register
  TIFR2 = 0;                                // TC2 Interrupt Flag Register
  TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // OC2B cleared/set on match when up/down counting, fast PWM
  TCCR2B |= (1 << WGM22) | (1 << CS21);     // prescaler 8
  OCR2A = 79;                               // TOP overflow value (Hz)
  OCR2B = 0;
}


/*
 * Set PWM duty cycle value to output pin
 *  // range = 0-79 = 1.25-100%)
 */
void pwmDuty(byte ocrb) 
{
  OCR2B = ocrb;                             // PWM Width (duty)
}


/*
 * Convert PercentValue to PWM Dutycycle value
 */
byte pctToDuty(unsigned char pct)
{
  if (pct > 100)
  {
    pct = 100; //Clamp to max value
  }

  int i = 0;
  i = ((pct * PWM_DUTY_MAX)/100);
  if (i > PWM_DUTY_MAX)
  {
    i = PWM_DUTY_MAX; //Clamp to max value
  }
    
//  Serial.print( "pctToDuty pct: " );
//  Serial.print( pct );
//  Serial.print( " duty: " );
//  Serial.println( i );
  return (byte)i;
}

