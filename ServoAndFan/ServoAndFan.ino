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

/* HW: Arduino Nano + mosfet + 4pin fan + digital servo
 * Bedroom ceiling fan / airvent actuator
 * */
 

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
#define CHILD_ID_SERVO 10   // Id of the sensor child
#define GW_ID 0 //Gateway node adr

#define SERVO_MIN 4//8 // Fine tune your servos min. 0-180
#define SERVO_MAX 80//55 //120  // Fine tune your servos max. 0-180
#define DETACH_DELAY 1500 // Tune this to let your movement finish before detaching the servo
#define FADE_DELAY 50//75// 100  // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim) LED



// Pin Assingment 1
#define ONE_WIRE_BUS 2 // Pin where dallase sensor is connected 
#define PWM_PIN 3 //Fan
#define SERVO_DIGITAL_OUT_PIN 6 //Servo
#define MOSFET_PIN 5    // Arduino pin attached to MOSFET Gate pin //analog pwm pin

#define MAX_ATTACHED_DS18B20 2

#define SLEEP_TIME 3000 //60000;//3000; // Sleep time between reads (in milliseconds)

#define PWM_DUTY_MAX 79 // PWM output max DUETY value


#define RUNTIME 120000
#define STOPTIME 120000

#define MOSFET_ON  HIGH
#define MOSFET_OFF LOW


#include <MySensors.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Servo.h>


MyMessage servoMsg(CHILD_ID_SERVO, V_DIMMER);
Servo myservo;  // create servo object to control a servo
// a maximum of eight servo objects can be created Sensor gw(9,10);
unsigned long timeOfLastChange = 0;
bool attachedServo = false;
static int16_t currentLevel = SERVO_MIN;  // Current dim level... //LED

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
uint32_t prevMillis;
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
boolean receivedConfig = false;
boolean mosfet = MOSFET_OFF;
boolean controllerRelay = true;
int pct = 0;
int oldpct = 0;
int fanpct = 0;
int fanpctStored = 0;


// Initialize temperature message
//MyMessage tempMsg(CHILD_ID_TEMP,V_TEMP); //temperature
MyMessage lightMsg(CHILD_ID_RELAY, V_LIGHT); // Relay for fan
MyMessage fanSpeedMsg(CHILD_ID_FANSPEED, V_PERCENTAGE); //Fanspeed in pct


void setup()  
{ 
  sensors.setResolution(TEMP_12_BIT); // Genauigkeit auf 12-Bit setzen
  //Serial.print( "Relay Setup");
  pinMode(MOSFET_PIN, OUTPUT);       // sets the pin as output 
  digitalWrite(MOSFET_PIN,mosfet); //turn off relay 
  //Serial.println( "Done");
  pinMode(PWM_PIN, OUTPUT);
  pwm25kHzBegin();

  // Request last servo state at startup
  request(CHILD_ID_SERVO, V_DIMMER);
  //Set servo to 0 degress
  myservo.write(currentLevel);
  myservo.attach(SERVO_DIGITAL_OUT_PIN);
  attachedServo = true;
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

  present(CHILD_ID_SERVO, S_COVER, "Servo", false);

  
  Serial.println( "present( CHILD_ID_RELAY, S_LIGHT );");
  present( CHILD_ID_RELAY, S_LIGHT , "FanRelay", false);

  Serial.println( "present( CHILD_ID_FANSPEED, S_DIMMER );");
  present( CHILD_ID_FANSPEED, S_DIMMER, "FanSpeed", false );

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
     present(numSensors+CHILD_ID_TEMP, S_TEMP, "TempSensor", false);
  }
  present(CHILD_ID_TEMPDIFF, S_TEMP, "TempDiff", false);
  Serial.println( "request Controller Relay state");
  //Request Master Relay state from controller
  request (CHILD_ID_RELAY, V_LIGHT);
  Serial.println( "request Fanspeed");  
  request (CHILD_ID_FANSPEED, V_DIMMER);
  request (CHILD_ID_FANSPEED, V_STATUS);
  Serial.println( "request Servo thing");    
  request (CHILD_ID_SERVO, S_COVER);
}


/*
 * Receive messages from controller
 */
void receive(const MyMessage &message)
{
  if (message.type == V_STATUS || message.type == V_DIMMER)
  {
    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );

    // Adjust incoming level if this is a V_STATUS variable update [0 == off, 1 == on]
    requestedLevel *= ( message.type == V_STATUS ? 1 : 0 );

    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;


    if (message.sensor == CHILD_ID_FANSPEED) //is the message for this node
    {
      fanpct = atoi( message.data );
      if (fanpct > 1)
        fanpctStored = fanpct;
      Serial.print( "Received Fan speed: " );
      Serial.println( fanpct );      
      Serial.print( "Fan speed: " );
      if (message.type == V_DIMMER)
      {
        Serial.print( "V_DIMMER: " );
      }
      else if (message.type == V_STATUS)
      {
        Serial.print( "V_STATUS: " );
        if (fanpct == 1)
        {
          //Restore fanspeed if turned on
          fanpct = fanpctStored;
        }
        fanState(fanpct > 1); 
      }
      
      // Serial.println( requestedLevel );
      Serial.print( ": Received Fan speed: " );
      Serial.println( fanpct );
    }

    if (message.sensor == CHILD_ID_RELAY) //is the message for this node
    {
      fanState(requestedLevel == 0);
    }
  }

  if (message.sensor == CHILD_ID_SERVO) //is the message for this node
  {
    Serial.println("Servo Attach");
    myservo.attach(SERVO_DIGITAL_OUT_PIN);
    if (message.type == V_DIMMER)
    {
      attachedServo = true;
      int val = message.getInt();
      Serial.print("Servo changed. new state: ");
      Serial.println(val);
      fanState(val >1);
    }
    else if (message.type == V_UP)
    {
      // Close the vent
      Serial.println("Servo UP command");
      Serial.println("Close the vent");
      //myservo.write(SERVO_MIN);
      fanState(false);
      fadeToLevel(SERVO_MIN);
      send(servoMsg.set(100));
    }
    else if (message.type == V_DOWN)
    {
      // Open the vent
      Serial.println("Servo DOWN command");
      Serial.println("Open the vent");      
      //myservo.write(SERVO_MAX);
      fadeToLevel(SERVO_MAX);
      send(servoMsg.set(0));
      fanState(true);
    }
    else if (message.type == V_STOP)
    {
      Serial.println("Servo STOP command");
      myservo.detach();
      attachedServo = false;
      fanState(false);  
    }
  }
  digitalWrite( MOSFET_PIN, controllerRelay); //Set Relay/Mosfet status
  timeOfLastChange = millis();
}

void fanState(bool state)
{
  controllerRelay = state;
  if (controllerRelay == false)
    fanpct = 0;
  else
    fanpct = fanpctStored;
  digitalWrite( MOSFET_PIN, controllerRelay);
  send(fanSpeedMsg.set(fanpct));   //Send the fanspeed to the controller
  
  Serial.print( "Fan mosfet: " );
  Serial.println( controllerRelay );

}

void loop()     
{         
  byte duty = 0;
  //Serial.println( "loop" );
  if (millis() - prevMillis >= SLEEP_TIME) 
  {
    //Serial.print("Slow Loop: millis: ");
    //Serial.println(prevMillis);
    prevMillis += SLEEP_TIME;
  
    if (oldpct != fanpct)
    {
      oldpct = fanpct;
      unsigned char pct = (unsigned char)fanpct;
      Serial.print("fanspeed Pct: ");
      Serial.println(fanpct);    
      send(fanSpeedMsg.set(fanpct));   //Send the fanspeed to the controller
      duty = pctToDuty((unsigned char)pct);
      //Set the duty cycle to the fan
      pwmDuty(duty);
    }
  }//SlowLoop End

  if (attachedServo && millis() - timeOfLastChange > DETACH_DELAY)
  {
    Serial.println("Servo Detach");
    myservo.detach();
    attachedServo = false;
  }
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
