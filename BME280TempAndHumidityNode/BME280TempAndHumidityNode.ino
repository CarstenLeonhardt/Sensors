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

   DESCRIPTION

   Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
   http://www.mysensors.org/build/temp
*/
//https://www.mysensors.org/download/sensor_api_20
//v2.1 Anpassung auf API 2.0 von JH
#define MY_RADIO_NRF24
#define MY_DEBUG    // Enables debug messages in the serial log 
#define MY_NODE_ID 47// Sets a static id for a node
//#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power
#define MY_REPEATER_FEATURE //enable package repeater

#include <MySensors.h>
#include <SPI.h>
#include <OneWire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

// Sleep time between sensor updates (in milliseconds)
static const uint64_t UPDATE_INTERVAL = 60000; //Ms

Adafruit_BME280 bme; // I2C
uint8_t BME280_i2caddr = 0x76;
uint32_t prevMillis;

float lastTemp;
float lastHum;
float lastBaro;

#define CHILD_ID_HUM  0
#define CHILD_ID_TEMP 1
#define CHILD_ID_BARO 2


MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgPress(CHILD_ID_BARO, V_PRESSURE);

void setup()
{
  Serial.begin(MY_BAUD_RATE);
  Wire.begin(); // Wire.begin(sda, scl)
  if (!bme.begin(BME280_i2caddr))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  // weather monitoring
  Serial.println("-- Weather Station Scenario --");
  Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  Serial.println("filter off");
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_1000);
  // suggested rate is 1/60Hz (1m)
  Serial.println("Send sketch info");
  sendSketchInfo("BME280 Sensor", "1.1");
  present(CHILD_ID_BARO, S_BARO, "Pressure", false);
  present(CHILD_ID_TEMP, S_TEMP, "Temperature", false);
  present(CHILD_ID_HUM, S_HUM, "Humidity", false);
  
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
      Serial.print("Curent repeat delay: ");
      Serial.println(repeatdelay);
    } 
    repeat++; 
    delay(repeatdelay);
  }
}


void loop()
{
  if (millis() - prevMillis >= UPDATE_INTERVAL)
  {
    prevMillis += UPDATE_INTERVAL;

    bme.takeForcedMeasurement();
    float HUM = bme.readHumidity();
    float TEMP = bme.readTemperature();
    float BARO = bme.readPressure() / 100;

    if (lastTemp != TEMP)
    {
      lastTemp = TEMP;
      Serial.print("Temp: ");
      Serial.print(TEMP);
      Serial.println(" C");
      //send(msgTemp.set(TEMP, 2));
      resend((msgTemp.set(TEMP, 2)), 20);
      delay(200);
    }

    if (lastHum != HUM)
    {
      Serial.print("Hum: ");
      Serial.print(HUM);
      Serial.println(" %");
      //send(msgHum.set(HUM, 2));
      resend((msgHum.set(HUM, 2)), 20);
      delay(200);      
    }

    if (lastBaro != BARO)
    {
      Serial.print("Baro: ");
      Serial.print(BARO);
      Serial.println(" Pa");
      //send(msgPress.set(BARO, 1));
      resend((msgPress.set(BARO, 1)), 20);
      delay(200);
    }
  }
}
