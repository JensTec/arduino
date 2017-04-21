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
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * Example sketch showing how to control physical relays.
 * This example will remember relay state after power failure.
 * http://www.mysensors.org/build/relay
 */

// Enable debug prints
#define MY_DEBUG

#define MY_RADIO_NRF24
#define MY_NODE_ID 10

#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>

// Battery level 
int ANALOG_INPUT_BATT = A0;             // select the input pin for the battery sense point
#define CHILD_ID_BATT 5

// Motion sensor
unsigned long SLEEP_TIME = 450000;      // Sleep time between reports (in milliseconds)
#define DIGITAL_INPUT_SENSOR_MOTION 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define CHILD_ID_MOTION 1               // Id of the sensor child

// Light sensor
#define CHILD_ID_LIGHT 2
BH1750 lightSensor;

// Temp/Hum DHT11 sensor
#define DIGITAL_INPUT_DHT11 2
#define CHILD_ID_HUM 3
#define CHILD_ID_TEMP 4
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
// static const uint8_t FORCE_UPDATE_N_READS = 5;

int oldBatteryPcnt = 0;
int wake = 0;
int motionCount = 0;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgMotion(CHILD_ID_MOTION, V_TRIPPED);
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgBatt(CHILD_ID_BATT, V_PERCENTAGE); 
DHT dht;


void setup()  
{  
  pinMode(DIGITAL_INPUT_SENSOR_MOTION, INPUT);      // sets the motion sensor digital pin as input
  dht.setup(DIGITAL_INPUT_DHT11);                   // set data pin of DHT sensor
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());
  lightSensor.begin();
  // set to default so ADC counts with 3.3V
  analogReference(DEFAULT);                         
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Sensor Block SJ", "1.0"); 
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_MOTION, S_MOTION);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_BATT, S_DIMMER);
}

void loop()
{
  #ifdef MY_DEBUG
    Serial.print("motionCount: ");
    Serial.println(motionCount);  
  #endif
  if (wake == 1 && motionCount <= 3) {  // set maximum count to 3, so maximum time for no sensor readings is 30 minutes. Sleep time set to 450secs
    motion();
    motionCount = motionCount + 1;
  }
  else {
    motion();
    sensors();
    motionCount = 0;
  }
  // Sleep until interrupt comes in on motion sensor. 
  wake = sleep(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR_MOTION), CHANGE, SLEEP_TIME);
}

void sensors()     
{
  // BATTERY
  // get the battery Voltage
  int sensorValue = analogRead(ANALOG_INPUT_BATT);
 
  // 1M, 470K divider across battery and using internal ADC ref of 3.3V
  // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
  // ((1e6+470e3)/470e3)*5 = Vmax = 1.6 Volts
  // 3.3/1023 = Volts per bit = 0.00322581

  int batteryPcnt = sensorValue / 4.96;   // max output of 5V battery is 1.6V in this setup with R1 and R2. 100% is 496 out of 1023.

  #ifdef MY_DEBUG
    float batteryV  = sensorValue * 0.010080645;     // 0.010080645 x 496 = 5 V
    Serial.print("Sensor Value: ");
    Serial.print(sensorValue);
    Serial.println(" units");
    
    Serial.print("Battery Voltage: ");
    Serial.print(batteryV);
    Serial.println(" V");

    Serial.print("Battery percent: ");
    Serial.print(batteryPcnt);
    Serial.println(" %");
  #endif

  send(msgBatt.set(batteryPcnt));
  
  // TEMP 
  // Force reading sensor, so it works also after sleep()
  dht.readSensor(true);

  // Get temperature from DHT library
  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT!");
  } else {
    temperature += SENSOR_TEMP_OFFSET;
    send(msgTemp.set(temperature, 1));
  
    #ifdef MY_DEBUG
      Serial.print("T: ");
      Serial.println(temperature);
    #endif
  }
  
  // HUMIDITY
  // Get humidity from DHT library
  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } else {
    send(msgHum.set(humidity, 1));
    #ifdef MY_DEBUG
      Serial.print("H: ");
      Serial.println(humidity);
    #endif  
  }

  // LIGHT
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  #ifdef MY_DEBUG
    Serial.print("Light:");
    Serial.println(lux);
  #endif  
  send(msgLight.set(lux));
}

void motion()
{
  // Read digital motion value
  boolean tripped = digitalRead(DIGITAL_INPUT_SENSOR_MOTION) == HIGH; 
  #ifdef MY_DEBUG
    Serial.print("Motion:");
    Serial.println(tripped);
  #endif
  send(msgMotion.set(tripped?"1":"0"));  // Send tripped value to gw 
  if (tripped == 0) {
    motionCount = motionCount - 1;
  } 
}


