/*
  basic state machine 2
 
  Modified to switch between states to write, read and clear EEPROM
 
 Demonstrates how to use a case statement to create a simple state machine.
 This code uses a potentiometer knob to select between 3 states.
 
 The circuit:
 * pot from analog in 0 to +5V
 * 10K resistor from analog in 0 to ground
 
 created 13 Apr 2010
 by Wendy Ju 
 modified from switchCase by Tom Igoe
 
 12 Sep 2018
 Modified to switch between states to write, read and clear EEPROM
 */

#define toneC    1911
#define toneC1    1804
#define toneD    1703
#define toneEb    1607
#define toneE    1517
#define toneF    1432
#define toneF1    1352
#define toneG    1276
#define toneAb    1204
#define toneA    1136
#define toneBb    1073
#define toneB    1012
#define tonec       955
#define tonec1      902
#define toned       851
#define toneeb      803
#define tonee       758
#define tonef       716
#define tonef1      676
#define toneg       638
#define toneab      602
#define tonea       568
#define tonebb      536
#define toneb       506
#define tonep       0 

int notes[] = {tonec, toneG, toneE, toneA, toneB, toneBb, toneA, toneG, tonee, toneg, tonea, tonef, toneg, tonee, tonec, toned, toneB};




#include <Wire.h>
#include "Adafruit_VCNL4010.h"


Adafruit_VCNL4010 vcnl;


#include <EEPROM.h>



int sensorPin = 1;    // select the input pin for the sensor
int ledPin = 13;  


int addr = 0;
byte value;
int softpotReading;
int note;

int noteDuration = 2000/8;
int speaker = 8;

int softpot = A1;
int btn = 2;

int val;



void setup() {
  // test connection of state change sensor
  pinMode(8, OUTPUT);
  pinMode(A2,INPUT);
 
  Serial.begin(9600);
  Serial.println("VCNL4010 test");

  if (! vcnl.begin()){
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("Found VCNL4010");
}

void loop() {
 
  int prox = vcnl.readAmbient();
  Serial.println(prox);
  
  if (prox >= 0 && prox <= 200){
    clearEEPROM();
    }
  
  
  if (prox>=200 && prox <=300){
     readEEPROM();
   }  
  else if(prox>300){
     writeEEPROM();    
   }
  delay(1000);
}


void clearEEPROM(){
    Serial.println("Clearing EEPROM");
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
      }
    Serial.println("EEPROM Cleared");
    addr = 0;
  }

void writeEEPROM(){
    softpotReading = analogRead(softpot);
    Serial.print("address= ");
    Serial.println(addr);
    Serial.print("Analog Reading = ");
    Serial.println(softpotReading);
 
    int val = analogRead(softpotReading) / 4;
    Serial.print("EEPROM Value = ");
    Serial.println(val);
    delay(500);  
    
    EEPROM.write(addr, val);
    addr = addr + 1;
  
    if (addr == EEPROM.length()) {
     addr = 0;
    }   
  }

void readEEPROM(){

    // initialize serial and wait for port to open:
     Serial.println("Reading from EEPROM");
     int endAddr = min(addr, EEPROM.length());
     for (int i = 0; i < endAddr; i++) {
      value = EEPROM.read(i);
      softpotReading = value;
      Serial.print("address= ");
      Serial.println (i);
      Serial.print("EEPROM value = " );
      Serial.println (value);
      note = map(value, 0, 255, 0, 17);
      tone(speaker, notes[note], noteDuration);
      delay(noteDuration);  
     }
  
  }
