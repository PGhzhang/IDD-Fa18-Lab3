# Data Logger (and using cool sensors!)

*A lab report by Hanyu Zhang.*

## In The Report

## Part A.  Writing to the Serial Monitor
 
**a. Based on the readings from the serial monitor, what is the range of the analog values being read?**

The analog values range from 0 - 1023.
 
**b. How many bits of resolution does the analog to digital converter (ADC) on the Arduino have?**

10 bit, because 2^10 = 1024.


## Part B. RGB LED

**How might you use this with only the parts in your kit? Show us your solution.**

I connected RGB LED pins according to the schema and edited the sample code to light it up. 

Code as follows

```
int redPin = 9;
int greenPin = 10;
int bluePin = 11;
int redVal;
int greenVal;
int blueVal;

int fsrReading;
int fsrmap;

void setup()
{
  Serial.begin(9600);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
}
 
void loop()
{
  fsrReading = analogRead(A0);
  fsrmap = map(fsrReading, 0,1023,0,255);
  Serial.println(fsrmap);

  if (fsrReading <= 85){
    setColor(0, 255, 0);  // green
    } 
  else if (fsrReading > 85 && fsrReading < 170){
    setColor(0, 0, 255);  // blue
    }
  else if(fsrReading >=170 && fsrReading <=255){
    setColor(255, 0, 0);  // red
    } 
  delay(1000); 
}

void setColor(int red, int green, int blue)
{
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
```

## Part C. Voltage Varying Sensors 
 
### 1. FSR, Flex Sensor, Photo cell, Softpot

**a. What voltage values do you see from your force sensor?**

The voltage value should range from 0-5V. I'm reading analog values.
Depending on how hard I press. When I press really hard, the analog value goes up to over 900 and when I press lightly, the analog value can be as low, for example 50.

**b. What kind of relationship does the voltage have as a function of the force applied? (e.g., linear?)**

According to [Datasheet](https://cdn-shop.adafruit.com/datasheets/FSR400Series_PD.pdf), the output voltage is propotional to the force applied, as indicated by the function as well as the graph. As the force applied increases, the FSR resistence decreases, the current flowing increases, resulting in larger voltage. 

![alt text](https://github.com/PGhzhang/IDD-Fa18-Lab3/blob/master/function.png)

![alt text](https://github.com/PGhzhang/IDD-Fa18-Lab3/blob/master/FSR%20voltage%20graph.png)


**c. In Examples->Basic->Fading the LED values range from 0-255. What do you have to do so that you get the full range of output voltages from the LED when using your FSR to change the LED color?**

I use analogRead to get analog values from FSR. Since analog values of FSR ranges from 0-1024, I map that number to a 0-255 scale. To change LED color, I set the rule that if fsrmap<85: turn green light on, if 85<fsr<170, turn blue light on, otherwise, red one.


**d. What resistance do you need to have in series to get a reasonable range of voltages from each sensor?**

I usde 10kOhm resister and it works fine.


**e. What kind of relationship does the resistance have as a function of stimulus? (e.g., linear?)**

resistence and voltages has a negative corelation

### 2. Accelerometer
 
**a. Include your accelerometer read-out code in your write-up.**

![alt text](https://github.com/PGhzhang/IDD-Fa18-Lab3/blob/master/acce%20serial%20output.png)
[Code Link](https://github.com/PGhzhang/IDD-Fa18-Lab3/blob/master/accel_Hanyu.ino)

Code
```
// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Basic demo for accelerometer readings from Adafruit LIS3DH
#include <Wire.h>

#include <Adafruit_LIS3DH.h>

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
}


void loop() {
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 

  Serial.println();

  //print X Y and Z data on LCD display

  lcd.setCursor(0, 0);
  lcd.print(lis.x);
  lcd.setCursor(4, 0);
  lcd.print(',');
  lcd.setCursor(6, 0);
  lcd.print(lis.y);
  lcd.setCursor(10, 0);
  lcd.print(',');
  lcd.setCursor(12, 0);
  lcd.print(lis.z);
 
  delay(3000); 
}
```


### 3. IR Proximity Sensor

**a. Describe the voltage change over the sensing range of the sensor. A sketch of voltage vs. distance would work also. Does it match up with what you expect from the datasheet?**

The voltage drops as you move away from the sensor and goes up when you move closer.
Proximity increase we you move towarsd and drops the other way.
Ambient decreases when there's object moving towards the sensor. 

**b. Upload your merged code to your lab report repository and link to it here.**

[Merged Code](https://github.com/PGhzhang/IDD-Fa18-Lab3/blob/master/merged_Hanyu.ino)

## Optional. Graphic Display

**Take a picture of your screen working insert it here!**

## Part D. Logging values to the EEPROM and reading them back
 
### 1. Reading and writing values to the Arduino EEPROM

**a. Does it matter what actions are assigned to which state? Why?**

Yes, the states are arranged in a way following the logical actions. When analog value =0, we would expect state resets by clearing up EEPROM values. On the other hand, if we assign case 0 to write, it would make no sense since we could only do limited sinces with reading/writing analog value 0.

**b. Why is the code here all in the setup() functions and not in the loop() functions?**
 
 Cases are written in loop() funcitons in SwitchState file, thus you don't need to put specific codes for each state in loop() functions.

**c. How many byte-sized data samples can you store on the Atmega328?**

1024 byte-sized data

**d. How would you get analog data from the Arduino analog pins to be byte-sized? How about analog data from the I2C devices?**
We can either map 0-1023 to 0-255 using map(analogRead(sensorPin), 0, 1023, 0, 255) or or simply divide analogRead values by 4.0. For I2C devices, we can map the max value of I2C device to 255 and the min value of I2C device to 0.

**e. Alternately, how would we store the data if it were bigger than a byte? (hint: take a look at the [EEPROMPut](https://www.arduino.cc/en/Reference/EEPROMPut) example)**

Split and store in different addresses using EEPROM.put()

**Upload your modified code that takes in analog values from your sensors and prints them back out to the Arduino Serial Monitor.**

[Code Link](https://github.com/PGhzhang/IDD-Fa18-Lab3/blob/master/SwitchState_Sensor.ino)


### 2. Design your logger
 
**a. Insert here a copy of your final state diagram.**


### 3. Create your data logger!
 
**a. Record and upload a short demo video of your logger in action.**

For my data logger, I used a softpot and an IR Proximity sensor. The device reads touches on softpot and translate into music notes. State change is dependent on the distance between your hand and IR Proximity sensor. You can trigger the music play by placing your hand close to IR Proximity sensor. Not too close though, because it will clear EEPROM memory otherwise.

[Code Link](https://github.com/PGhzhang/IDD-Fa18-Lab3/blob/master/data_logger_Hanyu.ino)

Video Demo

[![](http://img.youtube.com/vi/7lEf5IBylVU/0.jpg)](http://www.youtube.com/watch?v=7lEf5IBylVU "")


