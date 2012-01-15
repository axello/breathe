/* 
  Sensor Reading
  Breate / Air Quality Egg workshop
  Nov. 2011 - Jan. 2012
  
  Equipment: Nanode, 
  Sensors: MG811 (CO2), MQ7 (CO), Sensirion SHT71 (Humidity & temp)
  
  by Axel Roest, 
  Created 12 Jan. 2012

  ToDo:
  We should cycle both gas sensor heaters using a powerfet or transistor. 
  Especially the MG-811 needs a duty cycle of about 30%
  
*/

// wire needed for SHT71
#include "Sensirion.h"

#define SensorCo2 0
#define SensorCO 1
#define ReadingLed 5
#define NanodeLed 6

// resistor values (appr.) for the ground resistors of the divider
#define RCo2  30000.0
#define RCO   5000.0
#define  R0_Co2 30000.0
#define  R0_CO -26400.0

#define dataPin  A4
#define clockPin A5


int co2raw;
int coraw;
float co2;
float co;

float temperature;
float humidity;
float dewpoint;

Sensirion tempSensor = Sensirion(dataPin, clockPin);

void setup() {
  
  pinMode(ReadingLed, OUTPUT);  // Set servo pin as an output pin
  pinMode(NanodeLed, OUTPUT);  // Set servo pin as an output pin
  // finished setup, light led
  digitalWrite(ReadingLed, HIGH);
  
  Serial.begin(19200);
}

/* see Excel calculation and trendline
  y (Rs/R0) = a * x^b
  
  ==> x = pow(y/a , 1/b)
  */
  
float ppm(float y, float a, float b)
{
  return pow(y/a,1/b);
}
void readGasValues(void)
{
  float RSco2, RSco;
  
  co2raw=analogRead(SensorCo2);
  RSco2 = (1024.0 * RCo2 / (float)co2raw ) - RCo2;        // calculate voltage-divider value
  // do exponential conversion 
  // TODO compensate for RH, T and stuff
  co2 = ppm(RSco2/R0_Co2, 5.1633, -0.35);                  // see Excel and/or Datasheet for a,b values
  coraw=analogRead(SensorCO);
  RSco = (1024.0 * RCO / (float)coraw ) - RCO;
  co = ppm(RSco/R0_CO, 464.43, -0.061);
  
}

void serialPrintGasValues(void)
{
  Serial.print("CO2:");
  serialPrintFloat(co2);
  Serial.print(" ppm, CO:");
  serialPrintFloat(co);
  Serial.println(" ppm");
}

void serialPrintHumidTempValues(void)
{
  Serial.print("Temperature: ");
  serialPrintFloat(temperature);
  Serial.print(" C, Humidity: ");
  serialPrintFloat(humidity);
  Serial.print(" %, Dewpoint: ");
  serialPrintFloat(dewpoint);
  Serial.println(" C");
}

void serialPrintFloat(float f){
  Serial.print((int)f);
  Serial.print(".");
  int decplace = (f - (int)f) * 100;
  Serial.print(abs(decplace));
}

void loop()
{
  digitalWrite(ReadingLed,HIGH);            // toggle measuring led

  tempSensor.measure(&temperature, &humidity, &dewpoint);
  serialPrintHumidTempValues();
  
  readGasValues();
  serialPrintGasValues();
  
  digitalWrite(ReadingLed,LOW);  
  delay(2000);
}

