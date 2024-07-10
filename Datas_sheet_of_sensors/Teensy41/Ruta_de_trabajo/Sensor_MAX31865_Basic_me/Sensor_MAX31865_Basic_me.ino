#include <Adafruit_MAX31865.h>

/*
  10->CS
  11->SDI
  12->SDO
  13->CLK
  3V3->3V3
*/
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

#define RREF 430.0
#define RNOMINAL 100.0

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  thermo.begin(MAX31865_2WIRE);
}

void loop() { 
  // put your main code here, to run repeatedly:
  float temp = thermo.temperature(RNOMINAL, RREF);
  Serial.print("temp: ");
  Serial.println(temp);
  delay(1000);
}
