/*Libreria
Adafruit MAX3185 Library by Adfruit
*/
#include <Adafruit_MAX31865.h>
/*
  10->CS
  11->SDI
  12->SDO
  13->CLK
  3V3->3V3
*/
Adafruit_MAX31865 SensorMAX3185 = Adafruit_MAX31865(10);

void FunIniciarMAX3185(Adafruit_MAX31865 & thermo);
void FunObtenerDatosMAX3185(Adafruit_MAX31865 & thermo);

#define RREF 430.0
#define RNOMINAL 100.0

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FunIniciarMAX3185(SensorMAX3185);
}

void loop() { 
  // put your main code here, to run repeatedly:
  FunObtenerDatosMAX3185(SensorMAX3185);
  delay(2000);
}

/*----------------------------------FUNCIONES_PARA_MAX3185---------------------------------------------------------------------------*/
void FunIniciarMAX3185(Adafruit_MAX31865 & thermo){
  thermo.begin(MAX31865_2WIRE);
}

void FunObtenerDatosMAX3185(Adafruit_MAX31865 & thermo){

  float temp = thermo.temperature(RNOMINAL, RREF);
  Serial.print("temp: ");
  Serial.println(temp);

}