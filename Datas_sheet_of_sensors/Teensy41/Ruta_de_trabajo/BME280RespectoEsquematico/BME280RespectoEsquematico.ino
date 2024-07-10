#include <Wire.h>
/*Libreria
Adafruit BME280 Library by Adafruit
*/
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA ( 1013.25)

Adafruit_BME280 Sensor01Bme280;


void FunIniciarBME280(Adafruit_BME280 & bme);
void FunObtenerDatosBME280(Adafruit_BME280 & bme);

void setup() {
  // put your setup code here, to run once:
 Serial.begin(4800);
 Wire2.begin();
 FunIniciarBME280(Sensor01Bme280);

}

void loop() {
  // put your main code here, to run repeatedly:
  FunObtenerDatosBME280(Sensor01Bme280);
  delay(2000);
}
/**/
void FunIniciarBME280(Adafruit_BME280 & bme){
  Serial.println("Init BME280...");
  while(! bme.begin(0x76, &Wire2)) delay(10);
  return;
}

void FunObtenerDatosBME280(Adafruit_BME280 & bme)
{
  bme.takeForcedMeasurement();

  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();

}