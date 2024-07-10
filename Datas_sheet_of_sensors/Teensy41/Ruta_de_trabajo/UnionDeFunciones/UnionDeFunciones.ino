//LIBRERIAS PARA BME280
#include <BME280.h>
#include <BME280I2C.h>
#include <BME280I2C_BRZO.h>
#include <BME280Spi.h>
#include <BME280SpiSw.h>
#include <EnvironmentCalculations.h>
//LIBRERIAS GENERALES
#include <Wire.h>//Para la coneccion I2C

BME280I2C bme;

void BME280ImprimirDatos( Stream* client );
void BMEFunActivarSensor(int SERIAL_BAUD, BME280I2C & bme); 

void setup()
{           
BMEFunActivarSensor(115200, bme);
}

void loop()                     
{
BME280ImprimirDatos( &Serial);
delay(1000);
}
/*-------------------------FUNCIONES_PARA_BME280------------------------------------------------------------------*/
void BMEFunActivarSensor(int SERIAL_BAUD, BME280I2C & bme){
    BME280I2C var;
    bme = var;
  Serial.begin(SERIAL_BAUD);

  while(!Serial);
    //Incializa la conección I2C
  Wire.begin();

  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("BME280 Conección excelente.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Sensor no conectado! Error!");
  }
}

void BME280ImprimirDatos( Stream* client )
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("Pa");

   delay(1000);
}
