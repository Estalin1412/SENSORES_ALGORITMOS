/*---------------------------------------------------LIBRERIAS---------------------------------------------------------------------------*/
#include <Wire.h>
/*Libreria
Adafruit INA219 by adafruit
*/
#include <Adafruit_INA219.h>
/*Libreria
Adafruit BME280 Library by Adafruit
*/
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
/*--------------------------------------------------------------------VARIABLES------------------------------------------------------------*/
#define SEALEVELPRESSURE_HPA ( 1013.25)

/*Definimos objetos*/
Adafruit_INA219 SensorCorriente_Ina219;
Adafruit_BME280 Sensor01Bme280;
/*-----------------------------------------PROTOTIPOS DE FUNCIONES-------------------------------------------------------------------------*/
void FunIniciarINA219(Adafruit_INA219 & ina219);
void FunObtenerDatosINA219( Adafruit_INA219 &  ina219);

void FunIniciarBME280(Adafruit_BME280 & bme);
void FunObtenerDatosBME280(Adafruit_BME280 & bme);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(4800);
  Serial.println("Init Serial...");
  while(!Serial) delay(10);
  Wire2.begin();

  FunIniciarBME280(Sensor01Bme280);
  FunIniciarINA219(SensorCorriente_Ina219);
} 

void loop() {
  // put your main code here, to run repeatedly:
  FunObtenerDatosBME280(Sensor01Bme280);
  FunObtenerDatosINA219(SensorCorriente_Ina219);
  delay(2000);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------FuncionesParaIna219--------------------------------------------------------------------*/
void FunIniciarINA219(Adafruit_INA219 & ina219){
  // Wire2.setSCL(24);
  // Wire2.setSDA(25);

  while(!ina219.begin(&Wire2))  delay(10);
  
  Serial.println("INA219 conect succefull!!");

  ina219.setCalibration_32V_2A();

}

void FunObtenerDatosINA219( Adafruit_INA219 &  ina219){
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
  return;
}

/*-------------------------------------------FuncionesParaBME280-----------------------------------------------------*/
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