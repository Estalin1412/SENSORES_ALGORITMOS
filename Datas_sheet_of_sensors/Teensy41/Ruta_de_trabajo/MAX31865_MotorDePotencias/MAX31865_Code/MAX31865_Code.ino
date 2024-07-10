#include <Adafruit_MAX31865.h>

// Definir el pin CS (Chip Select)
#define CS_PIN 10

// Crear un objeto MAX31865 con la configuración por defecto (RTD_PT100)
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN);
/*
Pi 3V3 to sensor VIN
Pi GND to sensor GND
Pi 11 MOSI to sensor SDI
Pi 12 MISO to sensor SDO
Pi 13 SCLK to sensor CLK
Pi 10 to sensor CS 
*/

// Si estás utilizando un RTD PT1000, descomenta la siguiente línea y comenta la línea anterior
// Adafruit_MAX31865 max = Adafruit_MAX31865(CS_PIN, MAX31865_3WIRE, 1000);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(1); // Esperar a que el puerto serie se inicialice
  }
  
  // Inicializar el MAX31865
  if (!max31865.begin(MAX31865_3WIRE)) {  // También puedes usar MAX31865_2WIRE o MAX31865_4WIRE dependiendo de tu sensor
    Serial.println("No se pudo encontrar el sensor MAX31865. Verifique las conexiones.");
    while (1);
  }
}

void loop() {
  // Leer la resistencia del sensor RTD
  float rtd_resistance = max31865.readRTD();

  // Calcular la temperatura en grados Celsius
  float temperature = max31865.temperature(100, rtd_resistance);

  // Imprimir los resultados
  Serial.print("Resistencia RTD: ");
  Serial.print(rtd_resistance);
  Serial.print(" ohms\tTemperatura: ");
  Serial.print(temperature);
  Serial.println(" °C");

  uint8_t fault = max31865.readFault();
  if (fault) {
    Serial.print("Fallo detectado: ");
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD high threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD low threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x VBIAS");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x VBIAS");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x VBIAS");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Over/under voltage");
    }
    max31865.clearFault();
  }


  delay(1000); // Esperar 1 segundo antes de la siguiente lectura
}

