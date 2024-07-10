#include <Wire.h>
#include <Adafruit_INA219.h>

// Crear una instancia del sensor INA219
Adafruit_INA219 ina219;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
      // Espera a que se abra la consola serie (para placas como Leonardo)
      delay(1);
  }

  Serial.println("Inicializando el sensor INA219...");

  // Configurar los pines alternativos para Wire1
  Wire1.setSDA(25); // Configurar el pin 25 como SDA
  Wire1.setSCL(24); // Configurar el pin 24 como SCL
  Wire1.begin();

  // Inicializa el sensor INA219 usando Wire1
  ina219.begin(&Wire1);

  Serial.println("Medición de voltaje y corriente con el INA219...");
}

void loop(void) {
  float shuntVoltage_mV = 0;
  float busVoltage_V = 0;
  float current_mA = 0;
  float power_mW = 0;
  float loadVoltage_V = 0;

  // Leer el voltaje del shunt (en mV)
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  // Leer el voltaje del bus (en V)
  busVoltage_V = ina219.getBusVoltage_V();
  // Leer la corriente (en mA)
  current_mA = ina219.getCurrent_mA();
  // Leer la potencia (en mW)
  power_mW = ina219.getPower_mW();
  // Calcular el voltaje de carga
  loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

  // Imprimir los resultados
  Serial.print("Bus Voltage:   "); Serial.print(busVoltage_V); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage_mV); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadVoltage_V); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");

  // Esperar 2 segundos antes de la siguiente lectura
  delay(2000);
}
