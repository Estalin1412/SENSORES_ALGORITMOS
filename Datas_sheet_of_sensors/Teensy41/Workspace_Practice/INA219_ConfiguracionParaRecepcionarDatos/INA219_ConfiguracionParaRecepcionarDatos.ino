#include <Wire.h>

#define INA219_ADDR       0x40

#define REG_CONFG         0x00 
#define REG_SHUNTVOLTAGE  0x01
#define REG_BUSVOLTAGE    0x02
#define REG_POWER         0x03
#define REG_CORRENT       0x04
#define REG_CALIBRATION   0x05
/*
01 Shunt voltage 
02 Bus voltage 
03 Power(2)
04 Current(2) 
05 Calibration 
*/

void setup() {
  Serial.begin(115200);
/*
Esperar hasta que el serial se conecte si ses conecta finaliza el bucle para continuar
*/
while(!Serial) delay(10);
Wire1.setSDA(25);
Wire1.setSCL(24);
Wire1.begin();
/*[
Serial2 porque quiero activar el pin 24 para el scl y pin 25 sda;
y 4800 es lo que pide la naza para la transmisión de datos
*/
 Serial2.begin(4800);


}

void loop() {
  // put your main code here, to run repeatedly:

}
