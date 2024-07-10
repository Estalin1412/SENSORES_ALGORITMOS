#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
/*
  Libary: 
    MPU6050
    
  Pins conectivity:
    3V  - VCC
    GND - GNG
    SCL - 19-A5
    SDA - 18-A4
*/

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Inicializa el MPU-6050
  Serial.println("Inicializando el MPU-6050...");
  mpu.initialize();

  // Verifica la conexión
  Serial.println(mpu.testConnection() ? "MPU-6050 conectado correctamente" : "Error en la conexión del MPU-6050");

  // Configura el rango del acelerómetro y giroscopio (opcional)
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  // Lee los valores del acelerómetro y giroscopio
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Imprime los valores en el monitor serie
  Serial.print("Acelerómetro X: "); Serial.print(ax);
  Serial.print(" | Y: "); Serial.print(ay);
  Serial.print(" | Z: "); Serial.println(az);

  Serial.print("Giroscopio X: "); Serial.print(gx);
  Serial.print(" | Y: "); Serial.print(gy);
  Serial.print(" | Z: "); Serial.println(gz);

  delay(500);
}

