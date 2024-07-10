# INA219
Monitor de corriente y voltage
## Código
```ino
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
```
## Terminal
```
17:59:56.390 -> Bus Voltage:   0.89 V
17:59:56.390 -> Shunt Voltage: -0.03 mV
17:59:56.390 -> Load Voltage:  0.89 V
17:59:56.390 -> Current:       -0.40 mA
17:59:56.390 -> Power:         0.00 mW
```
## Guardado
```
Bus Voltage:(Voltage del bus) 
4byte -> 0-31bits -> 5 bytes
Shunt Voltage:(voltde del shunt)
4byte -> 0-31bits -> 5 bytes
Load Voltage:
4byte -> 0-31bits -> bytes
Current:(corriente) ->
5byte -> 0-39bits
Power:(potencia)
5byte -> 0-39bits   
```
# MAX31865
Sensor de temperatura por el método de una resistencia shunt
## Código
```ino
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
```
## Terminal
```
17:26:36.818 -> Resistencia RTD: 0.00 ohms	Temperatura: -242.02 °C
17:26:36.818 -> Fallo detectado: RTD low threshold
17:26:36.818 -> REFIN- > 0.85 x VBIAS
17:26:36.818 -> RTDIN- < 0.85 x VBIAS
```
## Guardado
```
Temperatura:
5 caracteres -> 5bytes -> 0-39
```
# MPU6050 GIROSCÓPIO
Giroscópio
## Código
```ino
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
```
## Terminal
```
11:17:36.821 -> Acelerómetro X: 10620 | Y: -8244 | Z: 12096
11:17:36.821 -> Giroscopio X: 1171 | Y: 1357 | Z: 1126
```
## Guardado
```
Acelerómetro:
x:
5 caracteres -> 5 bytes -> 40 bits 
y :
5 caracteres -> 5 bytes -> 40 bits
z:
5 caracteres -> 5 bytes -> 40 bits
Giroscópio:
x:
5 caracteres -> 5 bytes -> 40 bits
y:
5 caracteres -> 5 bytes -> 40 bits
z:
5 caracteres -> 5 bytes -> 40 bits
```
# BME280 SENSOR DE TEMPERATURA Y HUMEDAD
Sensor for measurement temperature, humidity and Presure  
## Código
```ino
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
```
## Terminal
```
11:40:47.383 -> Found BME280 sensor! Success.
11:40:47.425 -> Temp: 20.15°C		Humidity: 69.34% RH		Pressure: 100170.44Pa
```
## Guardado
```
Temp:
5 caracteres -> 5 bytes -> 40 bits
Humidity:
5 caracteres -> 5 bytes -> 40 bits
Pressure:
5 caracteres -> 5 bytes -> 40 bits
```
# BNO055 GRAVITROMETRO
Sensor  
## Código
```ino  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
```
## Terminal
- Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere
- Absolute Orientation (Quatenrion, 100Hz) Four point quaternion output for more accurate data manipulation
- Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s
- Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2
- Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro Tesla (uT)
- Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
- Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2
- Temperature (1Hz) Ambient temperature in degrees celsius
```
16:26:30.091 -> Orient:	x= 0.00 |	y= 4.37 |	z= 1.25
16:26:30.091 -> Gyro:	x= 0.00 |	y= -0.00 |	z= -0.01
16:26:30.091 -> Linear:	x= 0.00 |	y= 0.00 |	z= -0.32
16:26:30.091 -> Mag:	x= -15.25 |	y= 94.87 |	z= 33.50
16:26:30.091 -> Accl:	x= 0.72 |	y= -0.23 |	z= 9.47
16:26:30.091 -> Gravity:	x= 0.74 |	y= -0.22 |	z= 9.77
16:26:30.091 -> temperature:   24
16:26:30.091 -> Calibration: Sys=1 Gyro=3 Accel=0 Mag=0
```
## Guardado
```
Temp:

```


# GY-GPS6MV2
##
  ```ino
  
  ```
## Terminal
  ```

12:03:00.489 -> $GNGGA,170300.000,1201.04796,S,07703.00021,W,1,07,1.44,126.7,M,11.5,M,,*77

12:03:00.556 -> $GPGSA,A,3,13,15,12,19,17,22,24,,,,,,2.70,1.44,2.29,1*1F

12:03:00.604 -> $BDGSA,A,3,,,,,,,,,,,,,2.70,1.44,2.29,4*06

12:03:00.674 -> $GPGSV,3,1,11,13,66,126,41,15,56,210,38,12,41,293,29,19,41,97,26*44

12:03:00.740 -> $GPGSV,3,2,11,5,37,330,,17,26,120,22,22,24,150,23,24,24,219,34*46

12:03:00.780 -> $GPGSV,3,3,11,11,8,16,20,30,4,99,,14,3,141,*71

12:03:00.818 -> $BDGSV,0,1,00*69

12:03:00.887 -> $GNRMC,170300.000,A,1201.04796,S,07703.00021,W,0.001,140.10,270624,,,A*44

12:03:00.925 -> $GNZDA,170300.000,27,06,2024,,*4A

```
## Explicacion

### 1. $GNGGA - Información Fija del Sistema de Navegación Global por Satélite
- **Hora**: 17:03:00.000 UTC
- **Latitud**: 1201.04796 S (12 grados y 01.04796 minutos sur)
- **Longitud**: 07703.00021 W (77 grados y 03.00021 minutos oeste)
- **Calidad de la señal**: 1 (Fix GPS válido)
- **Número de satélites utilizados**: 07
- **HDOP (Dilución Horizontal de la Precisión)**: 1.44
- **Altitud**: 126.7 metros sobre el nivel del mar
- **Separación geoidal**: 11.5 metros
- **Checksum**: *77

### 2. $GPGSA - Datos Activos del Satélite GNSS
- **Modo de selección**: A (Automático)
- **Tipo de Fix**: 3 (3D Fix)
- **ID de satélites utilizados**: 13, 15, 12, 19, 17, 22, 24
- **PDOP (Dilución de la Precisión en la Posición)**: 2.70
- **HDOP (Dilución Horizontal de la Precisión)**: 1.44
- **VDOP (Dilución Vertical de la Precisión)**: 2.29
- **Checksum**: *1F

### 3. $BDGSA - Datos Activos del Satélite BeiDou (similares a GPGSA)
- **Checksum**: *06

### 4. $GPGSV - Satélites GNSS Visibles (Varias sentencias para GPS)
- **Total de mensajes GPGSV**: 3
- **Número de mensaje**: 1 a 3
- **Número total de satélites en vista**: 11
- **Información detallada de cada satélite** (como ID, elevación, azimuth, SNR)
- **Checksums**: *44, *46, *71

### 5. $BDGSV - Satélites BeiDou Visibles
- **Número de satélites en vista**: 00 (no satélites BeiDou detectados)
- **Checksum**: *69

### 6. $GNRMC - Información Mínima Recomendada GNSS
- **Hora**: 17:03:00.000 UTC
- **Estado**: A (Activo)
- **Latitud**: 1201.04796 S
- **Longitud**: 07703.00021 W
- **Velocidad sobre el suelo**: 0.001 nudos
- **Rumbo**: 140.10 grados
- **Fecha**: 27 de junio de 2024
- **Modo**: A (Autónomo)
- **Checksum**: *44

### 7. $GNZDA - Hora y Fecha
- **Hora**: 17:03:00.000 UTC
- **Día**: 27
- **Mes**: 06
- **Año**: 2024
- **Checksum**: *4A

# Longitud de datos de Pierina
```


```