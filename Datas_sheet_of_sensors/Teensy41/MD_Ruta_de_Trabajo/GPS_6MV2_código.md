# PROGRAMACIÓN DE GPS6MV2
## Código
```ino

```
## Terminal
### Cómo aparece en la terminal
```
07:59:26.219 -> $GNGGA,125925.648,,,,,0,00,,,M,,M,,*64

07:59:26.263 -> $GPGSA,A,1,,,,,,,,,,,,,,,,1*03

07:59:26.263 -> $BDGSA,A,1,,,,,,,,,,,,,,,,4*17

07:59:26.297 -> $GPGSV,1,1,01,20,,,22*7A

07:59:26.343 -> $BDGSV,0,1,00*69

07:59:26.387 -> $GNRMC,125925.648,V,,,,,,,270624,,,N*54

07:59:26.423 -> $GNZDA,125925.648,27,06,2024,,*4D
```

###  Lectura
1. $GNGGA - Global Navigation Satellite System Fix Data
- Hora: 125925.648 (12:59:25 UTC)
- Latitud y Longitud: No disponibles (campos vacíos)
- Calidad de la señal: 0 (fix no válido)
- Número de satélites: 00 (ningún satélite detectado)
- HDOP (Horizontal Dilution of Precision): No disponible
- Altitud: No disponible
- Unidades de Altitud: Metros
- Separación geoidal: No disponible
- Unidades de separación geoidal: Metros
- Tiempo desde la última actualización DGPS: No disponible
- ID de la estación DGPS: No disponible
- Checksum: *64
2. $GPGSA - GNSS DOP and Active Satellites
- Modo: A (Automático)
- Tipo de Fix: 1 (sin fix)
- Satélites utilizados: No hay listados (todos los campos están vacíos)
- PDOP (Position Dilution of Precision): No disponible
- HDOP (Horizontal Dilution of Precision): No disponible
- VDOP (Vertical Dilution of Precision): No disponible
- Checksum: *03
3. $BDGSA - GNSS DOP and Active Satellites (para satélites BeiDou)
- Modo: A (Automático)
- Tipo de Fix: 1 (sin fix)
- Satélites utilizados: No hay listados
- Checksum: *17
4. $GPGSV - GNSS Satellites in View (GPS)
- Número de mensajes: 1
- Mensaje actual: 1
- Número de satélites en vista: 01
- Satélite ID 20: Sin información adicional disponible, solo SNR (22)
- Checksum: *7A
5. $BDGSV - GNSS Satellites in View (BeiDou)
- Número de mensajes: 0
- Mensaje actual: 1
- Número de satélites en vista: 00
- Checksum: *69
6. $GNRMC - Recommended Minimum Specific GNSS Data
- Hora: 125925.648 (12:59:25 UTC)
- Estado: V (Advertencia, navegación inactiva)
- Latitud y Longitud: No disponibles
- Velocidad sobre el suelo: No disponible
- Rumbo: No disponible
- Fecha: 27 de junio de 2024
- Variación magnética: No disponible
- Checksum: *54
7. $GNZDA - Time & Date - UTC, Day, Month, Year and Local Time Zone
- Hora: 125925.648 (12:59:25 UTC)
- Día: 27
- Mes: 06
- Año: 2024
- Zona horaria local: No disponible
- Checksum: *4D