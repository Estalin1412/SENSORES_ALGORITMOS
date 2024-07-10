#include <MS5611.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#define placa "Arduino Mega"
#include <SoftwareSerial.h>
#include "DHT.h"
#include <MQ131.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SD.h>  //SD
#include <SPI.h> //SD




Adafruit_INA219 ina219_B(0x41);
Adafruit_INA219 ina219_C(0x45);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
MS5611 MS5611(0x77);   // 0x76 = CSB to VCC; 0x77 = CSB to GND
Adafruit_MAX31865 thermo = Adafruit_MAX31865(8, 9, 10, 11);
TinyGPS gps;
File myFile;
int pinCS = 53;
#define RREF      430.0
#define RNOMINAL  100.0
#define DHTPIN 7    
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);
int measurePin = A5;
int ledPower = 12;
int pinanalogico=A0;
int check =0;
int check_1 =0;
byte RFin_bytes[]={0,0};
unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
float control;//sensor de flujo

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);



String humedad1 = ""; //SHT31
String temperatura1 = ""; //SHT31
String humedad2 = "";  //DHT22
String temperatura2 = ""; //DHT22
String presion = "";   //GY-63/ MS5611
String temperatura3 = "";  //GY-63/ MS5611
String temperatura4 = ""; //RTD
String densidad1 = ""; //GP2Y10
String densidad21 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 0.3um /0.1L
String densidad22 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 0.5um /0.1L
String densidad23 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 1.0um /0.1L
String densidad24 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 2.5um /0.1L
String densidad25 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 5.0um /0.1L
String densidad26 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 10.0um /0.1L
String densidad27 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 2.5um /0.1L PMS
String densidad28 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 5.0um /0.1L PMS
String densidad29 = ""; //PMS7003 Tamaño de tamaño de las pártículas > 10.0um /0.1L PMS



String voltaje1 = "";   //INA219B 0X45
String corriente1 = ""; //INA219B 0X45
String potencia1 = "";  //INA219B 0X45
String voltaje2 = "";    //INA219C 0X41
String corriente2 = "";  //INA219C 0X41
String potencia2 = "";   //INA219C 0X41
String flujo = "";   //sensor de flujo
String concentracion1 = "";   //MQ131  O3 ppm
String concentracion2 = "";   //MQ131  O3 ppb
String concentracion3 = "";   //MQ131  O3 mg/m3
String concentracion4 = "";   //MQ131  O3 ug/m3

String ubicacion = "";    //GPS

String latitud = "";  //GPS
String longitud = "";   //GPS
String hora1 = "";   //GPS
String hora2 = "";   //GPS
String hora3 = "";   //GPS

String fecha1 = "";  //GPS
String fecha2 = "";  //GPS
String fecha3 = "";  //GPS

String Cadena = "";


String texto_h1 = "\nH. relativa(%):", texto_humedad1 = "";
String texto_t1= "\nTemp(°C):", texto_temperatura1 = "";
String texto_h2 = "\nH. relativa(%):", texto_humedad2 = "";
String texto_t2= "\nTemp(°C):", texto_temperatura2 = "";
String texto_p = "\nPresion(mbar):", texto_presion = "";
String texto_t3= "\nTemp(°C):", texto_temperatura3 = "";
String texto_t4= "\nTemp(°C):", texto_temperatura4 = "";
String texto_d1= "\nppm:", texto_densidad1 = "";
String texto_d21= "\n0.3um/0.1L:", texto_densidad21 = "";
String texto_d22= "\n0.5um/0.1L:", texto_densidad22 = "";
String texto_d23= "\n1.0um/0.1L:", texto_densidad23 = "";
String texto_d24= "\n2.5um/0.1L:", texto_densidad24 = "";
String texto_d25= "\n5.0um/0.1L:", texto_densidad25 = "";
String texto_d26= "\n10.0um/0.1L:", texto_densidad26 = "";
String texto_d27= "\n1.0um/0.1L:", texto_densidad27 = "";
String texto_d28= "\n5.0um/0.1L:", texto_densidad28 = "";
String texto_d29= "\n100um/0.1L:", texto_densidad29 = "";



String texto_v1= "\n10.0um/0.1L:", texto_voltaje1 = "";
String texto_c1= "\n10.0um/0.1L:", texto_corriente1 = "";
String texto_p1= "\n10.0um/0.1L:", texto_potencia1 = "";
String texto_v2= "\n10.0um/0.1L:", texto_voltaje2 = "";
String texto_c2= "\n10.0um/0.1L:", texto_corriente2 = "";
String texto_p2= "\n10.0um/0.1L:", texto_potencia2 = "";
String texto_f= "\n SLPM :", texto_flujo = "";
String texto_c_O3ppm= "\n ppm :", texto_concentracion1 = "";
String texto_c_O3ppb= "\n ppb :", texto_concentracion2 = "";
String texto_c_O3mg_m3= "\n mg/m3 :", texto_concentracion3 = "";
String texto_c_O3ug_m3= "\n ug/m3 :", texto_concentracion4 = "";


String texto_lat= "\n ppm :", texto_latitud = "";
String texto_longi= "\n ppb :", texto_longitud = "";
String texto_hor= "\n mg/m3 :", texto_hora = "";
String texto_fech= "\n ug/m3 :", texto_fecha = "";



String texto_u= "\n ub :", texto_ubicacion = "";



void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  pinMode(ledPower,OUTPUT);
  sht31.begin(0x44);
  dht.begin();
  MS5611.begin();
  thermo.begin(MAX31865_3WIRE); //RTD
  pinMode(13, OUTPUT);
  MQ131.begin(2,A4, HIGH_CONCENTRATION, 10);
  pinMode(pinCS, OUTPUT);
  
  if (! ina219_B.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
    if (! ina219_C.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
  
}

struct pms7003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms7003data data;
uint32_t currentFrequency;




void loop() {
  
  int result = MS5611.read();
  myFile = SD.open("test.txt", FILE_WRITE);
  
  Humedad1();
  Temperatura1();
  Humedad2();
  Temperatura2();
  Presion();
  Temperatura3();
  Temperatura4();
  Densidad1();
  Densidadpms();
  INA219B();
  INA219C();
  Flujo();
  ConcentracionO3();
  Ubicacion();
  GPS();
   
  Cadena=humedad1+ ";" + temperatura1 + ";" + humedad2 + ";" + temperatura2 + ";" + presion + ";" + temperatura3 + ";" + temperatura4 + ";" + densidad1 + ";" +  densidad21 + ";" + densidad22+ ";" + densidad23+ ";" + densidad24+ ";" + densidad25+ ";" + densidad26+ ";"  + densidad27+ ";" + densidad28+ ";" + densidad29 + ";" + corriente1+ ";" + corriente2+ ";" + flujo + ";" + concentracion1 + ";" + concentracion2+ ";" + concentracion3 + ";" + concentracion4+";" +latitud + ";" + longitud+ ";" + hora1 + ";" + hora2 + ";" + hora3 + ";" + fecha1+ ";" +fecha2+ ";" +fecha3;
 
  //Cadena=  voltaje1 + ";" + corriente1+ ";" + potencia1+ ";" + voltaje2+ ";" + corriente2+ ";" + potencia2;
  //Cadena=  flujo;
  //Cadena=  concentracion1 + ";" + concentracion2+ ";" + concentracion3 + ";" + concentracion4;
  Serial.println(Cadena);
  
     
  if (myFile) {    
    myFile.println(Cadena);
    //myFile.print(",");    
   // myFile.println(temperatura1);
    myFile.close(); // close the file
    Serial.println("Guardando datos...");
  }
    else {
    Serial.println("error opening test.txt");
  }
  delay(3000);

}

void Humedad1() {
  float h1 = sht31.readHumidity();
  humedad1 = String(h1);
  texto_humedad1 = texto_h1 + humedad1;
  //Serial.println(texto_humedad1);
  
}
void Temperatura1() {
  float t1 = sht31.readTemperature();
  temperatura1 = String(t1);
  texto_temperatura1 = texto_t1 + temperatura1;
  //Serial.println(texto_temperatura1);
 
}
void Humedad2() {
  float h2 = dht.readHumidity();
  humedad2 = String(h2);
  texto_humedad2 = texto_h2 + humedad2;
  //Serial.println(texto_humedad1);
  
}
void Temperatura2() {
  float t2 = dht.readTemperature();
  temperatura2 = String(t2);
  texto_temperatura2 = texto_t2 + temperatura2;
  //Serial.println(texto_temperatura1);
  
}

void Presion() {
  float p = 2*MS5611.getPressure();
  presion = String(p);
  texto_presion = texto_p + presion;
  //Serial.println(texto_humedad1);
  
}
void Temperatura3() {
  float t3 = MS5611.getTemperature();
  temperatura3 = String(t3);
  texto_temperatura3 = texto_t3 + temperatura3;
  //Serial.println(texto_temperatura1);
  
}

void Temperatura4() {
  float t4 = thermo.temperature(RNOMINAL, RREF);
  temperatura4 = String(t4);
  texto_temperatura4 = texto_t4 + temperatura4;

}

void Densidad1() {
  
  digitalWrite(ledPower,LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured*(5.0/1024);
  dustDensity = 0.17*calcVoltage-0.1;

  if ( dustDensity < 0)
  {
    dustDensity = 0.00;
  }
  
  float d1 = dustDensity;
  densidad1 = String(d1);
  texto_densidad1 = texto_d1 + densidad1;

}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 /*
 debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  
  */
  // The data comes in endian'd, this solves it so it works on all platforms

  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
  delay(1000);
}


void Densidadpms() {
  if (readPMSdata(&Serial3)) {
  int d21 = data.particles_03um;
  int d22 = data.particles_05um;
  int d23 = data.particles_10um;
  int d24 = data.particles_25um;
  int d25 = data.particles_50um;
  int d26 = data.particles_100um;
  
  int d27 = data.pm10_standard;
  int d28 = data.pm25_standard;
  int d29 = data.pm100_standard;
  
  
  densidad21 = String(d21);
  densidad22 = String(d22);
  densidad23 = String(d23);
  densidad24 = String(d24);
  densidad25 = String(d25);
  densidad26 = String(d26);
  
  densidad27 = String(d27);
  densidad28 = String(d28);
  densidad29 = String(d29);
  
  
  
  
  texto_densidad21 = texto_d21 + densidad21;
  texto_densidad22 = texto_d22 + densidad22;
  texto_densidad23 = texto_d23 + densidad23;
  texto_densidad24 = texto_d24 + densidad24;
  texto_densidad25 = texto_d25 + densidad25;
  texto_densidad26 = texto_d26 + densidad26;
  
  //Serial.println(texto_densidad21);
 // Serial.println(texto_densidad22);
 // Serial.println(texto_densidad23);
  //Serial.println(texto_densidad24);
  //Serial.println(texto_densidad25);
  //Serial.println(texto_densidad26);
  }
}


void INA219B() {
  float shuntvoltage1 = 0;
  float busvoltage1 = 0;
  float current_mA1 = 0;
  float loadvoltage1 = 0;
  float power_mW1 = 0;

  shuntvoltage1 = ina219_B.getShuntVoltage_mV();
  busvoltage1 = ina219_B.getBusVoltage_V();
  current_mA1 = ina219_B.getCurrent_mA();
  power_mW1 = ina219_B.getPower_mW();
  loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);
  
  float v1 = busvoltage1;
  float c1 = current_mA1;
  float p1 = power_mW1;
  voltaje1 = String(v1);
  corriente1 = String(c1);
  potencia1 = String(p1);
  texto_voltaje1 = texto_v1 + voltaje1;
  texto_corriente1 = texto_c1 + corriente1;
  texto_potencia1 = texto_p1 + potencia1;
  

}
 void INA219C() {
  float shuntvoltage2 = 0;
  float busvoltage2 = 0;
  float current_mA2 = 0;
  float loadvoltage2 = 0;
  float power_mW2 = 0; 
   
   
  shuntvoltage2 = ina219_C.getShuntVoltage_mV();
  busvoltage2 = ina219_C.getBusVoltage_V();
  current_mA2 = ina219_C.getCurrent_mA();
  power_mW2 = ina219_C.getPower_mW();
  loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);
   
  float v2 = busvoltage2;
  float c2 = current_mA2;
  float p2 = power_mW2;
  voltaje2 = String(v2);
  corriente2 = String(c2);
  potencia2 = String(p2);
  texto_voltaje2 = texto_v2 + voltaje2;
  texto_corriente2 = texto_c2 + corriente2;
  texto_potencia2 = texto_p2 + potencia2;

}

void Flujo() {
  float f = analogRead(pinanalogico);
  //f = (f/100);
  f = (((f-90)*5)/(1023-90));
  //f = (1.25*f-1.25); 
  flujo = String(f);
  texto_flujo = texto_f + flujo;
}

void ConcentracionO3() {
/*  MQ131.sample();
  float c_O3ppm = MQ131.getO3(PPM);
  float c_O3ppb = MQ131.getO3(PPB);
  float c_O3mg_m3 = MQ131.getO3(MG_M3);
  float c_O3ug_m3 = MQ131.getO3(UG_M3);
  
  concentracion1 = String(c_O3ppm);
  concentracion2 = String(c_O3ppb);
  concentracion3 = String(c_O3mg_m3);
  concentracion4 = String(c_O3ug_m3);
  
  texto_concentracion1 = texto_c_O3ppm + concentracion1;
  texto_concentracion2 = texto_c_O3ppb + concentracion2;
  texto_concentracion3 = texto_c_O3mg_m3 + concentracion3;
  texto_concentracion4 = texto_c_O3ug_m3 + concentracion4;
  */
}  
 
  void GPS() {
   
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);//latitud
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);//longitud
  print_date(gps);
    
    
 // float u = 2*MS5611.getPressure();
 // presion = String(p);
  //texto_presion = texto_p + presion;
  //Serial.println(texto_humedad1);
  
  }
  
  static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{

  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('0');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);

    
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
      
  }
  smartdelay(0);
  
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{}
static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime( &year, &month, &day,&hour, &minute, &second, &hundredths, &age);


  
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("0");
  else
  {
    char sz[32];
    sprintf(sz, "%02d;%02d;%02d;%02d;%02d;%02d ",
        hour, minute, second, month, day, year);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}
  
void Ubicacion() {
  float lat = -12.016668 ;
  float longi = -77.050369;
  int hor1 = 03;
  int hor2= 11;
  int hor3 = 44;
  int fech1 = 07;
  int fech2 = 01;
  int fech3 = 2021;
  
  lat = -12.016668 ;
  longi = -77.050369;
  hor1 = 03;
  hor2= 11;
  hor3 = 44;
  fech1 = 07;
  fech2 = 01;
  fech3 = 2021;
  
  
  
  latitud = String(lat);
  longitud = String(longi);
  hora1 = String(hor1);
  hora2 = String(hor2);
  hora3 = String(hor3);

  fecha1 = String(fech1);
  fecha2 = String(fech2);
  fecha3 = String(fech3);
  /*
  
  latitud = String("-12.016668");
  longitud = String("-77.050369");
  hora1 = String("03");
  hora2 = String("09");
  hora3 = String("44");

  fecha1 = String("07");
  fecha2 = String("01");
  fecha3 = String("2021");
  */
  
  
 // latitud + ";" + longitud+ ";" + hora1 + ";" + hora2+ ";" + hora3 + ";" + fecha1+ ";" +fecha2+ ";" +fecha3;
  

}  

void serialEvent1(){
  //Recepción de datos Seriales
  while(Serial1.available()) {              //Si existen datos seriales, leer a todos
    for(int n=0; n<2; n++){
    RFin_bytes[n] = Serial1.read();
    }
    Serial.println(RFin_bytes[0]);
    Serial.println(RFin_bytes[1]);
    check=(RFin_bytes[0]-5)/16;
    check_1=RFin_bytes[1]%16;
    Serial.print("Este es check:");
    Serial.println(check);
    Serial.print("Este es check_1:");
    Serial.println(check_1);
    if(check+check_1==16){
      switch (RFin_bytes[1]){
        case 255:
          Serial.println("sys_off");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);                    
   
          break;
        case 1:
          Serial.println("fan_off");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);
  
          break;
        case 2:
          Serial.println("fan_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);
  
          break;
        case 3:
          Serial.println("hp1_off");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);
  
          break;
        case 4:
          Serial.println("hp1_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);
  
          break;        
        case 5:
          Serial.println("hp2_off");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);
  
          break;
        case 6:
          Serial.println("hp2_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);
          
          break;  
        case 7:
          Serial.println("pms_off");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);  
          
          break;
        case 8:
          Serial.println("pms_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);           
          
          break;  
        case 9:
          Serial.println("ds_off");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);           
          
          break;
        case 10:
          Serial.println("ds_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);           
          
          break;
        case 11:
          Serial.println("ds_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);           
          
          break;
        case 12:
          Serial.println("ds_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);           
          
          break; 
        case 13:
          Serial.println("ds_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);           
          
          break;
        case 14:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);       
          break;
        case 15:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);            
          break;
        case 17:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 18:
          Serial.println("ds_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);           
          
          break;
        case 19:
          Serial.println("ds_on");
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);           
          
          break;
        case 20:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 21:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 22:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 23:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 24:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 25:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 26:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 27:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 28:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 29:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 30:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 31:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 33:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 34:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 35:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 36:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 37:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 38:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 39:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;
        case 40:
          Serial.println("ds_on"); 
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          digitalWrite(13, LOW);          
          break;                                                                                                                                                                                                                                                                                                                                                                     
      }  
    }
    else {
     Serial.println("Comando errado"); 
    }
  


   break;
    
  }


  
}