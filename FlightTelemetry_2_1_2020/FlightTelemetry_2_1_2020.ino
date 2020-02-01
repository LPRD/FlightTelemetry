#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <avr/pgmspace.h>
#include "Adafruit_BMP3XX.h"


#define TELEMETRY_SERIAL Serial1  //Serial fine for Arduino Boards + Due
//Teensy 3.6 has to use Serial1 or higher
#include <Telemetry.h>

//Setup for BMP388 Sensor
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 15
//Calibration Factor for BMP388, chech loacl pressure b4 flight!
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);  //software SPI

//put setup for GPS here:





Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);//ID, Address, Wire //use the syntax &Wire1, 2, ... for SDA1, 2, ...
RTC_DS1307 rtc;                         //55 as the only argument also works
File dataFile;
char filename[] = "DATA000.csv";

#define LED    13// BUILTIN_SDCARD  //use BUILTIN_SDCARD for Teensy 3.6
//also change line ~46 to: 
//if (!SD.begin(BUILTIN_SDCARD)) { TELEMETRY_SERIAL.println(F("SD err")); }
//10 (or other) for all other boards)

//I'm not sure if the above is correct


#define SEND_VECTOR_ITEM(field, value)\
  SEND_ITEM(field, value.x())         \
  SEND_GROUP_ITEM(value.y())          \
  SEND_GROUP_ITEM(value.z())

#define WRITE_CSV_ITEM(value)         \
  dataFile.print(F(", ")); dataFile.print(value);

#define WRITE_CSV_VECTOR_ITEM(value)  \
  WRITE_CSV_ITEM(value.x())           \
  WRITE_CSV_ITEM(value.y())           \
  WRITE_CSV_ITEM(value.z())

unsigned int missed_deadlines = 0;


void setup() {
  TELEMETRY_SERIAL.begin(57600);    TELEMETRY_SERIAL.println();

  while (!bmp.begin()) {
    TELEMETRY_SERIAL.println(F("BMP388 err"));
    digitalWrite(LED,LOW); delay(1000); digitalWrite(LED,HIGH);
  } 
  
  //does it make sense that the CS pin for the SD card is flashing on/off????

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  //put GPS initialization code here:



  
  
  while (!bno.begin()) {                            // flashes to signal error
    TELEMETRY_SERIAL.println(F("BNO055 err"));
    digitalWrite(LED,LOW); delay(1000); digitalWrite(LED,HIGH);
  }
  if (!rtc.isrunning()) { rtc.adjust(DateTime(__DATE__, __TIME__)); }
  if (!SD.begin(BUILTIN_SDCARD)) { TELEMETRY_SERIAL.println(F("SD err")); }
  else {                                            // generates file name
    for (uint16_t nameCount = 0; nameCount < 1000; nameCount++) {
      filename[4] = nameCount/100 + '0';
      filename[5] = (nameCount%100)/10 + '0';
      filename[6] = nameCount%10 + '0';
      if (!SD.exists(filename)) {                   // opens if file doesn't exist
        dataFile = SD.open(filename, FILE_WRITE);
        TELEMETRY_SERIAL.print(F("\twriting "));
        TELEMETRY_SERIAL.println(filename);
        dataFile.println(F("abs time,sys date,sys time,x_euler_angle,y_euler_angle,z_euler_angle,x_gyro,y_gyro,z_gyro,temperature,x_magnetometer,y_magnetometer,z_magnetometer,x_acceleration,y_acceleration,z_acceleration"));
        break;
      }
    }
  }
  
}


void loop() {
  long time0 = millis();
  imu::Vector<3> gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> magnetometer  = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  int8_t temp = bno.getTemp();
  
  int8_t BMPtemp= bmp.temperature; //in C, do I need int8_t type????
  //int8_t BMPpressure= bmp.pressure / 100.0; //hPa not super important
  int8_t BMPalt= bmp.readAltitude(SEALEVELPRESSURE_HPA); //m
  
  
  
  
    
  // Downlink
  BEGIN_SEND
  SEND_VECTOR_ITEM(euler_angle ,  euler);
  SEND_VECTOR_ITEM(gyro        ,  gyroscope);
  SEND_ITEM(temperature, temp);
  SEND_VECTOR_ITEM(magnetometer,  magnetometer);
  SEND_VECTOR_ITEM(acceleration,  accelerometer);

  SEND_ITEM(BMP_Temp, BMPtemp); //can the first part be named whatever I want????
  SEND_ITEM(BMP_Alt, BMPalt);
  
  END_SEND
  
  // Writing to SD Card
  DateTime now = rtc.now();
  dataFile.print(millis());           dataFile.print(',');
  dataFile.print(now.year()  ,DEC);   dataFile.print('/');
  dataFile.print(now.month() ,DEC);   dataFile.print('/');
  dataFile.print(now.day()   ,DEC);   dataFile.print(',');
  dataFile.print(now.hour()  ,DEC);   dataFile.print(':');
  dataFile.print(now.minute(),DEC);   dataFile.print(':');
  dataFile.print(now.second(),DEC);
  WRITE_CSV_VECTOR_ITEM(euler)
  WRITE_CSV_VECTOR_ITEM(gyroscope)
  WRITE_CSV_ITEM(temp)
  WRITE_CSV_VECTOR_ITEM(magnetometer)
  WRITE_CSV_VECTOR_ITEM(accelerometer)
  
  dataFile.println();
  dataFile.flush();
}
