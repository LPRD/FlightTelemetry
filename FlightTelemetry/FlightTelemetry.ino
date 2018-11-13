#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <avr/pgmspace.h>
#include <Telemetry.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
RTC_DS1307 RTC;
File dataFile;
char filename[] = "DATA000.csv";

#define LED           10

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
  Serial.begin(38400, SERIAL_8N2);    Serial.println();
  
  while (!bno.begin()) {                            // flashes to signal error
    Serial.println(F("BNO055 err"));
    digitalWrite(LED,LOW); delay(1000); digitalWrite(LED,HIGH);
  }
  if (!RTC.isrunning()) { RTC.adjust(DateTime(__DATE__, __TIME__)); }
  if (!SD.begin(10)) { Serial.println(F("SD err")); }
  else {                                            // generates file name
    for (uint16_t nameCount = 0; nameCount < 1000; nameCount++) {
      filename[4] = nameCount/100 + '0';
      filename[5] = (nameCount%100)/10 + '0';
      filename[6] = nameCount%10 + '0';
      if (!SD.exists(filename)) {                   // opens if file doesn't exist
        dataFile = SD.open(filename, FILE_WRITE);
        Serial.print(F("\twriting "));
        Serial.println(filename);
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
  
  // Downlink
  BEGIN_SEND
  SEND_VECTOR_ITEM(euler_angle ,  euler);
  SEND_VECTOR_ITEM(gyro        ,  gyroscope);
  SEND_ITEM(temperature, temp);
  SEND_VECTOR_ITEM(magnetometer,  magnetometer);
  SEND_VECTOR_ITEM(acceleration,  accelerometer);
  END_SEND
  
  // Writing to SD Card
  DateTime now = RTC.now();
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
