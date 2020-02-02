#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <avr/pgmspace.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>
#include <Telemetry.h>

//GPS setup
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

//radio setup
#define TELEMETRY_SERIAL Serial //Teensy 3.6 has to use Serial1 or higher

//BMP388 setup
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 15
//Calibration Factor for BMP388, chech loacl pressure b4 flight!
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);  //software SPI

//BNO setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);//ID, Address, Wire
//use the syntax &Wire1, 2,... for SDA1, 2,... //55 as the only argument also works

//internal clock setup
RTC_DS1307 rtc;

//SD logging setup
File dataFile;
char filename[] = "DATA000.csv";

#define LED 13 //Error LED

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

#define launch_lat 44.975313
#define launch_lon -93.232216
#define land_lat (44.975313+.00035)
#define land_lon (-93.232216+.00035)

void setup() {
  Serial2.begin(GPSBaud); //Serial2 is the radio
  TELEMETRY_SERIAL.begin(57600); TELEMETRY_SERIAL.println();
  
  while (!bmp.begin()) {                         //flashes to signal error
    TELEMETRY_SERIAL.println(F("BMP388 err"));
    digitalWrite(LED,LOW); delay(1000); digitalWrite(LED,HIGH);
  } 
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  while (!bno.begin()) {                         //flashes to signal error
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
        dataFile.println(F("abs time,sys date,sys time,x angle,y angle,z angle,x gyro,y gyro,z gyro,bno temp,x mag,y mag,z mag,x accel,y accel,z accel,bmp alt,gps alt,gps lat,gps lon,gps vel,gps dir,xy_from_lanch,dir_from_launch,xy_to_land,xy_dir_to_land,x_to_land,y_to_land,bmp temp,bmp pressure,sats,hdop"));
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
  double temp = bno.getTemp();
    double bmp_temp= bmp.temperature; //in Celcius, do I need int8_t type????
    double bmp_pressure= bmp.pressure / 100.0; //hPa or mbar 
  double bmp_alt= bmp.readAltitude(SEALEVELPRESSURE_HPA); //m
  int16_t sats= gps.satellites.value();
    float fix_hdop= gps.hdop.hdop(); //horiz. diminution of precision
  double gps_lat= gps.location.lat();
  double gps_lon= gps.location.lng();
  double gps_alt= gps.altitude.meters(); //alt above SL in m
  double gps_vel= gps.speed.mps(); //abs xy speed in m/s
  double gps_dir= gps.course.deg(); //abs xy course in deg, N=0, E=90...
  double xy_from_lanch= TinyGPSPlus::distanceBetween(launch_lat, launch_lon, gps_lat, gps_lon);
  double dir_from_launch= TinyGPSPlus::courseTo(launch_lat, launch_lon, gps_lat, gps_lon);  //points from launch xy to rocket xy
    double xy_to_land= TinyGPSPlus::distanceBetween(gps_lat, gps_lon, land_lat, land_lon);
    double xy_dir_to_land= TinyGPSPlus::courseTo(gps_lat, gps_lon, launch_lat, launch_lon);
    double x_to_land= TinyGPSPlus::distanceBetween(0, gps_lon, 0, land_lon);
    double y_to_land= TinyGPSPlus::distanceBetween(gps_lat, 0, land_lat, 0);
  //the indented values will be logged but not sent
  smartDelay(500);
  
  // Downlink
  BEGIN_SEND
  SEND_VECTOR_ITEM(euler_angle  , euler);
  SEND_VECTOR_ITEM(gyro         , gyroscope);
  SEND_ITEM(temperature         , temp);
  SEND_VECTOR_ITEM(magnetometer , magnetometer);
  SEND_VECTOR_ITEM(acceleration , accelerometer);
  SEND_ITEM(bmp_alt             , bmp_alt);
  SEND_ITEM(gps_alt             , gps_alt);
  //SEND_ITEM(gps_lat             , gps_lat);
  TELEMETRY_SERIAL.print(F(";"));               
  TELEMETRY_SERIAL.print(F("gps_lat"));            
  TELEMETRY_SERIAL.print(F(":"));               
  TELEMETRY_SERIAL.print(gps_lat,8);//more digits of precision
  //SEND_ITEM(gps_lon             , gps_lon);
  TELEMETRY_SERIAL.print(F(";"));               
  TELEMETRY_SERIAL.print(F("gps_lon"));            
  TELEMETRY_SERIAL.print(F(":"));               
  TELEMETRY_SERIAL.print(gps_lon,10);//more digits of precision
  SEND_ITEM(gps_vel             , gps_vel);
  SEND_ITEM(gps_dir             , gps_dir);
  SEND_ITEM(xy_from_lanch       , xy_from_lanch);
  SEND_ITEM(dir_from_launch     , dir_from_launch);sats
  SEND_ITEM(sats                , sats);
  END_SEND
  
  // Writing to SD Card
  DateTime now = rtc.now();
  //writing abs time,sys date,sys time 
  dataFile.print(millis());           dataFile.print(',');
  dataFile.print(now.year()  ,DEC);   dataFile.print('/');
  dataFile.print(now.month() ,DEC);   dataFile.print('/');
  dataFile.print(now.day()   ,DEC);   dataFile.print(',');
  dataFile.print(now.hour()  ,DEC);   dataFile.print(':');
  dataFile.print(now.minute(),DEC);   dataFile.print(':');
  dataFile.print(now.second(),DEC);
  //writing sensor values
  WRITE_CSV_VECTOR_ITEM(euler)
  WRITE_CSV_VECTOR_ITEM(gyroscope)
  WRITE_CSV_ITEM(temp)
  WRITE_CSV_VECTOR_ITEM(magnetometer)
  WRITE_CSV_VECTOR_ITEM(accelerometer)
  WRITE_CSV_ITEM(bmp_alt)
  WRITE_CSV_ITEM(gps_alt)
  WRITE_CSV_ITEM(gps_lat)
  WRITE_CSV_ITEM(gps_lon)
  WRITE_CSV_ITEM(gps_vel)
  WRITE_CSV_ITEM(gps_dir)
  WRITE_CSV_ITEM(xy_from_lanch)
  WRITE_CSV_ITEM(dir_from_launch)
  WRITE_CSV_ITEM(xy_to_land)
  WRITE_CSV_ITEM(xy_dir_to_land)
  WRITE_CSV_ITEM(x_to_land)
  WRITE_CSV_ITEM(y_to_land)
  WRITE_CSV_ITEM(bmp_temp)
  WRITE_CSV_ITEM(bmp_pressure)
  WRITE_CSV_ITEM(sats)
  WRITE_CSV_ITEM(fix_hdop)
  dataFile.println();
  dataFile.flush();
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}
