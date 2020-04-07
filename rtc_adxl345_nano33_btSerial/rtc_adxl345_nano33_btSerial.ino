#include <WiFiNINA.h>
#include <WiFiUdp.h>
//#include "SparkFunLSM6DS3.h"
#include "Wire.h"
//#include <SPI.h>
//#include <time.h>
#include <ArduinoLowPower.h>
#include <SparkFun_ADXL345.h>
#include <math.h>
#include <FlashStorage.h>
#include <ThingSpeak.h>
#include "secrets.h"

#define SERIAL_BAUDRATE 9600  //serial baud rate
#define SERVER_TIME 00  //ThingSpeak sending time(minute-debug-)
#define ALARM_TIMING 60000  //vehicle active mode measure-alarm timing(millis-debug-)
#define ACCEL_RANGE 4 //accelerometer range setting value
#define ACT_THRESHOLD 20  //accelerometer activity occur threshold
//accel data rate : 3200 ~ 0.098 our target rate is 100,50,25,12.5,6.25,3.125,1.563
#define ACCEL_RATE 1.56  //accelerometer data rate setting value
#define SAMPLING_NUM 15.0 // number of sampling (about 12.8 s)
#define ACCEL_DELAY 640 //measurement sampling timing for 1.56 Hz
#define DEFINE_ACCEL 10.0  //active alarm mode condition
#define pin 15
#define BAT_PIN A3
/*
   accelerometer
*/
// LSM6DS3Core myIMU(I2C_MODE,0x6B);
ADXL345 adxl = ADXL345();   //I2C
//ADXL345 adxl = ADXL345(3);  // SPI

//const int pin = 15;  //accelerometer activity-wake up interrupt pin

typedef struct {
  float avg_svg;
  float svg_max;
} AccelData;

float base_accx, base_accy, base_accz;

void calibAccel();
void calculAccel(AccelData& accel);


/*
   Wifi connection & RTC
*/
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int keyIndex = 0;
WiFiClient client;
int status = WL_IDLE_STATUS;
unsigned long GMT = 9 * 60 * 60; //gap seconds between (seoul,South Korea) and GMT

void printWiFiStatus();
void resetEpoch();
void printEpoch();
void print2digits(int number);

/*
   ThingSpeak
*/
unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

#ifndef PRINT_DEBUG_MESSAGES
#define PRINT_DEBUG_MESSAGES
#endif

// Initialize ThingSpeak values
/*
  float number1 = 0;
  float number2 = 0;
  int number3 = 0;
*/
int number1 = 0;
int number2 = 0;
//String myStatus = "";
String timeStamp = "";

/*
   for read & write data to flash
*/

//for checking data packet contains
char buf[128];

//member of packet(time activity occured)
typedef struct timestamp {
  int g_day, g_month, g_year, g_hours, g_minutes, g_seconds;
} timestamp;

typedef struct EEPROMpacket {
  timestamp active_time;
  /*
    float svg_max;
    float avg_svg;
  */
  int active;
  int num;
} EEPROMpacket;

//instance for read & write packet to flash and number of written packet
EEPROMpacket EEPROMpkt;
EEPROMpacket writtenPacket[48];
int pkt_num;

FlashStorage(accel_data_store0, EEPROMpacket);
FlashStorage(accel_data_store1, EEPROMpacket);
FlashStorage(accel_data_store2, EEPROMpacket);
FlashStorage(accel_data_store3, EEPROMpacket);
FlashStorage(accel_data_store4, EEPROMpacket);
FlashStorage(accel_data_store5, EEPROMpacket);
FlashStorage(accel_data_store6, EEPROMpacket);
FlashStorage(accel_data_store7, EEPROMpacket);
FlashStorage(accel_data_store8, EEPROMpacket);
FlashStorage(accel_data_store9, EEPROMpacket);
FlashStorage(accel_data_store10, EEPROMpacket);
FlashStorage(accel_data_store11, EEPROMpacket);
FlashStorage(accel_data_store12, EEPROMpacket);
FlashStorage(accel_data_store13, EEPROMpacket);
FlashStorage(accel_data_store14, EEPROMpacket);
FlashStorage(accel_data_store15, EEPROMpacket);
FlashStorage(accel_data_store16, EEPROMpacket);
FlashStorage(accel_data_store17, EEPROMpacket);
FlashStorage(accel_data_store18, EEPROMpacket);
FlashStorage(accel_data_store19, EEPROMpacket);
FlashStorage(accel_data_store20, EEPROMpacket);
FlashStorage(accel_data_store21, EEPROMpacket);
FlashStorage(accel_data_store22, EEPROMpacket);
FlashStorage(accel_data_store23, EEPROMpacket);
FlashStorage(accel_data_store24, EEPROMpacket);
FlashStorage(accel_data_store25, EEPROMpacket);
FlashStorage(accel_data_store26, EEPROMpacket);
FlashStorage(accel_data_store27, EEPROMpacket);
FlashStorage(accel_data_store28, EEPROMpacket);
FlashStorage(accel_data_store29, EEPROMpacket);
FlashStorage(accel_data_store30, EEPROMpacket);
FlashStorage(accel_data_store31, EEPROMpacket);
FlashStorage(accel_data_store32, EEPROMpacket);
FlashStorage(accel_data_store33, EEPROMpacket);
FlashStorage(accel_data_store34, EEPROMpacket);
FlashStorage(accel_data_store35, EEPROMpacket);
FlashStorage(accel_data_store36, EEPROMpacket);
FlashStorage(accel_data_store37, EEPROMpacket);
FlashStorage(accel_data_store38, EEPROMpacket);
FlashStorage(accel_data_store39, EEPROMpacket);
FlashStorage(accel_data_store40, EEPROMpacket);
FlashStorage(accel_data_store41, EEPROMpacket);
FlashStorage(accel_data_store42, EEPROMpacket);
FlashStorage(accel_data_store43, EEPROMpacket);
FlashStorage(accel_data_store44, EEPROMpacket);
FlashStorage(accel_data_store45, EEPROMpacket);
FlashStorage(accel_data_store46, EEPROMpacket);
FlashStorage(accel_data_store47, EEPROMpacket);


void sendThingSpeak(int number);
void initFlash(int pktcnt);
int readPacketFromFlash();
void writePacketToFlash();
//void packetMake(float svg_max, float avg_svg);
void packetMake(int active);
void getHighActiveTime();

/*
   interrupt flag (time to sending data flag, accel activity occur flag, alarm mode(vehicle active-state) flag )
*/
volatile bool time_flag = false;  //24 o'clock RTC wake up interrupt flag (for sending data)
volatile bool accel_flag = false; //accelerometer activity-wake up interrupt flag
//volatile bool inact_flag = false;
volatile bool setActiveAlarm_flag = false;  //vehicle active-state flag


/***********************function definition ******************************/

/*
   for accelerometer calibration, calculation
*/

void calibAccel() {

  int x, y, z;
  int32_t sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  adxl.setRate(ACCEL_RATE);
  delay(ACCEL_DELAY);
  for (int i = 0; i < 10; i++) {

    adxl.readAccel(&x, &y, &z);
    sumAcX += x;
    sumAcY += y;
    sumAcZ += z;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(ACCEL_DELAY / 2);
    digitalWrite(LED_BUILTIN, LOW);
    delay(ACCEL_DELAY / 2);
  }

  base_accx = sumAcX / 10;
  base_accy = sumAcY / 10;
  base_accz = sumAcZ / 10;


  //  for debugging
  Serial1.print("base_accx :");
  Serial1.println(base_accx);
  Serial1.print("base_accy :");
  Serial1.println(base_accy);
  Serial1.print("base_accz :");
  Serial1.println(base_accz);
}

void calculAccel(AccelData& accel) {
  // calibAccel();
  int x, y, z;
  float cal_x, cal_y, cal_z;

  float total_svg = 0;
  float svg_acc = 0;
  float avg_svg = 0;
  float svg_max = 0;

  //  adxl.setRate(ACCEL_RATE);
  delay(ACCEL_DELAY);

  //1.563Hz sampling
  for (int i = 0; i < SAMPLING_NUM ; i++) {

    adxl.readAccel(&x, &y, &z);

    cal_x = (float)x - base_accx;
    cal_y = (float)y - base_accy;
    cal_z = (float)z - base_accz;

    svg_acc = sqrt(pow(cal_x, 2.0) + pow(cal_y, 2.0) + pow(cal_z, 2.0));


    // Output Results to Serial

    Serial1.print(cal_x);
    Serial1.print(", ");
    Serial1.print(cal_y);
    Serial1.print(", ");
    Serial1.println(cal_z);

    Serial1.print("svg= ");
    Serial1.println(svg_acc);


    if (svg_acc > svg_max) {
      svg_max = svg_acc;
    }

    total_svg += svg_acc;

    digitalWrite(LED_BUILTIN, HIGH);
    delay(ACCEL_DELAY / 2);
    digitalWrite(LED_BUILTIN, LOW);
    delay(ACCEL_DELAY / 2);
  }

  avg_svg = total_svg / SAMPLING_NUM;

  Serial1.println("------------------");
  Serial1.print("avg_svg =");
  Serial1.println(avg_svg);
  Serial1.print("svg_max=");
  Serial1.println(svg_max);
  Serial1.println("------------------");

  //delay(500);
  accel.avg_svg = avg_svg;
  accel.svg_max = svg_max;

}

void sendThingSpeakOnce(int active, int num) {
  // int i;
  number1 = active;
  number2 = num;

  ThingSpeak.setField(1, number1);
  ThingSpeak.setField(2, number2);

  Serial1.println("ThingSpeak ready OK.");

  //  for (i = 0; x != 200 && i < 10 ; i++) {
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial1.println("Channel update successful.");
  }
  else {
    Serial1.println("Problem updating channel. HTTP error code ");
  }
  /*
    }
    if (i > 9) {
    Serial.println("packet update failed.");
    } else {
    Serial.println("packet update success.");
    }
  */
  Serial1.println();
}

void sendVoltageStateThingSpeak() {
  digitalWrite(LED_BUILTIN, HIGH);
  int j, x, y;
  int battery = readBattery();
  float voltage = ((float)map(battery, 0, 4096, 0, 60)) / 10.0;
  Serial1.print(" battery voltage : ");
  Serial1.println(voltage);
  sprintf(buf, "readValue : %d, battery voltage : %0.2f V", battery, voltage);
  String myStatus = String(buf);
  ThingSpeak.setStatus(myStatus);
  x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial1.println("Channel update successful.");
  }
  else {
    Serial1.println("Problem updating channel. HTTP error code ");
  }
  memset(buf, 0, sizeof(buf));
  delay(20000);
  digitalWrite(LED_BUILTIN, LOW);
}

void sendThingSpeak(int number) {
  int j, x, y;

  for (j = 0; j < number; j++) {
    digitalWrite(LED_BUILTIN, HIGH);
    //ex> "2017-01-12 13:22:54"
    sprintf(buf, "%d-%d-%d %d:%d:%d", 2000 + (writtenPacket[j].active_time.g_year), writtenPacket[j].active_time.g_month, writtenPacket[j].active_time.g_day, writtenPacket[j].active_time.g_hours, writtenPacket[j].active_time.g_minutes, writtenPacket[j].active_time.g_seconds);
    Serial1.println(buf);
    timeStamp = String(buf);
    /*
        number1 = writtenPacket[j].svg_max;
        number2 = writtenPacket[j].avg_svg;
        number3 = writtenPacket[j].num;
    */
    number1 = writtenPacket[j].active;
    number2 = writtenPacket[j].num;



    ThingSpeak.setField(1, number1);
    ThingSpeak.setField(2, number2);
    //    ThingSpeak.setField(3, number3);


    y = ThingSpeak.setCreatedAt(timeStamp);

    if (y == 200) {
      Serial1.println("Timestamp setting OK.");
    } else {
      Serial1.println("Timestamp is too long, or other error.");
    }

    Serial1.println("ThingSpeak ready OK.");

    //    for (i = 0; x != 200 && i < 10 ; i++) {
    x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200) {
      Serial1.println("Channel update successful.");
    }
    else {
      Serial1.println("Problem updating channel. HTTP error code ");
    }
    /*
      }
      if (i > 9) {
      Serial.print(j);
      Serial.println("-th packet update failed.");
      } else {
      Serial.print(j);
      Serial.println("-th packet update success.");
      }
      i = 0;
    */

    Serial1.println();
    memset(buf, 0, sizeof(buf));
    delay(20000); //thingSpeak free update time gap
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial1.println("all data are sent to thingSpeak channel.");
}

//erase all written data in flash
void initFlash(int pktcnt) {
  int i;
  for (i = 0; i < pktcnt; i++) {
    switch (i) {
      case 0 :
        accel_data_store0.flash.erase();
        break;
      case 1:
        accel_data_store1.flash.erase();
        break;
      case 2:
        accel_data_store2.flash.erase();
        break;
      case 3:
        accel_data_store3.flash.erase();
        break;
      case 4:
        accel_data_store4.flash.erase();
        break;
      case 5:
        accel_data_store5.flash.erase();
        break;
      case 6:
        accel_data_store6.flash.erase();
        break;
      case 7:
        accel_data_store7.flash.erase();
        break;
      case 8:
        accel_data_store8.flash.erase();
        break;
      case 9:
        accel_data_store9.flash.erase();
        break;
      case 10 :
        accel_data_store10.flash.erase();
        break;
      case 11:
        accel_data_store11.flash.erase();
        break;
      case 12:
        accel_data_store12.flash.erase();
        break;
      case 13:
        accel_data_store13.flash.erase();
        break;
      case 14:
        accel_data_store14.flash.erase();
        break;
      case 15:
        accel_data_store15.flash.erase();
        break;
      case 16:
        accel_data_store16.flash.erase();
        break;
      case 17:
        accel_data_store17.flash.erase();
        break;
      case 18:
        accel_data_store18.flash.erase();
        break;
      case 19:
        accel_data_store19.flash.erase();
        break;
      case 20 :
        accel_data_store20.flash.erase();
        break;
      case 21:
        accel_data_store21.flash.erase();
        break;
      case 22:
        accel_data_store22.flash.erase();
        break;
      case 23:
        accel_data_store23.flash.erase();
        break;
      case 24:
        accel_data_store24.flash.erase();
        break;
      case 25:
        accel_data_store25.flash.erase();
        break;
      case 26:
        accel_data_store26.flash.erase();
        break;
      case 27:
        accel_data_store27.flash.erase();
        break;
      case 28:
        accel_data_store28.flash.erase();
        break;
      case 29:
        accel_data_store29.flash.erase();
        break;
      case 30 :
        accel_data_store30.flash.erase();
        break;
      case 31:
        accel_data_store31.flash.erase();
        break;
      case 32:
        accel_data_store32.flash.erase();
        break;
      case 33:
        accel_data_store33.flash.erase();
        break;
      case 34:
        accel_data_store34.flash.erase();
        break;
      case 35:
        accel_data_store35.flash.erase();
        break;
      case 36:
        accel_data_store36.flash.erase();
        break;
      case 37:
        accel_data_store37.flash.erase();
        break;
      case 38:
        accel_data_store38.flash.erase();
        break;
      case 39:
        accel_data_store39.flash.erase();
        break;
      case 40 :
        accel_data_store40.flash.erase();
        break;
      case 41:
        accel_data_store41.flash.erase();
        break;
      case 42:
        accel_data_store42.flash.erase();
        break;
      case 43:
        accel_data_store43.flash.erase();
        break;
      case 44:
        accel_data_store44.flash.erase();
        break;
      case 45:
        accel_data_store45.flash.erase();
        break;
      case 46:
        accel_data_store46.flash.erase();
        break;
      case 47:
        accel_data_store47.flash.erase();
        break;
    }
  }
  Serial1.println("all data in flash was erased successly.");
  pkt_num = 0;
}


//read all written data from flash
int readPacketFromFlash() {

  int i = 0;
  while (i < pkt_num) {

    switch (i) {
      case 0 :
        accel_data_store0.read(&(writtenPacket[i]));
        break;
      case 1:
        accel_data_store1.read(&(writtenPacket[i]));
        break;
      case 2:
        accel_data_store2.read(&(writtenPacket[i]));
        break;
      case 3:
        accel_data_store3.read(&(writtenPacket[i]));
        break;
      case 4:
        accel_data_store4.read(&(writtenPacket[i]));
        break;
      case 5:
        accel_data_store5.read(&(writtenPacket[i]));
        break;
      case 6:
        accel_data_store6.read(&(writtenPacket[i]));
        break;
      case 7:
        accel_data_store7.read(&(writtenPacket[i]));
        break;
      case 8:
        accel_data_store8.read(&(writtenPacket[i]));
        break;
      case 9:
        accel_data_store9.read(&(writtenPacket[i]));
        break;
      case 10 :
        accel_data_store10.read(&(writtenPacket[i]));
        break;
      case 11:
        accel_data_store11.read(&(writtenPacket[i]));
        break;
      case 12:
        accel_data_store12.read(&(writtenPacket[i]));
        break;
      case 13:
        accel_data_store13.read(&(writtenPacket[i]));
        break;
      case 14:
        accel_data_store14.read(&(writtenPacket[i]));
        break;
      case 15:
        accel_data_store15.read(&(writtenPacket[i]));
        break;
      case 16:
        accel_data_store16.read(&(writtenPacket[i]));
        break;
      case 17:
        accel_data_store17.read(&(writtenPacket[i]));
        break;
      case 18:
        accel_data_store18.read(&(writtenPacket[i]));
        break;
      case 19:
        accel_data_store19.read(&(writtenPacket[i]));
        break;
      case 20 :
        accel_data_store20.read(&(writtenPacket[i]));
        break;
      case 21:
        accel_data_store21.read(&(writtenPacket[i]));
        break;
      case 22:
        accel_data_store22.read(&(writtenPacket[i]));
        break;
      case 23:
        accel_data_store23.read(&(writtenPacket[i]));
        break;
      case 24:
        accel_data_store24.read(&(writtenPacket[i]));
        break;
      case 25:
        accel_data_store25.read(&(writtenPacket[i]));
        break;
      case 26:
        accel_data_store26.read(&(writtenPacket[i]));
        break;
      case 27:
        accel_data_store27.read(&(writtenPacket[i]));
        break;
      case 28:
        accel_data_store28.read(&(writtenPacket[i]));
        break;
      case 29:
        accel_data_store29.read(&(writtenPacket[i]));
        break;
      case 30 :
        accel_data_store30.read(&(writtenPacket[i]));
        break;
      case 31:
        accel_data_store31.read(&(writtenPacket[i]));
        break;
      case 32:
        accel_data_store32.read(&(writtenPacket[i]));
        break;
      case 33:
        accel_data_store33.read(&(writtenPacket[i]));
        break;
      case 34:
        accel_data_store34.read(&(writtenPacket[i]));
        break;
      case 35:
        accel_data_store35.read(&(writtenPacket[i]));
        break;
      case 36:
        accel_data_store36.read(&(writtenPacket[i]));
        break;
      case 37:
        accel_data_store37.read(&(writtenPacket[i]));
        break;
      case 38:
        accel_data_store38.read(&(writtenPacket[i]));
        break;
      case 39:
        accel_data_store39.read(&(writtenPacket[i]));
        break;
      case 40 :
        accel_data_store40.read(&(writtenPacket[i]));
        break;
      case 41:
        accel_data_store41.read(&(writtenPacket[i]));
        break;
      case 42:
        accel_data_store42.read(&(writtenPacket[i]));
        break;
      case 43:
        accel_data_store43.read(&(writtenPacket[i]));
        break;
      case 44:
        accel_data_store44.read(&(writtenPacket[i]));
        break;
      case 45:
        accel_data_store45.read(&(writtenPacket[i]));
        break;
      case 46:
        accel_data_store46.read(&(writtenPacket[i]));
        break;
      case 47:
        accel_data_store47.read(&(writtenPacket[i]));
        break;
    }

    Serial1.print("read packet number: ");
    Serial1.println(writtenPacket[i].num);
    sprintf(buf, "%d-%d-%d %d:%d:%d vehicle state :%d", 2000 + (writtenPacket[i].active_time.g_year), writtenPacket[i].active_time.g_month, writtenPacket[i].active_time.g_day, writtenPacket[i].active_time.g_hours, writtenPacket[i].active_time.g_minutes, writtenPacket[i].active_time.g_seconds, writtenPacket[i].active);
    Serial1.println(buf);
    i++;
  }

  Serial1.print(i);
  Serial1.println(" packets in flash were Read Successly.");

  return i;
}

//write one data(acceleration & time) to flash
void writePacketToFlash() {

  switch (pkt_num) {
    case 0 :
      accel_data_store0.write(EEPROMpkt);
      break;
    case 1:
      accel_data_store1.write(EEPROMpkt);
      break;
    case 2:
      accel_data_store2.write(EEPROMpkt);
      break;
    case 3:
      accel_data_store3.write(EEPROMpkt);
      break;
    case 4:
      accel_data_store4.write(EEPROMpkt);
      break;
    case 5:
      accel_data_store5.write(EEPROMpkt);
      break;
    case 6:
      accel_data_store6.write(EEPROMpkt);
      break;
    case 7:
      accel_data_store7.write(EEPROMpkt);
      break;
    case 8:
      accel_data_store8.write(EEPROMpkt);
      break;
    case 9:
      accel_data_store9.write(EEPROMpkt);
      break;
    case 10 :
      accel_data_store10.write(EEPROMpkt);
      break;
    case 11:
      accel_data_store11.write(EEPROMpkt);
      break;
    case 12:
      accel_data_store12.write(EEPROMpkt);
      break;
    case 13:
      accel_data_store13.write(EEPROMpkt);
      break;
    case 14:
      accel_data_store14.write(EEPROMpkt);
      break;
    case 15:
      accel_data_store15.write(EEPROMpkt);
      break;
    case 16:
      accel_data_store16.write(EEPROMpkt);
      break;
    case 17:
      accel_data_store17.write(EEPROMpkt);
      break;
    case 18:
      accel_data_store18.write(EEPROMpkt);
      break;
    case 19:
      accel_data_store19.write(EEPROMpkt);
      break;
    case 20 :
      accel_data_store20.write(EEPROMpkt);
      break;
    case 21:
      accel_data_store21.write(EEPROMpkt);
      break;
    case 22:
      accel_data_store22.write(EEPROMpkt);
      break;
    case 23:
      accel_data_store23.write(EEPROMpkt);
      break;
    case 24:
      accel_data_store24.write(EEPROMpkt);
      break;
    case 25:
      accel_data_store25.write(EEPROMpkt);
      break;
    case 26:
      accel_data_store26.write(EEPROMpkt);
      break;
    case 27:
      accel_data_store27.write(EEPROMpkt);
      break;
    case 28:
      accel_data_store28.write(EEPROMpkt);
      break;
    case 29:
      accel_data_store29.write(EEPROMpkt);
      break;
    case 30 :
      accel_data_store30.write(EEPROMpkt);
      break;
    case 31:
      accel_data_store31.write(EEPROMpkt);
      break;
    case 32:
      accel_data_store32.write(EEPROMpkt);
      break;
    case 33:
      accel_data_store33.write(EEPROMpkt);
      break;
    case 34:
      accel_data_store34.write(EEPROMpkt);
      break;
    case 35:
      accel_data_store35.write(EEPROMpkt);
      break;
    case 36:
      accel_data_store36.write(EEPROMpkt);
      break;
    case 37:
      accel_data_store37.write(EEPROMpkt);
      break;
    case 38:
      accel_data_store38.write(EEPROMpkt);
      break;
    case 39:
      accel_data_store39.write(EEPROMpkt);
      break;
    case 40 :
      accel_data_store40.write(EEPROMpkt);
      break;
    case 41:
      accel_data_store41.write(EEPROMpkt);
      break;
    case 42:
      accel_data_store42.write(EEPROMpkt);
      break;
    case 43:
      accel_data_store43.write(EEPROMpkt);
      break;
    case 44:
      accel_data_store44.write(EEPROMpkt);
      break;
    case 45:
      accel_data_store45.write(EEPROMpkt);
      break;
    case 46:
      accel_data_store46.write(EEPROMpkt);
      break;
    case 47:
      accel_data_store47.write(EEPROMpkt);
      break;
    default:
      Serial1.println("Not enough memory space to use.");
      Serial1.println("Initialize the flash address.");
      initFlash(pkt_num - 1);
      pkt_num = 0;
      accel_data_store0.write(EEPROMpkt);
      break;
  }

  Serial1.println("Flash Writing Process Success.");

  pkt_num++;
}

//make data with packet time & accel data
/*
  void packetMake(float svg_max, float avg_svg) {
  getHighActiveTime();
  Serial.println("written packet time : ");
  Serial.println(EEPROMpkt.active_time.g_day);
  Serial.println(EEPROMpkt.active_time.g_month);
  Serial.println(EEPROMpkt.active_time.g_year);
  Serial.println(EEPROMpkt.active_time.g_hours);
  Serial.println(EEPROMpkt.active_time.g_minutes);
  Serial.println(EEPROMpkt.active_time.g_seconds);
  Serial.println("------------------");
  EEPROMpkt.svg_max = svg_max;
  EEPROMpkt.avg_svg = avg_svg;
  EEPROMpkt.num = pkt_num;


  Serial.println("written packet data : ");
  Serial.println(EEPROMpkt.svg_max);
  Serial.println(EEPROMpkt.avg_svg);
  Serial.println(EEPROMpkt.num);
  Serial.println("============");
  Serial.println("Packet Make Success");
  }
*/
void packetMake(int active) {
  getHighActiveTime();
  /*
    Serial.println("written packet time : ");
    Serial.println(EEPROMpkt.active_time.g_day);
    Serial.println(EEPROMpkt.active_time.g_month);
    Serial.println(EEPROMpkt.active_time.g_year);
    Serial.println(EEPROMpkt.active_time.g_hours);
    Serial.println(EEPROMpkt.active_time.g_minutes);
    Serial.println(EEPROMpkt.active_time.g_seconds);
  */
  Serial1.println("------------------");
  EEPROMpkt.active = active;
  EEPROMpkt.num = pkt_num;


  Serial1.println("written packet data : ");
  Serial1.println(EEPROMpkt.active);
  Serial1.println(EEPROMpkt.num);
  Serial1.println("============");
  Serial1.println("Packet Make Success");
}

//get present time
void getHighActiveTime() {
  EEPROMpkt.active_time.g_day = LowPower.rtc.getDay();
  EEPROMpkt.active_time.g_month = LowPower.rtc.getMonth();
  EEPROMpkt.active_time.g_year = LowPower.rtc.getYear();
  EEPROMpkt.active_time.g_hours = LowPower.rtc.getHours();
  EEPROMpkt.active_time.g_minutes = LowPower.rtc.getMinutes();
  EEPROMpkt.active_time.g_seconds = LowPower.rtc.getSeconds();
  Serial1.println("Timestamp Write Success");
}

int readBattery() {
  int readValue = analogRead(BAT_PIN);
  Serial1.print("reading value is ");
  Serial1.println(readValue);

  return readValue;
}
/*****************************setup()*********************************/
void setup() {
  while (!Serial1);
  Serial1.begin(SERIAL_BAUDRATE);
  pinMode(pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  pkt_num = 0;

  /*
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      while (1);  //breakout
    }
  */

  while (status != WL_CONNECTED) {
    Serial1.print("Attempting to connect to SSID: ");
    Serial1.println(ssid);

    status = WiFi.begin(ssid, pass);

    delay(5000);
  }

  printWiFiStatus();
  ThingSpeak.begin(client);

  LowPower.rtc.begin();

  resetEpoch(); //getting present time epoch

  LowPower.rtc.setAlarmMinutes(SERVER_TIME);  //setting alarm time every hours
  LowPower.rtc.enableAlarm(LowPower.rtc.MATCH_MMSS);
  LowPower.rtc.attachInterrupt(onTimeFlag);  //alarm interrupt wake up setting

  //accelerometer initial seqeunce timing value
  delayMicroseconds(1100);
  //accelerometer setting
  adxl.powerOn();
  //adxl.writeTo(ADXL345_POWER_CTL, 24);  //auto sleep mode, measure state
  //adxl.writeTo(0x2E,0x00); //enable DATA_READY Interrupt

  adxl.setRangeSetting(ACCEL_RANGE);  // range settings : Accepted values are 2g, 4g, 8g or 16g
  //  adxl.setRate(ACCEL_RATE);  // data rate setting : 1.563 Hz
  //adxl.setSpiBit(1);

  adxl.setActivityXYZ(1, 1, 1); // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(ACT_THRESHOLD);  // 62.5mg per increment   // Set activity   // activity thresholds (0-255)

  adxl.setInactivityXYZ(0, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)

  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(20);

  adxl.setTapDetectionOnXYZ(0, 0, 0);

  adxl.setInterruptLevelBit(1);   //set INT pin HIGH (active:low, non:high)
  //adxl.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, ADXL345_INT1_PIN);

  adxl.InactivityINT(0);
  adxl.ActivityINT(1);  //Turn on Interrupts for Activity mode(1 == ON, 0 == OFF)
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);

  calibAccel();
  delay(100);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(pin), onAccelFlag, FALLING);

  Serial1.println("mcu is going to sleep.. ");
  Serial1.println();

  delay(100);
  //  Serial1.end();
  LowPower.sleep();
}

/**********************************loop()******************************/
void loop() {
//  Serial1.begin(SERIAL_BAUDRATE);
  //  while (!Serial);
  Serial1.println("wake up!");

  //at 24 o'clock, sending data to server via AP
  if (time_flag) {
    Serial1.println("onTime interrupt generated!");
    //    resetEpoch();
    time_flag = false;

    sendVoltageStateThingSpeak();
    if (pkt_num > 0) {
      int num = readPacketFromFlash();

      if (num == pkt_num) {
        Serial1.println("all written packets were read from flash Successly.");
      }

      sendThingSpeak(pkt_num);
      delay(100);
      initFlash(pkt_num);
      delay(100);
    }
    adxl.setActivityXYZ(1, 1, 1);
    adxl.ActivityINT(1);

    //attachInterrupt(digitalPinToInterrupt(pin), onAccelFlag, FALLING);
    delay(100);
    LowPower.attachInterruptWakeup(digitalPinToInterrupt(pin), onAccelFlag, FALLING);
  }

  //if activity interrupt signal occur from adxl345, accel_flag is true
  else if (accel_flag) {

    accel_flag = false;
    AccelData accel;
    calculAccel(accel);
    //  packetMake(accel.svg_max, accel.avg_svg);
    //  writePacketToFlash();
    delay(100);

    //Determine whether the average accel-value is higher or lower than the reference value,
    //if lower, go "just normal mode" : 24'o clock alarm setting and waiting for activity INT
    //if higher,go "active-alarm setting mode" : every 30 minutes wake-up setting, and estimate acceleration
    if (accel.avg_svg < DEFINE_ACCEL) {
      //sendThingSpeakOnce(0, pkt_num);
      packetMake(0);
      delay(100);
      writePacketToFlash();
      delay(100);

      Serial1.print(accel.avg_svg);
      Serial1.print(" is < ");
      Serial1.println(DEFINE_ACCEL);
      Serial1.println("just normal mode.");

      delay(100);
      LowPower.rtc.attachInterrupt(onTimeFlag);
      adxl.setActivityXYZ(1, 1, 1);
      adxl.ActivityINT(1);
      LowPower.attachInterruptWakeup(digitalPinToInterrupt(pin), onAccelFlag, FALLING);

    } else {
      //sendThingSpeakOnce(1, pkt_num);
      packetMake(1);
      delay(100);
      writePacketToFlash();
      delay(100);

      Serial1.print(accel.avg_svg);
      Serial1.print(" is > ");
      Serial1.println(DEFINE_ACCEL);
      Serial1.print("Active ");
      Serial1.print(ALARM_TIMING / 1000);
      Serial1.println("sec alarm ON set");

      delay(100);
      LowPower.rtc.disableAlarm();
      /*
        LowPower.rtc.setAlarmSeconds(30);
        LowPower.rtc.enableAlarm(LowPower.rtc.MATCH_SS);
      */
      LowPower.setAlarmIn(ALARM_TIMING);
      //LowPower.rtc.begin(false);
      LowPower.rtc.attachInterrupt(onHighFlag);
    }
  }//end of accel_flag

  //if vehicle is running mode, setActiveAlarm_flag is true
  else if (setActiveAlarm_flag) {

    setActiveAlarm_flag = false;

    AccelData accel;
    calculAccel(accel);
    delay(100);
    //    packetMake(accel.svg_max, accel.avg_svg);
    //    writePacketToFlash();

    if (accel.avg_svg > DEFINE_ACCEL) {
      // sendThingSpeakOnce(1, pkt_num);
      packetMake(1);
      delay(100);
      writePacketToFlash();
      delay(100);

      Serial1.print(accel.avg_svg);
      Serial1.print(" is > ");
      Serial1.println(DEFINE_ACCEL);
      Serial1.print("and ");
      Serial1.print(ALARM_TIMING / 1000);
      Serial1.println("sec alarm is ON constantly.");

      delay(100);
      LowPower.setAlarmIn(ALARM_TIMING);
      //LowPower.rtc.begin(false);
      LowPower.rtc.attachInterrupt(onHighFlag);
    }
    else {
      // sendThingSpeakOnce(0, pkt_num);
      packetMake(0);
      delay(100);
      writePacketToFlash();
      delay(100);

      Serial1.print(accel.avg_svg);
      Serial1.println(" is < ");
      Serial1.println(DEFINE_ACCEL);
      Serial1.println("Last written.");
      Serial1.println("and alarm OFF set.");

      LowPower.rtc.detachInterrupt();
      LowPower.rtc.disableAlarm();
      LowPower.rtc.setAlarmMinutes(SERVER_TIME);
      LowPower.rtc.enableAlarm(LowPower.rtc.MATCH_MMSS);

      delay(100);
      LowPower.rtc.attachInterrupt(onTimeFlag);
      adxl.setActivityXYZ(1, 1, 1);
      adxl.ActivityINT(1);
      LowPower.attachInterruptWakeup(digitalPinToInterrupt(pin), onAccelFlag, FALLING);

    }
  }//end of setActiveAlarm_flag
  Serial1.println("mcu is going to sleep.. ");
  Serial1.println();
  //Serial1.end();
  LowPower.sleep();
}

/************************Interrupt & Alarm ISR ****************************/

void onHighFlag() {
  setActiveAlarm_flag = true;
}

void onTimeFlag() {
  time_flag = true;
  adxl.ActivityINT(0);
  adxl.setActivityXYZ(0, 0, 0);
  detachInterrupt(pin);
}

void onAccelFlag() {
  LowPower.rtc.detachInterrupt();
  adxl.ActivityINT(0);
  adxl.setActivityXYZ(0, 0, 0);
  detachInterrupt(pin);

  byte interrupts = adxl.getInterruptSource();
  if (adxl.triggered(interrupts, ADXL345_ACTIVITY)) {

    accel_flag = true;
  }
}

/************************ RTC getting WiFi epoch *************************/

void resetEpoch() {
  unsigned long epoch;
  int numberOfTries = 0, maxTries = 10;
  do {
    epoch = WiFi.getTime();
    numberOfTries++;

    if (numberOfTries >= maxTries) {
      Serial1.println("NTP unreachable!");
      numberOfTries = 0;
      delay(2000);
    }
  } while (epoch == 0);

  Serial1.print("Epoch received : ");
  Serial1.println(epoch);
  //    LowPower.rtc.setEpoch(epoch + GMT);
  LowPower.rtc.setEpoch(epoch);
  printEpoch();
}

void print2digits(int number) {
  if (number < 10) {
    Serial1.print("0"); // print a 0 before if the number is < than 10
  }
  Serial1.print(number);
}

void printWiFiStatus() {
  Serial1.print("SSID : ");
  Serial1.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial1.print("IP Address : ");
  Serial1.println(ip);

  long rssi = WiFi.RSSI();
  Serial1.print("signal strength (RSSI) : ");
  Serial1.print(rssi);
  Serial1.println( "dBm");
}

void printEpoch() {
  Serial1.println("-----------------");
  Serial1.println("set WiFi RTC");
  print2digits(LowPower.rtc.getDay());
  Serial1.print("/");
  print2digits(LowPower.rtc.getMonth());
  Serial1.print("/");
  print2digits(LowPower.rtc.getYear());
  Serial1.print(" ");
  // ...and time
  print2digits(LowPower.rtc.getHours());
  Serial1.print(":");
  print2digits(LowPower.rtc.getMinutes());
  Serial1.print(":");
  print2digits(LowPower.rtc.getSeconds());
  Serial1.println();
}
