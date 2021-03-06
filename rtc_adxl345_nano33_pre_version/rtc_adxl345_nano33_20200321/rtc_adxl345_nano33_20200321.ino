#include <WiFiNINA.h>
#include <WiFiUdp.h>
//#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include <SPI.h>
//#include <time.h>
#include <ArduinoLowPower.h>
#include <SparkFun_ADXL345.h>
#include <math.h>
#include <FlashStorage.h>
#include <ThingSpeak.h>
#include "secrets.h"

/*
 * accelerometer data calculation value 
 */
#define DEFINE_ACCEL 3.0
#define SAMPLING_NUM 20.0

/*
 * Wifi connection value
 */
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int keyIndex = 0;
WiFiClient client;
int status = WL_IDLE_STATUS;

/*
 * ThingSpeak value
 */
unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

// Initialize ThingSpeak values
float number1 = 0;
float number2 = 0;
int number3 = 0;
String myStatus = "";

unsigned long GMT = 9 * 60 * 60; //gap seconds between (seoul,South Korea) and GMT
const int pin = 2;  //accelerometer activity-wake up interrupt generation pin

//accelerometer
// LSM6DS3Core myIMU(I2C_MODE,0x6B);  
ADXL345 adxl = ADXL345();   //I2C
//ADXL345 adxl = ADXL345(3);  // SPI

/*
 * interrupt flag (time to sending data flag, accel activity occur flag, alarm mode(vehicle active-state) flag )   
 */
volatile bool time_flag = false;  //24 o'clock RTC wake up interrupt flag (for sending data)
volatile bool accel_flag = false; //accelerometer activity-wake up interrupt flag
//volatile bool inact_flag = false;
volatile bool setActiveAlarm_flag = false;  //vehicle active-state flag 


/*
 * for read * write data to flash
 */

//for checking data packet contains
char buf[64];

//member of packet(time activity occured) 
typedef struct timestamp {
  int g_day, g_month, g_year, g_hours, g_minutes, g_seconds;
} timestamp;

typedef struct EEPROMpacket {
  timestamp active_time;
  float svg_max;
  float avg_svg;
  int num;
} EEPROMpacket;

//instance for read & write packet to flash and number of written packet
EEPROMpacket EEPROMpkt;
EEPROMpacket writtenPacket[48];
volatile int pkt_num;

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

void sendThingSpeak(int number) {
  int j;
  for (j = 0; j < number; j++) {
    sprintf(buf, "%d/%d/%d/%d:%d:%d,max_svg: %f avg_svg: %f\n", writtenPacket[j].active_time.g_day, writtenPacket[j].active_time.g_month, writtenPacket[j].active_time.g_year, writtenPacket[j].active_time.g_hours, writtenPacket[j].active_time.g_minutes, writtenPacket[j].active_time.g_seconds, writtenPacket[j].svg_max, writtenPacket[j].avg_svg);
    Serial.println(buf);
    myStatus = String(buf);

    number1 = writtenPacket[j].svg_max;
    number2 = writtenPacket[j].avg_svg;
    number3 = writtenPacket[j].num;

    ThingSpeak.setField(1, number1);
    ThingSpeak.setField(2, number2);
    ThingSpeak.setField(3, number3);

    ThingSpeak.setStatus(myStatus);

    Serial.println("ThingSpeak ready OK.");
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200) {
      Serial.println("Channel update successful.");
    }
    else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }

    delay(20000);

  }
  Serial.println("all data are sent to thingSpeak channel.");
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
  Serial.println("all data in flash was erased successly.");
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

    Serial.print("read packet number: ");
    Serial.println(writtenPacket[i].num);
    sprintf(buf, "%d/%d/%d/%d:%d:%d,max_svg: %f avg_svg: %f\n", writtenPacket[i].active_time.g_day, writtenPacket[i].active_time.g_month, writtenPacket[i].active_time.g_year, writtenPacket[i].active_time.g_hours, writtenPacket[i].active_time.g_minutes, writtenPacket[i].active_time.g_seconds, writtenPacket[i].svg_max, writtenPacket[i].avg_svg);
    Serial.println(buf);
    i++;
  }

  Serial.print(i);
  Serial.println(" packets in flash were Read Successly.");

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
      Serial.println("Not enough memory space to use.");
      Serial.println("Initialize the flash address.");
      initFlash(pkt_num - 1);
      pkt_num = 0;
      accel_data_store0.write(EEPROMpkt);
      break;
  }

  Serial.println("Flash Writing Process Success.");

  pkt_num++;
}

//make data with packet time & accel data 
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

//get present time
void getHighActiveTime() {
  EEPROMpkt.active_time.g_day = LowPower.rtc.getDay();
  EEPROMpkt.active_time.g_month = LowPower.rtc.getMonth();
  EEPROMpkt.active_time.g_year = LowPower.rtc.getYear();
  EEPROMpkt.active_time.g_hours = LowPower.rtc.getHours();
  EEPROMpkt.active_time.g_minutes = LowPower.rtc.getMinutes();
  EEPROMpkt.active_time.g_seconds = LowPower.rtc.getSeconds();
  Serial.println("Timestamp Write Success");
}

/*
 * for accelerometer calibration
 */
typedef struct accel{
  float avg_svg;
  float svg_max;
}
 
float base_accx, base_accy, base_accz;

void calibAccel() {
  int x, y, z;
  uint32_t sumAcX = 0, sumAcY = 0, sumAcZ = 0;

  for (int i = 0; i < 10; i++) {
    adxl.readAccel(&x, &y, &z);
    sumAcX += x;
    sumAcY += y;
    sumAcZ += z;
    delay(10);
  }

  base_accx = sumAcX / 10;
  base_accy = sumAcY / 10;
  base_accz = sumAcZ / 10;


  //  for debugging
  Serial.print("base_accx :");
  Serial.println(base_accx);
  Serial.print("base_accy :");
  Serial.println(base_accy);
  Serial.print("base_accz :");
  Serial.println(base_accz);

}

void determineAccel(accel *accel){
  
    calibAccel();

    int x, y, z;
    float cal_x = 0, cal_y = 0, cal_z = 0;

    uint32_t total_svg = 0;
    float svg_acc = 0;
    float avg_svg = 0;
    float svg_max = 0;


    //1.563Hz sampling
    for (int i = 0; i < SAMPLING_NUM ; i++) {

      adxl.readAccel(&x, &y, &z);

      cal_x = x - base_accx;
      cal_y = y - base_accy;
      cal_z = z - base_accz;

      svg_acc = sqrt(pow(cal_x, 2.0) + pow(cal_y, 2.0) + pow(cal_z, 2.0));

      if (svg_acc > svg_max) {
        svg_max = svg_acc;
      }

      total_svg += svg_acc;

      // Output Results to Serial
      /*
            Serial.print(cal_x);
            Serial.print(", ");
            Serial.print(cal_y);
            Serial.print(", ");
            Serial.println(cal_z);

            Serial.print("svg= ");
            Serial.println(svg_acc);
      */
      delay(630);
    }

    avg_svg = total_svg / SAMPLING_NUM;


    Serial.println("------------------");
    Serial.print("avg_svg =");
    Serial.println(avg_svg);
    Serial.print("svg_max=");
    Serial.println(svg_max);
    Serial.println("------------------");

    accel.avg_svg=avg_svg;
    accel.svg_max=svg_max;
}
/*****************************setup()*********************************/
void setup() {
  while (!Serial);
  Serial.begin(9600);
  pinMode(pin, INPUT_PULLUP);

  pkt_num = 0;

  /*
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      while (1);  //breakout
    }
  */

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);

    delay(5000);
  }

  printWiFiStatus();

  ThingSpeak.begin(client);

  LowPower.rtc.begin();

  resetEpoch(); //getting present time epoch

  LowPower.rtc.setAlarmMinutes(40);  //setting alarm time every hours
  LowPower.rtc.enableAlarm(LowPower.rtc.MATCH_MMSS);
  LowPower.rtc.attachInterrupt(onTimeFlag);  //alarm interrupt wake up setting

  //accelerometer initial seqeunce timing value
  delayMicroseconds(1100);
  //accelerometer setting
  adxl.powerOn();
  //adxl.writeTo(ADXL345_POWER_CTL, 24);  //auto sleep mode, measure state
  //adxl.writeTo(0x2E,0x00); //enable DATA_READY Interrupt

  adxl.setRangeSetting(8);  // range settings : Accepted values are 2g, 4g, 8g or 16g
  adxl.setRate(1.563);  // data rate setting : 1.563 Hz
  //adxl.setSpiBit(1);

  adxl.setActivityXYZ(0, 0, 1); // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);  // 62.5mg per increment   // Set activity   // activity thresholds (0-255)

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

  /*
    bool autoSleep_check=adxl.getRegisterBit(ADXL345_POWER_CTL,4);
    if(autoSleep_check){
    Serial.println("adxl345 auto- sleep is on");
    }else{
      Serial.println("adxl345 auto- sleep is off");
    }
  */
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(pin), onAccelFlag, FALLING);

  Serial.println("mcu is going to sleep.. ");
  Serial.println();
  Serial.end();
  LowPower.sleep();
}

/**********************************loop()******************************/
void loop() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("wake up!! active-mode");
  /*
    bool autoSleep_check=adxl.getRegisterBit(ADXL345_POWER_CTL,4);
    if(autoSleep_check){
    Serial.println("adxl345 auto- sleep is on");
    }else{
    Serial.println("adxl345 auto- sleep is off");
    }
  */

  //at 24 o'clock, sending data to server via AP
  if (time_flag) {
    Serial.println("onTime interrupt generated!");
    resetEpoch();
    time_flag = false;

    if (pkt_num > 0) {
      int num = readPacketFromFlash();

      if (num == pkt_num) {
        Serial.println("all written packets were read from flash Successly.");
      }

      sendThingSpeak(pkt_num);

      initFlash(pkt_num);
    }
    adxl.setActivityXYZ(0, 0, 1);
    adxl.ActivityINT(1);

    //attachInterrupt(digitalPinToInterrupt(pin), onAccelFlag, FALLING);
    LowPower.attachInterruptWakeup(digitalPinToInterrupt(pin), onAccelFlag, FALLING);
  }

  //if activity interrupt signal occur from adxl345, accel_flag is true 
  else if (accel_flag) {

    accel_flag = false;

    accel accel;

    determineAccel(&accel);

    //Determine whether the average accel-value is higher or lower than the reference value,
    //if lower, go "just normal mode" : 24'o clock alarm setting and waiting for activity INT
    //if higher,go "active-alarm setting mode" : every 30 minutes wake-up setting, and estimate acceleration
    if (accel.avg_svg < DEFINE_ACCEL) {
      Serial.print(acce.avg_svg);
      Serial.println(" is < 3.0");
      Serial.println("just normal mode.");
      LowPower.rtc.attachInterrupt(onTimeFlag);

      adxl.setActivityXYZ(0, 0, 1);
      adxl.ActivityINT(1);
      LowPower.attachInterruptWakeup(digitalPinToInterrupt(pin), onAccelFlag, FALLING);
      /*
            attachInterrupt(digitalPinToInterrupt(pin), onAccelFlag, FALLING);

            GCLK->CLKCTRL.bit.CLKEN = 0; //disable GCLK module
            while (GCLK->STATUS.bit.SYNCBUSY);

            GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID( GCM_EIC )) ;  //EIC clock switched on GCLK6
            while (GCLK->STATUS.bit.SYNCBUSY);

            GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(6));  //source for GCLK6 is OSCULP32K
            while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

            GCLK->GENCTRL.bit.RUNSTDBY = 1;  //GCLK6 run standby
            while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

            EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
            EIC->WAKEUP.reg |= (1 << in);

            NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
      */

    } else {
      packetMake(accel.svg_max, accel.avg_svg);
      writePacketToFlash();
      
      Serial.print(accel.avg_svg);
      Serial.println(" is > 3.0");
      Serial.println("Active 30sec alarm ON set");

      LowPower.rtc.disableAlarm();
      LowPower.rtc.setAlarmSeconds(30);
      LowPower.rtc.enableAlarm(LowPower.rtc.MATCH_SS);
      LowPower.rtc.attachInterrupt(onHighFlag);

    }
    svg_max = 0;
    total_svg = 0;
  }//end of accel_flag

  else if (setActiveAlarm_flag) {
    
    setActiveAlarm_flag=false;
    
    accel accel;
    determineAccel(&accel);
    
    packetMake(accel.svg_max, accel.avg_svg);
    writePacketToFlash();
    Serial.println("EEPROM Writing..");
    
    if (accel.avg_svg > DEFINE_ACCEL) {
      Serial.print(accel.avg_svg);
      Serial.println(" is > 3.0");
      Serial.println("and 30 sec alarm is ON constantly.");
    }
    else {
      Serial.print(accel.avg_svg);
      Serial.println(" is < 3.0");
      Serial.println("Last written.");
      Serial.println("and alarm OFF set.");

      LowPower.rtc.detachInterrupt();
      LowPower.rtc.disableAlarm();
      LowPower.rtc.setAlarmMinutes(40);
      LowPower.rtc.enableAlarm(LowPower.rtc.MATCH_MMSS);
      LowPower.rtc.attachInterrupt(onTimeFlag);

      adxl.setActivityXYZ(0, 0, 1);
      adxl.ActivityINT(1);

      LowPower.attachInterruptWakeup(digitalPinToInterrupt(pin), onAccelFlag, FALLING);

      /*
            attachInterrupt(digitalPinToInterrupt(pin), onAccelFlag, FALLING);

            GCLK->CLKCTRL.bit.CLKEN = 0; //disable GCLK module
            while (GCLK->STATUS.bit.SYNCBUSY);

            GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID( GCM_EIC )) ;  //EIC clock switched on GCLK6
            while (GCLK->STATUS.bit.SYNCBUSY);

            GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(6));  //source for GCLK6 is OSCULP32K
            while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

            GCLK->GENCTRL.bit.RUNSTDBY = 1;  //GCLK6 run standby
            while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

            EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
            EIC->WAKEUP.reg |= (1 << in);

            NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
      */
    }
  }//end of setActiveAlarm_flag
  
  Serial.println("mcu is going to sleep.. ");
  Serial.println();
  Serial.end();
  LowPower.sleep();
}

/*
 * Interrupt & Alarm ISR 
 */
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

/*
 * RTC getting epoch 
 */

void resetEpoch() {
  unsigned long epoch;
  int numberOfTries = 0, maxTries = 10;
  do {
    epoch = WiFi.getTime();
    numberOfTries++;
  } while ((epoch == 0) && (numberOfTries < maxTries));

  if (numberOfTries == maxTries) {
    Serial.print("NTP unreachable!");
  } else {
    Serial.print("Epoch received : ");
    Serial.println(epoch);
    LowPower.rtc.setEpoch(epoch + GMT);

    printEpoch();
  }
}

void print2digits(int number) {
  if (number < 10) {
    Serial.print("0"); // print a 0 before if the number is < than 10
  }
  Serial.print(number);
}

void printWiFiStatus() {
  Serial.print("SSID : ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address : ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI) : ");
  Serial.print(rssi);
  Serial.println( "dBm");
}

void printEpoch() {
  Serial.println("-----------------");
  Serial.println("set WiFi RTC");
  print2digits(LowPower.rtc.getDay());
  Serial.print("/");
  print2digits(LowPower.rtc.getMonth());
  Serial.print("/");
  print2digits(LowPower.rtc.getYear());
  Serial.print(" ");

  // ...and time
  print2digits(LowPower.rtc.getHours());
  Serial.print(":");
  print2digits(LowPower.rtc.getMinutes());
  Serial.print(":");
  print2digits(LowPower.rtc.getSeconds());
  Serial.println();
}
