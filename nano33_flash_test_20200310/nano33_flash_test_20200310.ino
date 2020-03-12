#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <FlashStorage.h>
#include <ThingSpeak.h>
#include <RTCZero.h>

#define SECRET_SSID "olleh_WiFi_CF1D"    // replace MySSID with your WiFi network name
#define SECRET_PASS "0000009981"  // replace MyPassword with your WiFi password

#define SECRET_CH_ID 1013182      // replace 0000000 with your channel number
#define SECRET_WRITE_APIKEY "HR76N7LOVHS5TCE3"   // replace XYZ with your channel write API Key

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int keyIndex = 0;
WiFiClient client;
RTCZero rtc;

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;
bool time_flag=false;
bool accel_flag=false;

// Initialize our values
float number1 = 0;
float number2 = 0;
int number3 = 0;

int cnt;

String myStatus = "";

int status = WL_IDLE_STATUS;

unsigned long GMT = 9 * 60 * 60;

char buf[64];

typedef struct timestamp {
  int g_day, g_month, g_year, g_hours, g_minutes, g_seconds;
} timestamp;

typedef struct EEPROMpacket {
  timestamp active_time;
  float svg_max;
  float avg_svg;
  int num;
} EEPROMpacket;

EEPROMpacket EEPROMpkt;
EEPROMpacket writtenPacket[48];
int pkt_num;

Flash(accel_data_store, sizeof(EEPROMpacket));

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
   pkt_num = 0;
}


void initFlash() {
  int i;
  for (i = 0; i < pkt_num; i++) {
    accel_data_store.erase(i);
  }
  pkt_num = 0;
}


int readPacketFromFlash() {

  int i = 0;
  while (i < pkt_num) {
    accel_data_store.read(&(writtenPacket[i]), i);
    Serial.print("read packet number: ");
    Serial.println(writtenPacket[i].num);
    sprintf(buf, "%d/%d/%d/%d:%d:%d,max_svg: %f avg_svg: %f\n", writtenPacket[i].active_time.g_day, writtenPacket[i].active_time.g_month, writtenPacket[i].active_time.g_year, writtenPacket[i].active_time.g_hours, writtenPacket[i].active_time.g_minutes, writtenPacket[i].active_time.g_seconds, writtenPacket[i].svg_max, writtenPacket[i].avg_svg);
    Serial.println(buf);
    i++;
  }
  
  Serial.print(i);
  Serial.println(" packets in flash read Success.");

  return i;
}

void writePacketToFlash() {

  accel_data_store.write(&EEPROMpkt, pkt_num);

  pkt_num++;
  Serial.println("Flash Writing Process Success.");
}

void packetMake(float svg_max, float avg_svg) {
  getHighActiveTime();
  Serial.print("written packet time : ");
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

  Serial.print("written packet data : ");
  Serial.println(EEPROMpkt.svg_max);
  Serial.println(EEPROMpkt.avg_svg);
  Serial.println(EEPROMpkt.num);
  Serial.println("============");
  Serial.println("Packet Make Success");
}

void getHighActiveTime() {
  EEPROMpkt.active_time.g_day = rtc.getDay();
  EEPROMpkt.active_time.g_month = rtc.getMonth();
  EEPROMpkt.active_time.g_year = rtc.getYear();
  EEPROMpkt.active_time.g_hours = rtc.getHours();
  EEPROMpkt.active_time.g_minutes = rtc.getMinutes();
  EEPROMpkt.active_time.g_seconds = rtc.getSeconds();
  Serial.println("Timestamp Write Success");
}

void resetEpoch() {
  unsigned long epoch;
  int numberOfTries = 0, maxTries = 6;
  do {
    epoch = WiFi.getTime();
    numberOfTries++;
  } while ((epoch == 0) && (numberOfTries < maxTries));

  if (numberOfTries == maxTries) {
    Serial.print("NTP unreachable!");
  } else {
    Serial.print("Epoch received : ");
    Serial.println(epoch);
    rtc.setEpoch(epoch + GMT);

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
  print2digits(rtc.getDay());
  Serial.print("/");
  print2digits(rtc.getMonth());
  Serial.print("/");
  print2digits(rtc.getYear());
  Serial.print(" ");

  // ...and time
  print2digits(rtc.getHours());
  Serial.print(":");
  print2digits(rtc.getMinutes());
  Serial.print(":");
  print2digits(rtc.getSeconds());
  Serial.println();
}

void setup() {

  while (!Serial);
  Serial.begin(115200);
  pkt_num = 0;
  cnt=0;
  accel_flag=true;
  // put your setup code here, to run once:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);

    delay(10000);
  }

  printWiFiStatus();

  ThingSpeak.begin(client);

  rtc.begin();

  resetEpoch(); //getting present time epoch

}

void loop() {
  if (time_flag) {
    Serial.println("onTime interrupt generated!");
    resetEpoch();
    time_flag = false;
    // put your main code here, to run repeatedly:
    int num = readPacketFromFlash();

    if (num == pkt_num) {
      Serial.println("all written packets are read from flash Successly.");
    }
    sendThingSpeak(pkt_num);

    initFlash();

    delay(1000);
    accel_flag=false;
  }
  else if (accel_flag) {
   
    packetMake((float)cnt,(float)cnt);
    writePacketToFlash();
    cnt++;
    delay(1000);

    if(!(cnt%5)){
      time_flag=true;
    }
  }
}
