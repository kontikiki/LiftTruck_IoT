#include <ArduinoBLE.h>
#include <WiFiNINA.h>
#include <FlashStorage.h>

typedef struct {
  boolean valid;
  char ssid[64] = {0,};
  char password[64] = {0,};
} ID;

ID myID;
FlashStorage(my_ssid_store, ID);
int len = 0;

char *ssid = myID.ssid;
char *pass = myID.password;

int keyIndex = 0;
WiFiClient client;
int status = WL_IDLE_STATUS;

void printWiFiStatus();

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool getID_debug = false;
bool printID_debug=false;
String txValue;
String rxValue;

// BLE Battery Service
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
//BLEService uartService("1801");

// BLE Uart Characteristic
//BLECharacteristic uartCharacteristic("2a01",BLENotify,txValue);

BLEStringCharacteristic uartCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",  // String characteristic UUID
    BLENotify, 512); // remote clients will be able to get notifications if this characteristic changes
BLEStringCharacteristic getCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 512);


void setup() {
  Serial.begin(115200);    // initialize serial communication
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("UartService");
  BLE.setAdvertisedService(uartService); // add the service UUID
  uartService.addCharacteristic(uartCharacteristic); // add the battery level characteristic
  uartService.addCharacteristic(getCharacteristic);
  BLE.addService(uartService); // Add the battery service

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  getCharacteristic.setEventHandler(BLEWritten, getCharacteristicWritten);
  // set an initial value for the characteristic
  uartCharacteristic.setValue("hello World");

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  Serial.println("Connect ble");
}

void loop() {

  BLE.poll();
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central) {
    // while the central is still connected to peripheral:
    while (central.connected()) {
      myID = my_ssid_store.read();

      if (myID.valid == false) {
        if (getID_debug == false) {
          Serial.println("Send your SSID& PASSWORD( ex>SSID,PASSWORD )");
          getID_debug = true;
        }
        if (getCharacteristic.written()) {
          if (getCharacteristic.value()) {
            int index = rxValue.indexOf(",");
            String ssid = rxValue.substring(0, index);
            String passwd = rxValue.substring(index + 1, rxValue.length());
            ssid.toCharArray(myID.ssid, ssid.length() + 1);
            passwd.toCharArray(myID.password, passwd.length() + 1);
          }
        }

        if (myID.ssid[0] != NULL) {
          myID.valid = true;
          my_ssid_store.write(myID);
          Serial.print("your SSID: ");
          Serial.println(myID.ssid);
          Serial.print("your PASSWORD: ");
          Serial.println(myID.password);
          Serial.println("=========");
        }
      }
      else {
        if(printID_debug==false){
        Serial.println("your store data is below..");
        Serial.println("=========");
        Serial.print("your SSID: ");
        Serial.println(myID.ssid);
        Serial.print("your PASSWORD: ");
        Serial.println(myID.password);
        Serial.println("=========");
        printID_debug=true;
        Serial.println("if your central BLE is connected, do DISCONNECT."); 
        }
      }
    }
    oldDeviceConnected = true;
    BLE.end();
    Serial.println("disconnected ble. NEXT, connecting WiFi");
  }

  if (oldDeviceConnected) {
    while (status != WL_CONNECTED) {
      Serial.println("=========");
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      status = WiFi.begin(ssid, pass);
      delay(5000);
    }
    printWiFiStatus();
    oldDeviceConnected = false;
  }
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  digitalWrite(LED_BUILTIN, HIGH);
  deviceConnected = true;
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  digitalWrite(LED_BUILTIN, LOW);
  deviceConnected = false;
}

void getCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  //Serial.print("Characteristic event, written: ");
  rxValue = getCharacteristic.value();
  if (rxValue.length() > 0) {
    for (int i = 0; i < rxValue.length(); i++)
      Serial.print(rxValue[i]);
    Serial.println();
  }
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
