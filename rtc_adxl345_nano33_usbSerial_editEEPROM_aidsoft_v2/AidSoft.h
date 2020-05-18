#ifndef AidSoft_h
#define AidSoft_h

#define AS_VER "0.1"

#include "Arduino.h"
#include <Client.h>
#include "ArduinoJson.h"

#define AIDSOFT_URL "withinnet.devaidsoft.net"
#define AIDSOFT_PORT_NUMBER 80

#ifdef ARDUINO_ARCH_AVR
    #ifdef ARDUINO_AVR_YUN
        #define USER_AGENT "lib-arduino/" AS_VER "(arduino yun)"
    #else
        #define USER_AGENT "lib-arduino/" AS_VER " (arduino uno or mega)"
    #endif
#elif defined(ARDUINO_ARCH_ESP8266)
    #define USER_AGENT "lib-arduino/" AS_VER " (ESP8266)"
#elif defined(ARDUINO_SAMD_MKR1000)
	#define USER_AGENT "lib-arduino/" AS_VER " (arduino mkr1000)"
#elif defined(ARDUINO_SAM_DUE)
	#define USER_AGENT "lib-arduino/" AS_VER " (arduino due)"
#elif defined(ARDUINO_ARCH_SAMD) 
	#define USER_AGENT "lib-arduino/" AS_VER " (arduino samd)"
#elif defined(ARDUINO_ARCH_SAM)
	#define USER_AGENT "lib-arduino/" AS_VER " (arduino sam)"
#elif defined(ARDUINO_ARCH_SAMD_BETA)
	#define USER_AGENT "lib-arduino/" AS_VER " (arduino samd_beta)"
#elif defined(ARDUINO_ARCH_ESP32)
	#define USER_AGENT "lib-arduino/" AS_VER " (ESP32)"
#elif defined(ARDUINO_ARCH_SAMD_BETA)
	#define USER_AGENT "lib-arduino/" AS_VER " (arduino vidor)"
#else
	#define USER_AGENT "lib-arduino/" AS_VER " (unknown)"
#endif

#define TIMEOUT_MS_SERVERRESPONSE 5000  // Wait up to five seconds for server to respond

#define OK_SUCCESS              200     // OK / Success
#define ERR_BADAPIKEY           400     // Incorrect API key (or invalid AidSoft server address)
#define ERR_BADURL              404     // Incorrect API key (or invalid AidSoft server address)
#define ERR_OUT_OF_RANGE        -101    // Value is out of range or string is too long (> 255 bytes)
#define ERR_INVALID_FIELD_NUM   -201    // Invalid field number specified
#define ERR_SETFIELD_NOT_CALLED -210    // setField() was not called before writeFields()
#define ERR_CONNECT_FAILED      -301    // Failed to connect to ThingSpeak
#define ERR_UNEXPECTED_FAIL     -302    // Unexpected failure during write to TAidSoft
#define ERR_BAD_RESPONSE        -303    // Unable to parse response
#define ERR_TIMEOUT             -304    // Timeout waiting for server to respond
#define ERR_NOT_INSERTED        -401    // Point was not inserted (most probable cause is the rate limit of once every 15 seconds)

extern WiFiClient client;

bool connectServer();
void emptyStream();
int finishWrite();
int getHTTPResponse(String & response);
void HTTPheader(int contentLength,const char *apikey);

bool connectServer()
{
  bool connectSuccess = false;

  connectSuccess = client.connect(const_cast<char *>(AIDSOFT_URL), AIDSOFT_PORT_NUMBER);
  if (connectSuccess)
  {
    Serial.println("Success.");
  }
  else
  {
    Serial.println("Failed.");
  }
  return connectSuccess;
}

int getHTTPResponse(String & response)
{

  // make sure all of the HTTP request is pushed out of the buffer before looking for a response
  client.flush();

  long timeoutTime = millis() + TIMEOUT_MS_SERVERRESPONSE;

  while (client.available() < 17) {
    delay(2);
    if (millis() > timeoutTime) {
      return ERR_TIMEOUT;
    }
  }

  if (!client.find(const_cast<char *>("HTTP/1.1")))
  {
    return ERR_BAD_RESPONSE; // Couldn't parse response (didn't find HTTP/1.1)
  }
  int status = client.parseInt();

  if (status != OK_SUCCESS)
  {
    return status;
  }

  // Find Content-Length
  if (!client.find(const_cast<char *>("Content-Length:"))) {
    return ERR_BAD_RESPONSE; // Couldn't parse response (didn't find HTTP/1.1)
  }
  int contentLength = client.parseInt();

  if (!client.find(const_cast<char *>("\r\n\r\n")))
  {
    return ERR_BAD_RESPONSE;
  }

  timeoutTime = millis() + TIMEOUT_MS_SERVERRESPONSE;

  while (client.available() < contentLength) {
    delay(2);
    if (millis() > timeoutTime) {
      return ERR_TIMEOUT;
    }
  }

  String tempString = String("");
  char y = 0;
  for (int i = 0; i < contentLength; i++) {
    y = client.read();
    tempString.concat(y);
  }
  response = tempString;
  
  return status;
}

void emptyStream() {
  while (client.available() > 0) {
    client.read();
  }
}

int finishWrite() {
  String entryIDText = String();
  int status = getHTTPResponse(entryIDText);

  emptyStream();

  if (status != OK_SUCCESS)
  {
    client.stop();
    return status;
  }
  long entryID = entryIDText.toInt();
  
  client.stop();

  if (entryID == 0)
  {
    Serial1.println("ERR_NOT_INSERTED.");
    //did not accept the write
    status = ERR_NOT_INSERTED;
  }

  return status;
}

void HTTPheader(int contentLength,const char *apikey){
  client.print("POST /api/DPS HTTP/1.1\r\n");
  client.print("Host: withinnet.devaidsoft.net\r\n");
  if (NULL != apikey)
  {
    client.print("api_key: ");
    client.print(apikey);
    client.print("\r\n");
  }

  client.print("Content-Type: application/json\r\n");
  client.print("Content-Length: ");
  client.print(contentLength);
  client.print("\r\n");
  client.print("User-Agent: ");
  client.print(USER_AGENT);
  client.print("\r\n\r\n");
}

#endif
