
/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
 * @date June 2018 - 2019
 * @brief HPMA115S0 sensor on ESP32 with bluetooth GATT notify server
 * @license GPL3
 */

#include <Arduino.h>
#include <OTAHandler.h>
#include <Wire.h>
#include <InfluxArduino.hpp>
#include <CanAirIoApi.hpp>
#include <ConfigApp.hpp>
#include <ArduinoJson.h>
#include <numeric>
#include <hpma115S0.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>
#include <GUIUtils.hpp>
#include <vector>
#include <sps30.h>
#include <SoftwareSerial.h>   //!!!!!!!!!!!!!!!!!!!
#include "main.h"
#include "status.h"

/******************************************************************************
* S E T U P  B O A R D   A N D  F I E L D S
* ---------------------
* please select board on platformio.ini file
******************************************************************************/

#ifdef WEMOSOLED // display via i2c for WeMOS OLED board & TTGO18650
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 4, 5, U8X8_PIN_NONE);
#elif HELTEC  // display via i2c for Heltec board
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 15, 4, 16);
#elif TTGO_TQ // display via i2c for TTGO_TQ
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 4, 5);
#else         // display via i2c for D1MINI board
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
#endif

// HPMA115S0 sensor config
#ifdef WEMOSOLED
#define HPMA_RX 13 // config for Wemos board & TTGO18650
#define HPMA_TX 15 // some old TTGO18650 have HPMA_RX 18 & HPMA_TX 17
#elif HELTEC
#define HPMA_RX 13 // config for Heltec board, ESP32Sboard & ESPDUINO-32
#define HPMA_TX 12 // some old ESP32Sboard have HPMA_RX 27 & HPMA_TX 25
#elif TTGO_TQ
#define HPMA_RX 13 // config for TTGO_TQ board
#define HPMA_TX 18
#else
#define HPMA_RX 17 // config for D1MIN1 board
#define HPMA_TX 16

////
#define HPMA_RX1 18 // config for D1MIN1 board
#define HPMA_RX2 19 // config for D1MIN1 board
#define HPMA_RX3 23 // config for D1MIN1 board
#define HPMA_RX4 5 // config for D1MIN1 board

////
#endif

/*
 * Read particle matter using a Honeywell HPMA115S0-XXX and a ESP8266 (WeMos Mini)
 */

#define DEBUG false
#define pin_rx1 18 //OK Honeywell
#define pin_tx1 22
#define pin_rx2 19 //Paila PMSA003, checksum mismatch
#define pin_tx2 21
#define pin_rx3 23 //OK PMS7003
#define pin_tx3 27
#define pin_rx4 5  //Panasonic
#define pin_tx4 25

SoftwareSerial Device1(pin_rx4, pin_tx4);
//SoftwareSerial Device2(pin_rx3, pin_tx3);

int pm25 = 0;   // PM2.5
int pm10 = 0;   // PM10

unsigned long lastReading = 0;

int readResponse1(int l = 32) {
  int i = 0;
  int buf[l];
  
  unsigned long start = millis();

  while(Device1.available() > 0 && i < l) {

    buf[i] = Device1.read();                 // read bytes from device

    if(DEBUG) {
      Serial.print("i: "); Serial.print(i);
      Serial.print(" buf[i]: "); Serial.println(buf[i], HEX);
    }

    // check for HEAD or skip a byte
    if(i == 0 && !(buf[0] == 0x40 || buf[0] == 0x42 || buf[0] == 0xA5 || buf[0] == 0x96)) {
      if(DEBUG) { Serial.println("Skipping Byte"); }
      continue;
    } else {
      i++;
    }

    if(buf[0] == 0x42 && buf[1] == 0x4d) {  // Autosend
      if(DEBUG) { Serial.println("Autosend"); }
      l=32;
    }

    if(buf[0] == 0x40 && buf[2] == 0x4) {   // Reading
      if(DEBUG) { Serial.println("Reading"); }
      l=8;
    }

    if(buf[0] == 0xA5 && buf[1] == 0xA5) {  // Pos. ACK
      if(DEBUG) { Serial.println("ACK"); }
      return true;
    }
    
    if(buf[0] == 0x96 && buf[1] == 0x96) {  // Neg. ACK
      if(DEBUG) { Serial.println("NACK"); }
      return false;
    }

    if (millis() - start > 1000) {          // trigger Timeout after 1 sec
      Serial.println("Timeout");
      return false;
    }

  }

  // check checksum in Reading
  if(buf[2] == 0x04) {
    // HEAD+LEN+CMD
    int cs = buf[0] + buf[1] + buf[2];
    int c;
    
    // DATA
    for(c = 3; c < (2 + buf[1]); c++) {
      // Serial.println(buf[c]);
      cs += buf[c];
    }
    // CS = MOD((65536-(HEAD+LEN+CMD+DATA)), 256)
    cs = (65536 - cs) % 256;
    
    // validate checksum
    if(cs == buf[c]) {
      // calculate PM values
      pm25 = buf[3] * 256 + buf[4];
      pm10 = buf[5] * 256 + buf[6];
      return true;
    } else {
      Serial.println("Checksum mismatch");
    }
  } else if(buf[3] == 0x1c) {  // Autoreading
    int cs = 0;
    int c;
    // DATA
    for(c = 0; c <= buf[3]; c++) {
      // Serial.println(buf[c]);
      cs += buf[c];
    }
    int checksum = buf[30] * 256 + buf[31];
    if(DEBUG) {
      Serial.print("Checksum: "); Serial.print(checksum, HEX);
      Serial.print(" CS: "); Serial.println(cs, HEX);
    }

    if(cs == checksum) {
      // calculate PM values
      pm25 = buf[6] * 256 + buf[7];
      pm10 = buf[8] * 256 + buf[9];
      return true;
    } else {
      Serial.println("Checksum mismatch");
    }
  } else {
    // unkown
  }
  
  return false;
}
/*
int readResponse2(int l = 32) {
  int i = 0;
  int buf[l];
  
  unsigned long start = millis();

  while(Device2.available() > 0 && i < l) {

    buf[i] = Device2.read();                 // read bytes from device

    if(DEBUG) {
      Serial.print("i: "); Serial.print(i);
      Serial.print(" buf[i]: "); Serial.println(buf[i], HEX);
    }

    // check for HEAD or skip a byte
    if(i == 0 && !(buf[0] == 0x40 || buf[0] == 0x42 || buf[0] == 0xA5 || buf[0] == 0x96)) {
      if(DEBUG) { Serial.println("Skipping Byte"); }
      continue;
    } else {
      i++;
    }

    if(buf[0] == 0x42 && buf[1] == 0x4d) {  // Autosend
      if(DEBUG) { Serial.println("Autosend"); }
      l=32;
    }

    if(buf[0] == 0x40 && buf[2] == 0x4) {   // Reading
      if(DEBUG) { Serial.println("Reading"); }
      l=8;
    }

    if(buf[0] == 0xA5 && buf[1] == 0xA5) {  // Pos. ACK
      if(DEBUG) { Serial.println("ACK"); }
      return true;
    }
    
    if(buf[0] == 0x96 && buf[1] == 0x96) {  // Neg. ACK
      if(DEBUG) { Serial.println("NACK"); }
      return false;
    }

    if (millis() - start > 1000) {          // trigger Timeout after 1 sec
      Serial.println("Timeout");
      return false;
    }

  }

  // check checksum in Reading
  if(buf[2] == 0x04) {
    // HEAD+LEN+CMD
    int cs = buf[0] + buf[1] + buf[2];
    int c;
    
    // DATA
    for(c = 3; c < (2 + buf[1]); c++) {
      // Serial.println(buf[c]);
      cs += buf[c];
    }
    // CS = MOD((65536-(HEAD+LEN+CMD+DATA)), 256)
    cs = (65536 - cs) % 256;
    
    // validate checksum
    if(cs == buf[c]) {
      // calculate PM values
      pm25 = buf[3] * 256 + buf[4];
      pm10 = buf[5] * 256 + buf[6];
      return true;
    } else {
      Serial.println("Checksum mismatch");
    }
  } else if(buf[3] == 0x1c) {  // Autoreading
    int cs = 0;
    int c;
    // DATA
    for(c = 0; c <= buf[3]; c++) {
      // Serial.println(buf[c]);
      cs += buf[c];
    }
    int checksum = buf[30] * 256 + buf[31];
    if(DEBUG) {
      Serial.print("Checksum: "); Serial.print(checksum, HEX);
      Serial.print(" CS: "); Serial.println(cs, HEX);
    }

    if(cs == checksum) {
      // calculate PM values
      pm25 = buf[6] * 256 + buf[7];
      pm10 = buf[8] * 256 + buf[9];
      return true;
    } else {
      Serial.println("Checksum mismatch");
    }
  } else {
    // unkown
  }
  
  return false;
}
*/
void setup() {
  // init Serial for ESP8266
  Serial.begin(115200); Serial.println();
  // init Serial for Device
  Device1.begin(9600); Device1.println();
//  Device2.begin(9600); Device2.println();
}

void loop() {

  if(millis() - lastReading >= 1000 || lastReading == 0) {
    lastReading = millis();

    // handle AutoSend
    
        if(readResponse1()) {
          Serial.print("PM 2.5_1: "); Serial.print(pm25);
          Serial.print(" / PM 10_1: "); Serial.println(pm10);
        }
    }

/*  
  if(millis() - lastReading >= 1000 || lastReading == 0) {
    lastReading = millis();

    // handle AutoSend
    
        if(readResponse2()) {
          Serial.print("PM 2.5_2: "); Serial.print(pm25);
          Serial.print(" / PM 10_2: "); Serial.println(pm10);
        }
    }

    */
}