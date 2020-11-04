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
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>
#include <Adafruit_SHT31.h>
#include <GUIUtils.hpp>
#include <vector>
#include <sps30.h>
#include <SoftwareSerial.h>
#include "main.h"
#include "status.h"

/******************************************************************************
* S E T U P  B O A R D   A N D  F I E L D S
* ---------------------
* please select board on platformio.ini file
******************************************************************************/

#ifdef WEMOSOLED // display via i2c for WeMOS OLED board & TTGO18650
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 4, 5, U8X8_PIN_NONE);
#elif HELTEC // display via i2c for Heltec board
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 15, 4, 16);
#elif TTGO_TQ // display via i2c for TTGO_TQ
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 4, 5);
#else       // display via i2c for D1MINI board
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE,U8X8_PIN_NONE,U8X8_PIN_NONE);
#endif

// HPMA115S0 sensor config
#ifdef WEMOSOLED
#define HPMA_RX 13  // config for Wemos board & TTGO18650
#define HPMA_TX 15  // some old TTGO18650 have HPMA_RX 18 & HPMA_TX 17
#elif HELTEC
#define HPMA_RX 13  // config for Heltec board, ESP32Sboard & ESPDUINO-32
#define HPMA_TX 12  // some old ESP32Sboard have HPMA_RX 27 & HPMA_TX 25
#elif TTGO_TQ
#define HPMA_RX 13  // config for TTGO_TQ board
#define HPMA_TX 18
#else
#define HPMA_RX 17  // config for D1MIN1 board
#define HPMA_TX 16
#endif

#define pin_rx1 18 // Honeywell
#define pin_tx1 4  // NA
#define pin_rx2 19 // PMSA003, checksum mismatch
#define pin_tx2 12 // NA
#define pin_rx3 23 // PMS7003
#define pin_tx3 32 // NA
#define pin_rx4 5 // Panasonic !!!!!!!!!!!
#define pin_tx4 25

SoftwareSerial Device1(pin_rx1, pin_tx1);
SoftwareSerial Device2(pin_rx2, pin_tx2);
SoftwareSerial Device3(pin_rx3, pin_tx3);

SPS30 sps30;

/******************************************************************************
*   S E N S O R  M E T H O D S
******************************************************************************/

/**
 * [DEPRECATED] sensorConfig:
 * The next method is only if Honeywell sensor was config without autosend
 */
#ifdef HONEYWELL
void sensorConfig(){
  Serial.println("-->[HPMA] configuration hpma115S0 sensor..");
  hpmaSerial.begin(9600,SERIAL_8N1,HPMA_RX,HPMA_TX);
  hpma115S0.Init();
  delay(100);
  hpma115S0.EnableAutoSend();
  delay(100);
  hpma115S0.StartParticleMeasurement();
  delay(100);
  Serial.println("-->[HPMA] sensor configured.");
}
#endif

void ErrtoMess(char *mess, uint8_t r){
  char buf[80];
  Serial.print(mess);
  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

void Errorloop(char *mess, uint8_t r){
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  setErrorCode(ecode_sensor_timeout);
  delay(500); // waiting for sensor..
}

void SensirionInit(){
  // Begin communication channel;
  Serial.println(F("-->[SPS30] starting SPS30 sensor.."));
  if (sps30.begin(SP30_COMMS) == false){
    Errorloop((char *)"-->[E][SPS30] could not initialize communication channel.", 0);
  }
  // check for SPS30 connection
  if (sps30.probe() == false){
    Errorloop((char *)"-->[E][SPS30] could not probe / connect with SPS30.", 0);
  }
  else
    Serial.println(F("-->[SPS30] Detected SPS30."));
  // reset SPS30 connection
  if (sps30.reset() == false){
    Errorloop((char *)"-->[E][SPS30] could not reset.", 0);
  }
  // start measurement
  if (sps30.start() == true)
    Serial.println(F("-->[SPS30] Measurement OK"));
  else
    Errorloop((char *)"-->[E][SPS30] Could NOT start measurement", 0);
  if (SP30_COMMS == I2C_COMMS){
    if (sps30.I2C_expect() == 4)
      Serial.println(F("-->[E][SPS30] Due to I2C buffersize only PM values  \n"));
  }
}

void sensorInit(){
  // HONEYWELL
  Serial.println("-->[HPMA]  starting hpma115S0 sensor..");
  delay(100);
  Device1.begin(9600);
  //Device1.println();
  gui.AddMessage("1.");      /////////////////////////
  delay(200);
  // PANASONIC
  Serial.println("-->[SN]    starting SN-GCJA5 sensor..");
  delay(100);
  hpmaSerial.begin(9600, SERIAL_8N1, pin_rx4, pin_tx4);
  gui.AddMessage("2.");      /////////////////////////
  delay(200);
  //PMSA003
  Serial.println("-->[PMS]   starting PMSA003 sensor..");
  delay(100);
  Device2.begin(9600);
  //Device2.println();
  gui.AddMessage("3.");      /////////////////////////
  delay(200);
  //PMS7003
  Serial.println("-->[PMS]   starting PMS7003 sensor..");
  delay(100);
  Device3.begin(9600);
  //Device3.println();
  gui.AddMessage("4.");      /////////////////////////
  delay(200);
  //#else //SENSIRION
  SensirionInit();
  gui.welcomeAddMessage("5.");      /////////////////////////
  delay(1000);
}

void wrongDataState(){
  setErrorCode(ecode_sensor_read_fail);
  gui.displaySensorAverage(apm25);
  gui.displaySensorData(0, 0, chargeLevel, 0.0, 0.0, 0);
  Serial.println("-->[E][Sensor] !wrong data!");
  //Serial.print(v25.size()); // Show numsensor like v25
  v25.push_back(pm25);      //!!!!!!!!!!Definir num
  v10.push_back(pm10);
  //Serial.print(v25.size()); // Show numsensor like v25
  switch (numsensor - 1){

 case 0: //PMS7003
    Device3.end();
    Serial.println("-->[E][PMS7003] !error reading data!");
    delay(100);
    Device3.begin(9600);
    //Device3.println();
    delay(100);
    break;

  case 1: // PMSA003
    Device2.end();
    Serial.println("-->[E][PMSA003] !error reading data!");
    delay(100);
    Device2.begin(9600);
    //Device2.println();
    delay(100);
    break;

  case 2: // PANASONIC
    hpmaSerial.end();
    Serial.println("-->[E][SNGCJA5] !error reading data!");
    delay(100);
    hpmaSerial.begin(9600, SERIAL_8N1, pin_rx4, pin_tx4);
    delay(100);
    break;

  case 3: // HONEYWELL
    Device1.end();
    Serial.println("-->[E][HPMA] !error reading data!");
    delay(100);
    Device1.begin(9600);
    //Device1.println();
    delay(100);
    break;

  case -1: // SPS30
    Serial.println("-->[E][SPS30] !error reading data!");
    delay(100);
    SensirionInit();
    delay(100);
    break;
  }
  statusOff(bit_sensor);
  delay(500);
}

/***
 * Average methods
 **/

void saveDataForAverage(unsigned int pm25, unsigned int pm10, float pm25f){
  v25.push_back(pm25);
  v10.push_back(pm10);

  switch (numsensor - 1){

  case 0: // PMS7003
    v250.push_back(pm25);
    break;

  case 1: // PMSA003
    v251.push_back(pm25);
    break;

  case 2: // PANASONIC
    v252.push_back(pm25);
    break;

  case 3: // HONEYWELL
    v253.push_back(pm25);
    break;

  case -1: // SPS30
    v254.push_back(pm25);
    v254f.push_back(pm25f);
    break;
  }
}

/*
unsigned int getPM25Average(){
  unsigned int pm25_average = round(accumulate(v25.begin(), v25.end(), 0.0) / v25.size());
  v25.clear();
  return pm25_average;
}

unsigned int getPM10Average(){
  unsigned int pm10_average = round(accumulate(v10.begin(), v10.end(), 0.0) / v10.size());
  v10.clear();
  return pm10_average;
}
*/

void averageLoop(){
  // if (v25.size() >= cfg.stime)
  ////////////// 5 sensors
  if (v25.size() >= cfg.stime * 5){
    //apm25 = getPM25Average();     // global var for display
    //apm10 = getPM10Average();
    v25.clear();

    apm250 = round(accumulate(v250.begin(), v250.end(), 0.0) / v250.size());
    v250.clear();
    apm251 = round(accumulate(v251.begin(), v251.end(), 0.0) / v251.size());
    v251.clear();
    apm252 = round(accumulate(v252.begin(), v252.end(), 0.0) / v252.size());
    v252.clear();
    apm253 = round(accumulate(v253.begin(), v253.end(), 0.0) / v253.size());
    v253.clear();
    // Int and Float for SPS30
    apm254 = round(accumulate(v254.begin(), v254.end(), 0.0) / v254.size());
    v254.clear();
    apm254f = accumulate(v254f.begin(), v254f.end(), 0.0) / v254f.size();
    v254f.clear();

    apm25 = apm253;     // PM2.5 HONEYWELL
    apm10 = apm250;     // PM2.5 PMS7003 
    cfg.lat = apm251;   // PM2.5 PMSA003
    cfg.lon = apm252;   // PM2.5 PANASONIC
    cfg.alt = apm254;   // PM2.5 SPS30
    cfg.spd = apm254f;  // PM2.5float SPS30
  }
}

char getLoaderChar(){ 
  char loader[] = {'/','|','\\','-'};
  return loader[random(0,4)];
}

void showValues(int pm25, int pm10){
  gui.displaySensorAverage(apm25); // it was calculated on bleLoop()
  gui.displaySensorData(pm25, pm10, chargeLevel, humi, temp, rssi);
  gui.displayLiveIcon();
  saveDataForAverage(pm25, pm10, pm25f);
}

/***
 * PM2.5 and PM10 read and visualization
 **/

void sensorLoop(){
  int try_sensor_read = 0;
  String txtMsg = "";
  switch (numsensor){

    case 0: // PMS7003
    numsensor = numsensor + 1;
    while (txtMsg.length() < 32 && try_sensor_read++ < SENSOR_RETRY){
      while (Device3.available() > 0){
        char inChar = Device3.read();
        txtMsg += inChar;
        Serial.print("-->[PMS7003] read " + String(getLoaderChar()) + "\r");
      }
      Serial.print("-->[PMS7003] read " + String(getLoaderChar()) + "\r");
    }
    if (try_sensor_read > SENSOR_RETRY){
      setErrorCode(ecode_sensor_timeout);
      Serial.println("-->[PMS7003] read > fail!");
      Serial.println("-->[E][PMS7003] disconnected ?");
      delay(500); // waiting for sensor..
    }

    if (txtMsg[0] == 66){
      if (txtMsg[1] == 77){
        Serial.print("-->[PMS7003] read > done!");
        statusOn(bit_sensor);
        pm25 = txtMsg[6] * 256 + byte(txtMsg[7]);
        pm10 = txtMsg[8] * 256 + byte(txtMsg[9]);
        if (pm25 < 1000 && pm10 < 1000){
          showValues(pm25, pm10);
        }
        else
          wrongDataState();
      }
      else
        wrongDataState();
    }
    else
      wrongDataState();
    break;

    case 1: //PMSA003
    numsensor = numsensor + 1;
    while (txtMsg.length() < 32 && try_sensor_read++ < SENSOR_RETRY){
      while (Device2.available() > 0){
        char inChar = Device2.read();
        txtMsg += inChar;
        Serial.print("-->[PMSA003] read " + String(getLoaderChar()) + "\r");
      }
      Serial.print("-->[PMSA003] read " + String(getLoaderChar()) + "\r");
    }
    if (try_sensor_read > SENSOR_RETRY){
      setErrorCode(ecode_sensor_timeout);
      Serial.println("-->[PMSA003] read > fail!");
      Serial.println("-->[E][PMSA003] disconnected ?");
      delay(500); // waiting for sensor..
    }

    if (txtMsg[0] == 66){
      if (txtMsg[1] == 77){
        Serial.print("-->[PMSA003] read > done!");
        statusOn(bit_sensor);
        pm25 = txtMsg[6] * 256 + byte(txtMsg[7]);
        pm10 = txtMsg[8] * 256 + byte(txtMsg[9]);
        if (pm25 < 1000 && pm10 < 1000){
          showValues(pm25, pm10);
        }
        else
          wrongDataState();
      }
      else
        wrongDataState();
    }
    else
      wrongDataState();
    break;

    case 2: // PANASONIC
    numsensor = numsensor + 1;
    while (txtMsg.length() < 32 && try_sensor_read++ < SENSOR_RETRY){
      while (hpmaSerial.available() > 0){
        char inChar = hpmaSerial.read();
        txtMsg += inChar;
        Serial.print("-->[SNGCJA5] read " + String(getLoaderChar()) + "\r");
      }
      Serial.print("-->[SNGCJA5] read " + String(getLoaderChar()) + "\r");
    }
    if (try_sensor_read > SENSOR_RETRY){
      setErrorCode(ecode_sensor_timeout);
      Serial.println("-->[SNGCJA5] read > fail!");
      Serial.println("-->[E][SNGCJA5] disconnected ?");
      delay(500); // waiting for sensor..
    }

    if (txtMsg[0] == 02){
      Serial.print("-->[SNGCJA5] read > done!");
      statusOn(bit_sensor);
      pm25 = txtMsg[6] * 256 + byte(txtMsg[5]);
      pm10 = txtMsg[10] * 256 + byte(txtMsg[9]);
      if (pm25 < 2000 && pm10 < 2000){
        showValues(pm25, pm10);
      }
      else
        wrongDataState();
    }
    else
      wrongDataState();

    break;


  case 3: // HONEYWELL
    numsensor = numsensor + 1;
    while (txtMsg.length() < 32 && try_sensor_read++ < SENSOR_RETRY){
      while (Device1.available() > 0){
        char inChar = Device1.read();
        txtMsg += inChar;
        Serial.print("-->[HPMA] read " + String(getLoaderChar()) + "\r");
      }
      Serial.print("-->[HPMA] read " + String(getLoaderChar()) + "\r");
    }
    if (try_sensor_read > SENSOR_RETRY){
      setErrorCode(ecode_sensor_timeout);
      Serial.println("-->[HPMA] read > fail!");
      Serial.println("-->[E][HPMA] disconnected ?");
      delay(500); // waiting for sensor..
    }

    if (txtMsg[0] == 66){
      if (txtMsg[1] == 77){
        Serial.print("-->[HPMA115] read > done!");
        statusOn(bit_sensor);
        pm25 = txtMsg[6] * 256 + byte(txtMsg[7]);
        pm10 = txtMsg[8] * 256 + byte(txtMsg[9]);
        if (pm25 < 1000 && pm10 < 1000){
          showValues(pm25, pm10);
        }
        else
          wrongDataState();
      }
      else
        wrongDataState();
    }
    else
      wrongDataState();
    break;
 
  case 4:      // SPS30
    delay(35); //Delay for sincronization
    numsensor = 0;
    // loop to get data
    do {
      ret = sps30.GetValues(&val);

      //  Serial.println(sizeof(struct sps_values));
      // data might not have been ready
      if (ret == ERR_DATALENGTH){
        if (error_cnt++ > 3){
          ErrtoMess((char *)"-->[E][SPS30] Error during reading values: ", ret);
          Serial.println("error cnt");
          wrongDataState();
          return;
        }
        delay(1000);
      }
      // if other error
      else if (ret != ERR_OK){
        ErrtoMess((char *)"-->[E][SPS30] Error during reading values: ", ret);
        Serial.println("error else if");
        wrongDataState();
        return;
      }
    } while (ret != ERR_OK);

    Serial.print("-->[SPS30]   read > done!");
    statusOn(bit_sensor);

    pm25 = round(val.MassPM2);
    pm10 = round(val.MassPM10);
    pm25f = val.MassPM2;

    if (pm25 < 1000 && pm10 < 1000){
         //showValues(pm25, pm10);

       gui.displaySensorAverage(apm25); // it was calculated on bleLoop()
       gui.displaySensorData(pm25, pm10, chargeLevel, humi, temp, rssi);
       gui.displayLiveIcon();
       saveDataForAverage(pm25, pm10, pm25f);
    }
    else {
      wrongDataState();
    }
    break;
  }
}

void statusLoop(){
  if (v25.size() == 0) {
    Serial.print("-->[STATUS] ");
    Serial.println(status.to_string().c_str());
    updateStatusError();
    wifiCheck();
  }
  gui.updateError(getErrorCode());
  gui.displayStatus(wifiOn,true,deviceConnected,dataSendToggle);
  if(triggerSaveIcon++<3) gui.displayPrefSaveIcon(true);
  else gui.displayPrefSaveIcon(false);
  if(dataSendToggle) dataSendToggle=false;
}

String getNotificationData(){
  StaticJsonDocument<40> doc;
  doc["P25"] = apm25;  // notification capacity is reduced, only main value
  String json;
  serializeJson(doc,json);
  return json;
}

String getSensorData(){
  StaticJsonDocument<150> doc;
  doc["P25"] = apm25;
  doc["P10"] = apm10;
  doc["lat"] = cfg.lat;
  doc["lon"] = cfg.lon;
  doc["alt"] = cfg.alt;
  doc["spd"] = cfg.spd;
  doc["sta"] = status.to_string().c_str();
  String json;
  serializeJson(doc,json);
  return json;
}

void getHumidityRead() {
  humi = am2320.readHumidity();
  temp = am2320.readTemperature();
  if (isnan(humi)){
    humi = 0.0;
    am2320.begin();
  }
  if (isnan(temp))
    temp = 0.0;
  Serial.println("-->[AM2320] Humidity: "+String(humi)+" % Temp: "+String(temp)+" °C");

  humiS = sht31.readHumidity();
  tempS = sht31.readTemperature();
  if (isnan(humiS)) {
    humiS = 0.0;
    sht31.begin(0x44);
  }
  if (isnan(tempS)) 
    tempS = 0.0;
  Serial.println("-->[SHT31]  Humidity: "+String(humiS)+" % Temp: "+String(tempS)+" °C");
}

void humidityLoop(){
  if (v25.size() == 0){
    getHumidityRead();
  }
}

/******************************************************************************
*   B A T T E R Y   C H A R G E   S T A T U S   M E T H O D S
******************************************************************************/
// IP5306_2 = pin 27 ESP32, pin 2 IP5306
// IP5306_3 = pin 26 ESP32, pin 3 IP5306

void batteryloop() {
#ifdef TTGO_TQ
  Rdelay = 0;
  while (digitalRead(IP5306_2) == HIGH)
  {
    delayMicroseconds(100); // Sincronization in 1
  }
  delayMicroseconds(50); // Probably double shoot in 0
  while (digitalRead(IP5306_2) == HIGH)
  {
    delayMicroseconds(100); // Sincronization in 1
  }
  while (digitalRead(IP5306_2) == LOW && Rdelay < 56)
  {
    delayMicroseconds(100); // Sincronization in 0
    Rdelay = Rdelay + 1;
  }
  if (Rdelay > 52)
  {
    chargeLevel = 0; // 0%
    return;
  }
  delayMicroseconds(1600);
  if (digitalRead(IP5306_2) == HIGH)
  {
    delayMicroseconds(100);
    if (digitalRead(IP5306_2) == HIGH)
    {
      chargeLevel = 100; // 100%
      return;
    }
  }
  if (digitalRead(IP5306_3) == LOW)
  {
    delayMicroseconds(100);
    if (digitalRead(IP5306_3) == LOW)
    {
      chargeLevel = 25; // 25%
      return;
    }
  }
  delayMicroseconds(1100);
  if (digitalRead(IP5306_3) == HIGH)
  {
    delayMicroseconds(100);
    if (digitalRead(IP5306_3) == HIGH)
    {
      chargeLevel = 75; // 75%
      return;
    }
  }
  if (digitalRead(IP5306_3) == LOW)
  {
    chargeLevel = 50; // 50%
    return;
  }
#endif
}

/******************************************************************************
*   C A N A I R I O  P U B L I S H   M E T H O D S
******************************************************************************/

bool apiIsConfigured(){
  return cfg.apiusr.length() > 0 && cfg.apipss.length() > 0 && cfg.dname.length() > 0;
}

void apiInit(){
  if (wifiOn && apiIsConfigured()) {
    Serial.println("-->[API] Connecting..");
    // stationId and deviceId, optional endpoint, host and port
    if(cfg.apiuri.equals("") && cfg.apisrv.equals(""))
      api.configure(cfg.dname.c_str(), cfg.deviceId); 
    else
      api.configure(cfg.dname.c_str(), cfg.deviceId, cfg.apiuri.c_str(), cfg.apisrv.c_str(), cfg.apiprt); 
    api.authorize(cfg.apiusr.c_str(), cfg.apipss.c_str());
    // api.dev = true;
    cfg.isNewAPIConfig=false; // flag for config via BLE
    delay(1000);
  }
}

void apiLoop() {
  if (v25.size()==0 && wifiOn && cfg.isApiEnable() && apiIsConfigured() && resetvar != 0) {
    Serial.print("-->[API] writing to ");
    Serial.print(""+String(api.ip)+"..");
    bool status = api.write(0,apm25,apm10,humi,humiS,cfg.lat,cfg.lon,cfg.alt,cfg.spd,cfg.stime);
    int code = api.getResponse();
    if(status) {
      Serial.println("done. ["+String(code)+"]");
      statusOn(bit_cloud);
      dataSendToggle = true;
    /*
      Serial.print("    Data stored: ");
      Serial.print(apm25);
      Serial.print(" ");
      Serial.print(apm10);
      Serial.print(" ");
      Serial.print(cfg.lat);
      Serial.print(" ");
      Serial.print(cfg.lon);
      Serial.print(" ");
      Serial.print(cfg.alt);
      Serial.print(" ");
      Serial.println(cfg.spd);    
    */
    }
    else {
      Serial.println("fail! ["+String(code)+"]");
      statusOff(bit_cloud);
      setErrorCode(ecode_api_write_fail);
      if (code == -1) {
        Serial.println("-->[E][API] publish error (-1)");
        delay(1000);
      }
    }
  }
}

/******************************************************************************
*   I N F L U X D B   M E T H O D S
******************************************************************************/

bool influxDbIsConfigured(){
  return cfg.ifxdb.length()>0 && cfg.ifxip.length()>0 && cfg.dname.length()>0;
}

void influxDbInit() {
  if(wifiOn && influxDbIsConfigured()) {
    Serial.println("-->[INFLUXDB] connecting..");
    influx.configure(cfg.ifxdb.c_str(), cfg.ifxip.c_str()); //third argument (port number) defaults to 8086
    Serial.print("-->[INFLUXDB] Using HTTPS: ");
    Serial.println(influx.isSecure()); //will be true if you've added the InfluxCert.hpp file.
    cfg.isNewIfxdbConfig=false; // flag for config via BLE
    delay(1000);
  }
}

/**
 * @influxDbParseFields:
 *
 * Supported:
 * "id","pm1","pm25","pm10,"hum","tmp","humS","tmpS","lat","lng","alt","spd","stime","tstp"
 *
 */
void influxDbParseFields(char* fields){
  sprintf(
    fields,
    "pm1=%u,pm25=%u,pm10=%u,hum=%f,tmp=%f,humS=%f,tmpS=%f,lat=%f,lng=%f,alt=%f,spd=%f,stime=%i,tstp=%u",
    0,apm25,apm10,humi,temp,humiS,tempS,cfg.lat,cfg.lon,cfg.alt,cfg.spd,cfg.stime,0
  );
}

void influxDbAddTags(char* tags) {
  sprintf(tags,"mac=%04X%08X",(uint16_t)(cfg.chipid >> 32),(uint32_t)cfg.chipid);
}

bool influxDbWrite() {
  char tags[64];
  influxDbAddTags(tags);
  char fields[256];
  influxDbParseFields(fields);
  return influx.write(cfg.dname.c_str(), tags, fields);
}

void influxDbLoop() {
  if(v25.size()==0 && wifiOn && cfg.isIfxEnable() && influxDbIsConfigured() && resetvar != 0){
    int ifx_retry = 0;
    Serial.print("-->[INFLUXDB] writing to ");
    Serial.print("" + cfg.ifxip + "..");
    while(!influxDbWrite() && ( ifx_retry++ < IFX_RETRY_CONNECTION )){
      Serial.print(".");
      delay(200);
    }
    if(ifx_retry > IFX_RETRY_CONNECTION ) {
      Serial.println("failed!\n-->[E][INFLUXDB] write error, try wifi restart..");
      statusOff(bit_cloud);
      setErrorCode(ecode_ifdb_write_fail);
      wifiRestart();
    }
    else {
      Serial.println("done. ["+String(influx.getResponse())+"]");
      statusOn(bit_cloud);
      dataSendToggle = true;
    }
  }
}

/******************************************************************************
*   W I F I   M E T H O D S
******************************************************************************/

class MyOTAHandlerCallbacks: public OTAHandlerCallbacks{
  void onStart(){
    gui.showWelcome();
    gui.welcomeAddMessage("Upgrading..");
  };
  void onProgress(unsigned int progress, unsigned int total){
    gui.showProgress(progress,total);
  };
  void onEnd(){
    gui.welcomeAddMessage("");
    gui.welcomeAddMessage("success!");    delay(1000);
    gui.welcomeAddMessage("rebooting.."); delay(500);
  }
  void onError(){
    gui.welcomeAddMessage("");
    gui.welcomeAddMessage("error, try again!"); delay(2000);
  }
};

void otaLoop(){
  timerAlarmDisable(timer);                         // disable interrupt
  if(wifiOn)ota.loop();
  timerAlarmEnable(timer);                         // enable interrupt
}

void otaInit(){
  ota.setup("CanAirIO","CanAirIO");
  ota.setCallbacks(new MyOTAHandlerCallbacks());
}

bool wifiCheck(){
  wifiOn = WiFi.isConnected();
  if(wifiOn)statusOn(bit_wan);  // TODO: We need validate internet connection
  else {
    statusOff(bit_cloud);
    statusOff(bit_wan);
  }
  return wifiOn;
}

void wifiConnect(const char* ssid, const char* pass) {
  Serial.print("-->[WIFI] Connecting to "); Serial.print(ssid);
  WiFi.begin(ssid, pass);
  WiFi.setHostname("CanAirIO");
  int wifi_retry = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retry++ < WIFI_RETRY_CONNECTION) {
    Serial.print(".");
    delay(500);           // increment this delay on possible reconnect issues
  }
  if(wifiCheck()){
    cfg.isNewWifi=false;  // flag for config via BLE
    Serial.println("done\n-->[WIFI] connected!");
    Serial.print("-->[WIFI][IP]"); Serial.println(WiFi.localIP());
    otaInit();
  }
  else{
    Serial.println("fail!\n-->[E][WIFI] disconnected!");
    setErrorCode(ecode_wifi_fail);
  }
}

void wifiInit(){
  if(cfg.wifiEnable && cfg.ssid.length() > 0 && cfg.pass.length() > 0) {
    wifiConnect(cfg.ssid.c_str(), cfg.pass.c_str());
  }
}

void wifiStop(){
  if(wifiOn){
    Serial.println("-->[WIFI] Disconnecting..");
    WiFi.disconnect(true);
    wifiOn = false;
    delay(1000);
  }
}

void wifiRestart(){
  wifiStop();
  wifiInit();
}

void wifiLoop(){
  wifiRSSI();
  if(v25.size()==0 && cfg.wifiEnable && cfg.ssid.length()>0 && !wifiCheck()) {
    wifiConnect(cfg.ssid.c_str(), cfg.pass.c_str());
    influxDbInit();
    apiInit();
  }
}

void wifiRSSI(){
  if (wifiOn)
    rssi = WiFi.RSSI();
  else
    rssi = 0;
}

/******************************************************************************
*   B L U E T O O T H  M E T H O D S
******************************************************************************/
class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
      Serial.println("-->[BLE] onConnect");
      statusOn(bit_paired);
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("-->[BLE] onDisconnect");
      statusOff(bit_paired);
      deviceConnected = false;
    };
}; // BLEServerCallbacks

class MyConfigCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        if(cfg.save(value.c_str())){
          triggerSaveIcon=0;
          cfg.reload();
          if(cfg.isNewWifi){
            wifiRestart();
            apiInit();
            influxDbInit();
          }
          if(cfg.isNewIfxdbConfig) influxDbInit();
          if(cfg.isNewAPIConfig) apiInit();
          if(!cfg.wifiEnable) wifiStop();
        }
        else{
          setErrorCode(ecode_invalid_config);
        }
        pCharactConfig->setValue(cfg.getCurrentConfig().c_str());
        pCharactData->setValue(getSensorData().c_str());
      }
    }
};

void bleServerInit(){
  // Create the BLE Device
  BLEDevice::init("CanAirIO_ESP32");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic for PM 2.5
  pCharactData = pService->createCharacteristic(
      CHARAC_DATA_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  // Create a BLE Characteristic for Sensor mode: STATIC/MOVIL
  pCharactConfig = pService->createCharacteristic(
      CHARAC_CONFIG_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  // Create a Data Descriptor (for notifications)
  pCharactData->addDescriptor(new BLE2902());
  // Saved current sensor data
  pCharactData->setValue(getSensorData().c_str());
  // Setting Config callback
  pCharactConfig->setCallbacks(new MyConfigCallbacks());
  // Saved current config data
  pCharactConfig->setValue(cfg.getCurrentConfig().c_str());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("-->[BLE] GATT server ready. (Waiting for client)");
}

void bleLoop(){
  // notify changed value
  if (deviceConnected && v25.size()==0) { // v25 test for get each ~5 sec aprox
    Serial.println("-->[BLE] sending notification..");
    pCharactData->setValue(getNotificationData().c_str());  // small payload for notification
    pCharactData->notify();
    pCharactData->setValue(getSensorData().c_str());        // load big payload for possible read
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("-->[BLE] start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

/******************************************************************************
*   R E S E T
******************************************************************************/

void resetLoop(){
  if (wifiOn){    
        if (resetvar == 1199) {      
        resetvar = 0;
        delay(45000);   // 45 seconds, reset at 30 seconds
    }
    resetvar = resetvar + 1;
  }
}

/******************************************************************************
*  M A I N
******************************************************************************/

void IRAM_ATTR resetModule(){
  Serial.println("\n-->[INFO] Watchdog reached, rebooting..");
  esp_wifi_disconnect();
  delay(200);
  esp_wifi_stop();
  delay(200);
  esp_wifi_deinit();
  delay(200);
  ESP.restart();
}

void enableWatchdog(){
  timer = timerBegin(0, 80, true);                 // timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true); // setting callback
  //timerAlarmWrite(timer, 30000000, false);         // set time in us (30s)
  timerAlarmWrite(timer, 25000000, false);         // set time in us (25s)
  timerAlarmEnable(timer);                         // enable interrupt
}

void setup(){
#ifdef TTGO_TQ
  pinMode(IP5306_2, INPUT);
  pinMode(IP5306_3, INPUT);
#endif
  pinMode(21, INPUT_PULLUP);  // i2c
  pinMode(22, INPUT_PULLUP);  // i2c
  pinMode(pin_rx1, INPUT);
  pinMode(pin_rx2, INPUT);
  pinMode(pin_rx3, INPUT);
  pinMode(pin_tx1, OUTPUT);
  pinMode(pin_tx2, OUTPUT);
  pinMode(pin_tx3, OUTPUT);
  enableWatchdog();  // enable timer for reboot in any loop blocker   //////////////////
  delay(1000);
  Serial.begin(115200);
  gui.displayInit(u8g2);
  gui.showWelcome();
  cfg.init("canairio");
  Serial.println("\n== INIT SETUP ==\n");
  Serial.println("-->[INFO] ESP32MAC: "+String(cfg.deviceId));
  //gui.welcomeAddMessage("Sensor test..");
  // sensorInit();
  am2320.begin();
  //Serial.println("SHT31 test");
  sht31.begin(0x44);
  //if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
  //    Serial.println("Couldn't find SHT31");
  //    while (1) delay(1);
  //  }
  bleServerInit();
  //gui.welcomeAddMessage("GATT server..");
  if(cfg.ssid.length()>0) gui.welcomeAddMessage("WiFi:"+cfg.ssid);
  else gui.welcomeAddMessage("WiFi radio test..");
  wifiInit();
  gui.welcomeAddMessage("InfluxDB..");
  influxDbInit();
  gui.welcomeAddMessage("CanAirIO API..");
  apiInit();
  //
  //gui.welcomeAddMessage("Sensor test..");
  sensorInit();
  delay(1000);
  //
  pinMode(LED,OUTPUT);
  gui.welcomeAddMessage("==SETUP READY==");
  //enableWatchdog();  // enable timer for reboot in any loop blocker
  delay(500);
}

void loop(){
  gui.pageStart();
  sensorLoop();    // read sensor data and showed it
  averageLoop();   // calculated of sensor data average
  humidityLoop();  // read AM2320
  batteryloop();   // battery charge status
  bleLoop();       // notify data to connected devices
  wifiLoop();      // check wifi and reconnect it
  apiLoop();       // CanAir.io API publication
  influxDbLoop();  // influxDB publication
  statusLoop();    // update sensor status GUI
  otaLoop();       // check for firmware updates
  gui.pageEnd();   // gui changes push
  delay(800);
  timerWrite(timer, 0);  //reset timer (feed watchdog)
  resetLoop();     // reset every 20 minutes with Wifion
}