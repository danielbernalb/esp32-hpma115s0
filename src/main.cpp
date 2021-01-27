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
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <DHT.h>
#include <MHZ19.h>
#include <GUIUtils.hpp>
#include <vector>
#include <sps30.h>
#include "main.h"
#include "status.h"

/******************************************************************************
* S E T U P  B O A R D   A N D  F I E L D S
* ---------------------
* please select board on platformio.ini file
******************************************************************************/

#ifdef WEMOSOLED // display via i2c for WeMOS OLED board & TTGO18650
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 4, 5, U8X8_PIN_NONE);
#elif ESP32DevKit // display via i2c for ESP32-DevKitC - U series
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, OLEDclockd, OLEDdatad, U8X8_PIN_NONE);
#elif ESP32S // display via i2c for ESP32S
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE,U8X8_PIN_NONE,U8X8_PIN_NONE);
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
#elif ESP32DevKit
#define HPMA_RX PINRXSENd  // config for ESP32DevKit
#define HPMA_TX PINTXSENd
#elif ESP32S
#define HPMA_RX 17  // config for D1MIN1 board
#define HPMA_TX 16
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

#ifdef SENSIRION
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
#endif

void sensorInit(){
#ifdef HONEYWELL
  Serial.println("-->[HPMA] starting hpma115S0 sensor..");
  delay(100);
  #ifndef TTGO_TQ
  hpmaSerial.begin(9600, SERIAL_8N1, HPMA_RX, HPMA_TX);
  #else
  if(WrongSerialData == false){
    hpmaSerial.begin(9600, SERIAL_8N1, HPMA_RX, HPMA_TX);
  }
  delay(100);
  #endif
#elif PANASONIC
  Serial.println("-->[SN] starting SN-GCJA5 sensor..");
  delay(100);
  #ifndef TTGO_TQ
  hpmaSerial.begin(9600, SERIAL_8N1, HPMA_RX, HPMA_TX);
  #else
  if(WrongSerialData == false){
    hpmaSerial.begin(9600, SERIAL_8N1, HPMA_RX, HPMA_TX);
  }
  delay(100);
  #endif
#elif SENSIRION
// Begin communication channel
 SP30_COMMS1.begin(115200, SERIAL_8N1, HPMA_RX, HPMA_TX);
  Serial.println(F("-->[SPS30] starting SPS30 sensor.."));
  if (sps30.begin(&SP30_COMMS1) == false){
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
#elif MHZ14
 Serial.println("-->[MHZ14] starting MHZ14 CO2 sensor..");
  delay(100);
  MHZ14Serial.begin(9600, SERIAL_8N1, HPMA_RX, HPMA_TX);
  if(WrongSerialData == false){
    MHZ14Serial.begin(9600, SERIAL_8N1, HPMA_RX, HPMA_TX);
  }
  myMHZ19.begin(MHZ14Serial);
  myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))
  delay(100);
#elif CM1106
  Serial.println("-->[CM1106] starting CM1106 CO2 sensor..");
  delay(100);
  co2cm1106.begin(9600, SERIAL_8N1, HPMA_RX, HPMA_TX);
  Serial.print("-->[CM1106] Stop Autocalibration: ");
  static byte cmd[10] = {0x11, 0x07, 0x10, 0x64, 0x02, 0x07, 0x01, 0x90, 0x64, 0x76}; // Command Close ABC
  static byte response[4] = {0};
  co2cm1106.write(cmd, 10);
  co2cm1106.readBytes(response, 4);
  if (response[0] == 0x16 && response[1] == 0x01 && response[2] == 0x10 && response[3] == 0xD9) Serial.println("done");
    else Serial.println(" failed");
#endif
}

void wrongDataState(){
  setErrorCode(ecode_sensor_read_fail);
  gui.displaySensorAverage(apm25);
  gui.displaySensorData(0,0,chargeLevel,0.0,0.0,0);
#ifdef HONEYWELL
  Serial.print("-->[E][HPMA] !wrong data!");
  #ifndef TTGO_TQ
    hpmaSerial.end();
  #endif
#elif PANASONIC
  Serial.print("-->[E][SNGC] !wrong data!");
  #ifndef TTGO_TQ
    hpmaSerial.end();
  #endif
#else
  Serial.print("-->[E][SPS30] !wrong data!");
#endif
  statusOff(bit_sensor);
  WrongSerialData = true;
  sensorInit();
  delay(500);
}

/***
 * Average methods
 **/

void saveDataForAverage(unsigned int pm25, unsigned int pm10){
  v25.push_back(pm25);
  v10.push_back(pm10);
}

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

void averageLoop(){
  if (v25.size() >= cfg.stime){
    apm25 = getPM25Average();  // global var for display
    apm10 = getPM10Average();
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
  saveDataForAverage(pm25, pm10);
  WrongSerialData = false;
}

/***
 * PM2.5 and PM10 read and visualization
 **/

void sensorLoop(){

#ifdef MHZ14
  int CO2; 
  CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
  pm10 = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
  Serial.print("-->[MHZ14] read > done!");
  statusOn(bit_sensor);
  pm25 = CO2;
    if (pm25 < 10000 && pm10 < 100){
        showValues(pm25, pm10);
  }
#elif CM1106
  int CO2;
  CO2 = CO2CM1106val();
  Serial.print("-->[CM1106] read > done!");
  statusOn(bit_sensor);
  pm25 = CO2;
  pm10 = 0;
    if (pm25 < 10000 && pm10 < 100){
        showValues(pm25, pm10);
  }
#endif

#ifndef SENSIRION
#ifndef MHZ14
#ifndef CM1106
  int try_sensor_read = 0;
  String txtMsg = "";
  while (txtMsg.length() < 32 && try_sensor_read++ < SENSOR_RETRY){
    while (hpmaSerial.available() > 0){
      char inChar = hpmaSerial.read();
      txtMsg += inChar;
#ifdef HONEYWELL
      Serial.print("-->[HPMA] read " + String(getLoaderChar()) + "\r");
#else
      Serial.print("-->[SNGC] read " + String(getLoaderChar()) + "\r");
#endif
    }
#ifdef HONEYWELL
    Serial.print("-->[HPMA] read " + String(getLoaderChar()) + "\r");
#elif PANASONIC
    Serial.print("-->[SNGC] read " + String(getLoaderChar()) + "\r");
#endif
  }
  if (try_sensor_read > SENSOR_RETRY){
    setErrorCode(ecode_sensor_timeout);
#ifdef HONEYWELL
    Serial.println("-->[HPMA] read > fail!");
    Serial.println("-->[E][HPMA] disconnected ?");
#else
    Serial.println("-->[SNGC] read > fail!");
    Serial.println("-->[E][SNGC] disconnected ?");
#endif
    delay(500); // waiting for sensor..
  }
#endif
#endif
#endif

#ifdef PANASONIC
  if (txtMsg[0] == 02){
    Serial.print("-->[SNGC] read > done!");
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

#elif HONEYWELL // HONEYWELL
  if (txtMsg[0] == 66){
    if (txtMsg[1] == 77){
      Serial.print("-->[HPMA] read > done!");
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

#elif SENSIRION
  delay(35); //Delay for sincronization
  do {
    ret = sps30.GetValues(&val);
    if (ret == ERR_DATALENGTH){
      if (error_cnt++ > 3) {
        ErrtoMess((char *)"-->[E][SPS30] Error during reading values: ", ret);
        return;
      }
      delay(1000);
    }
    else if (ret != ERR_OK){
      ErrtoMess((char *)"-->[E][SPS30] Error during reading values: ", ret);
      return;
    }
  } while (ret != ERR_OK);

  Serial.print("-->[SPS30] read > done!");
  statusOn(bit_sensor);

  pm25 = round(val.MassPM2);
  pm10 = round(val.MassPM10);

  if (pm25 < 1000 && pm10 < 1000){
    showValues(pm25, pm10);
  }
  else
    wrongDataState();
#endif
}

#ifdef CM1106
int CO2CM1106val() {
  static byte cmd[4] = {0x11, 0x01, 0x01, 0xED}; // Commands 0x01 Read ppm, 0x10 open/close ABC, 0x03 Calibrate 
  static byte response[8] = {0};                 // response 0x16, 0x05, 0x01, DF1, DF2, DF3, DF4, CRC.  ppm=DF1*256+DF2
  co2cm1106.write(cmd, 4);
  co2cm1106.readBytes(response, 8);
  int crc = 0;
  for (int i = 0; i < 7; i++) crc += response[i];
  crc = 256 - crc%256;
  if (((int) response[0] == 0x16) && ((int)response[7] == crc)) {
    unsigned int responseHigh = (unsigned int) response[3];
    unsigned int responseLow = (unsigned int) response[4];
    return (256 * responseHigh) + responseLow;
  } else {
    while(co2cm1106.available() > 0)  char t = co2cm1106.read();  // Clear serial input buffer;
    return -1; 
  }
}
#endif

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
 #ifdef BME280S 
  humi = bme.readHumidity();
  temp = bme.readTemperature();
  if (isnan(humi)){
    humi = 0.0;
    bme.begin(0x76);
  }
  if (isnan(temp))
    temp = 0.0;
  Serial.println("-->[BME280] Humidity: "+String(humi)+" % Temp: "+String(temp)+" °C");
 #elif DHT22S
  humi = dht.readHumidity();
  temp = dht.readTemperature();
  if (isnan(humi)){
    humi = 0.0;
    dht.begin();
  }
  if (isnan(temp))
    temp = 0.0;
  Serial.println("-->[DHT22] Humidity: "+String(humi)+" % Temp: "+String(temp)+" °C");
 #elif AHT10
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serial.println("-->[AHT10] Humidity: "+String(humidity.relative_humidity)+" % Temp: "+String(temp.temperature)+" °C");
 #else
  humi = am2320.readHumidity();
  temp = am2320.readTemperature();
  if (isnan(humi)){
    humi = 0.0;
    am2320.begin();
  }
  if (isnan(temp))
    temp = 0.0;
  Serial.println("-->[AM2320] Humidity: "+String(humi)+" % Temp: "+String(temp)+" °C");
 #endif 
}

void humidityLoop(){
  if (v25.size() == 0){
    getHumidityRead();
  }
}

void humTempInit(){
#ifdef ESP32DevKit
 #ifdef BME280S
   //Wire.begin(16, 21);   //I2C_SDA, I2C_SCL
   bme.begin(0x76, 16, 21);
 #elif DHT22S
   dht.begin();
 #elif AHT10
 I2CBME.begin(18, 19);
  if (! aht.begin(&I2CBME)) {
    Serial.println("Could not find AHT? Check wiring");
    delay(10);
  }
  Serial.println("AHT10 found");
 #else
   //Wire.begin(16,21);   //I2C_SDA, I2C_SCL
   Wire.begin(HUMSDAd,HUMSCLd);   //I2C_SDA, I2C_SCL
 #endif
#elif ESP32S
 #ifdef BME280S
   //Wire.begin(16, 21);   //I2C_SDA, I2C_SCL
   //bme.begin(0x76, 16, 21);
   bme.begin(0x76);
 #elif DHT22S
   dht.begin();
 #elif AHT10
 I2CBME.begin(23, 22);
  if (! aht.begin(&I2CBME)) {
    Serial.println("Could not find AHT? Check wiring");
    delay(10);
  }
  Serial.println("AHT10 found");
 #else
   am2320.begin();   //I2C_SDA, I2C_SCL
 #endif
#else
 #ifdef BME280S
   bme.begin(0x76);
 #elif DHT22S
   dht.begin();
 #else
   am2320.begin();   //I2C_SDA, I2C_SCL
 #endif
#endif
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
    bool status = api.write(0,apm25,apm10,humi,temp,cfg.lat,cfg.lon,cfg.alt,cfg.spd,cfg.stime);
    int code = api.getResponse();
    if(status) {
      Serial.println("done. ["+String(code)+"]");
      statusOn(bit_cloud);
      dataSendToggle = true;
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
 * "id","pm1","pm25","pm10,"hum","tmp","lat","lng","alt","spd","stime","tstp"
 *
 */
void influxDbParseFields(char* fields){
  sprintf(
    fields,
    "pm1=%u,pm25=%u,pm10=%u,hum=%f,tmp=%f,lat=%f,lng=%f,alt=%f,spd=%f,stime=%i,tstp=%u",
    0,apm25,apm10,humi,temp,cfg.lat,cfg.lon,cfg.alt,cfg.spd,cfg.stime,0
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
        if (resetvar == 43199) {        // 12 horas RESET
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
  timerAlarmWrite(timer, 30000000, false);         // set time in us (30s)
  timerAlarmEnable(timer);                         // enable interrupt
}

void setup(){
#ifdef TTGO_TQ
  pinMode(IP5306_2, INPUT);
  pinMode(IP5306_3, INPUT);
#endif
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  Serial.begin(115200);
  gui.displayInit(u8g2);
  gui.showWelcome();
  cfg.init("canairio");
  Serial.println("\n== INIT SETUP ==\n");
  Serial.println("-->[INFO] ESP32MAC: "+String(cfg.deviceId));
  enableWatchdog();  // enable timer for reboot in any loop blocker
  gui.welcomeAddMessage("Sensors test..");
  sensorInit();
  humTempInit();
  bleServerInit();
  gui.welcomeAddMessage("GATT server..");
  if(cfg.ssid.length()>0) gui.welcomeAddMessage("WiFi:"+cfg.ssid);
  else gui.welcomeAddMessage("WiFi radio test..");
  wifiInit();
  gui.welcomeAddMessage("CanAirIO API..");
  influxDbInit();
  apiInit();
  pinMode(LED,OUTPUT);
  gui.welcomeAddMessage("==SETUP READY==");
  delay(500);
#ifdef MHZ14
  gui.welcomeAddMessage("PREHEAT timer 180 seconds:");
  Serial.print("Preheat timer 180 seconds:");
   for (resetvar=0; resetvar<=181; resetvar++) {
    delay(1000);
    gui.welcomeAddMessage(String(resetvar));
    Serial.println(resetvar);
    timerWrite(timer, 0);  //reset timer (feed watchdog)
   }
#endif
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
#ifdef HONEYWELL
  delay(500);
#elif PANASONIC
  delay(500);
#else
  delay(900);
#endif
  timerWrite(timer, 0);  //reset timer (feed watchdog)
  resetLoop();     // reset every 20 minutes with Wifion
}