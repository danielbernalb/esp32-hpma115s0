
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
#include <GUIUtils.hpp>
#include <vector>
#include <sps30.h>
#include <sensirion_uart.h>
#include "main.h"
#include "status.h"

/******************************************************************************
* S E T U P  B O A R D   A N D  F I E L D S
* ---------------------
* please select board on platformio.ini file
******************************************************************************/

void setup() {
	Serial.begin(115200);
	sensirion_uart_open();

    while (sps30_probe() != 0) {
        Serial.println("probe failed");
        delay(1000);
    }

    if (sps30_start_measurement() != 0) {
        Serial.println("error starting measurement");
    }
}

void loop() {
    struct sps30_measurement measurement;
    s16 ret;

    while(true) {
        delay(1000);
        ret = sps30_read_measurement(&measurement);

        if (ret < 0) {
          Serial.println("read measurement failed");
        } else {
            if (SPS_IS_ERR_STATE(ret)) {
              Serial.println("Measurements may not be accurate");
            }else{
                Serial.print("pm2.5 :");Serial.print(measurement.mc_2p5);
                Serial.print("  pm10.0 :");Serial.println(measurement.mc_10p0);
            }
        }
    }

    sps30_stop_measurement();
    sensirion_uart_close();
}