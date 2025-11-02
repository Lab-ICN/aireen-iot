/**
 * @brief Configuration file
 */

#pragma once

// device config
#define NODE_ID 1

// pin map
#define LDR_PIN 35
#define TDS_PIN 34
#define PH_PIN A1

// hardware constants
#define TDS_AREF 3.3        // reference voltage on ADC, default 3.3V on ESP32
#define TDS_ADC_RANGE 4096  // 4096 for 12-bit ADC
#define LDR_REFERENCE 5.0

// environmental constants
#define TDS_TEMPERATURE 25  // default temperature for TDS calculations
#define LDR_THRESHOLD 5

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// mqtt config
#define MQTT_HOST ""
#define MQTT_PORT 1883
#define MQTT_CLIENT "esp32/" STR(NODE_ID) "/"
#define MQTT_TOPIC "aireen/esp32/" STR(NODE_ID) "/"

// wifi config
#define WIFI_SSID ""
#define WIFI_PASS ""