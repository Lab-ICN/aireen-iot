#include "config.h"

#include <Arduino.h>
#include <gravity_tds.h>
#include <ldr.h>
#include <mqtt_client.h>
#include <WiFi.h>
#include "esp_sleep.h"

#include <ph_meter.h>
#include <EEPROM.h>

GravityTDS gravityTds;
LDR ldr(LDR_PIN, LDR_THRESHOLD, LDR_REFERENCE);
MqttClient mqtt(MQTT_HOST, MQTT_PORT, MQTT_CLIENT);

float temperature = 25, tdsValue = 0;
int ldrValue = 0;
int ldrVoltage = 0;
bool isLdrBright = false;

float voltage, phValue;
DFRobot_PH ph;

const uint64_t SLEEP_DURATION_SEC = 5;

void setup() {
    Serial.begin(115200);
    delay(1000);

    EEPROM.begin(32);

    Serial.println("\n=== Booting from Deep Sleep ===");

    Serial.print("[WiFi] Connecting");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        Serial.print(".");
        retry++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] Failed to connect, going back to sleep");
        esp_deep_sleep(SLEEP_DURATION_SEC * 1000000ULL);
    }

    Serial.println("\n[WiFi] Connected!");

    mqtt.begin();
    Serial.print("[MQTT] Connecting...");
    mqtt.loop();
    Serial.println(" connected!");

    gravityTds.setPin(TDS_PIN);
    gravityTds.setAref(TDS_AREF);
    gravityTds.setAdcRange(ADC_RANGE);
    gravityTds.begin();

    gravityTds.setTemperature(temperature);
    gravityTds.update();
    tdsValue = gravityTds.getTdsValue();

    ph.begin();
    voltage = analogRead(PH_PIN) * 3300.0 / ADC_RANGE;
    phValue = ph.readPH(voltage, temperature);

    ldrValue = ldr.readRaw();
    ldrVoltage = ldr.readVoltage();
    isLdrBright = ldr.isBright();

    mqtt.publish((MQTT_TOPIC "sensors/ldr"), String(ldrValue).c_str());
    mqtt.publish((MQTT_TOPIC "sensors/tds"), String(tdsValue).c_str());
    mqtt.publish((MQTT_TOPIC "sensors/ph"), String(phValue, 2).c_str());

    mqtt.publish(MQTT_TOPIC "system/heap", String(ESP.getFreeHeap()).c_str());
    mqtt.publish(MQTT_TOPIC "system/rssi", String(WiFi.RSSI()).c_str());

    Serial.printf("[DATA] LDR: %d | TDS: %.2f | pH: %.2f\n",
                  ldrValue, tdsValue, phValue);

    unsigned long start = millis();
    while (millis() - start < 500) {
        mqtt.loop();
        delay(10);
    }

    Serial.println("[SLEEP] Disconnecting WiFi...");
    WiFi.disconnect(true);

    Serial.printf("[SLEEP] Entering deep sleep for %llu seconds...\n",
                  SLEEP_DURATION_SEC);
    esp_deep_sleep(SLEEP_DURATION_SEC * 1000000ULL);
}

void loop() {
}
