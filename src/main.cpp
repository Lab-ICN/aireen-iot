#include "config.h"

#include <Arduino.h>
#include <gravity_tds.h>
#include <ldr.h>
#include <mqtt_client.h>
#include <WiFi.h>

GravityTDS gravityTds;
LDR ldr(LDR_PIN, LDR_THRESHOLD, LDR_REFERENCE);
MqttClient mqtt(MQTT_HOST, MQTT_PORT, MQTT_CLIENT);

float temperature = 25, tdsValue = 0;
int ldrValue = 0;
int ldrVoltage = 0;
bool isLdrBright = false;

unsigned long previousMillis = 0;
const unsigned long interval = 5000;

void messageCallback(char *topic, byte *payload, unsigned int length) {
    Serial.print("[MQTT] Message on ");
    Serial.print(topic);
    Serial.print(": ");

    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char) payload[i];
    }
    Serial.println(message);

    if (message.equalsIgnoreCase("ping")) {
        mqtt.publish((MQTT_TOPIC "cmd"), "pong");
        Serial.println("[MQTT] Replied with pong");
    }
}

void setup() {
    Serial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");

    mqtt.begin();
    mqtt.setCallback(messageCallback);
    mqtt.reconnect();
    mqtt.subscribe(MQTT_TOPIC "cmd");
    Serial.println("[MQTT] Subscribed to all subtopics");

    gravityTds.setPin(TDS_PIN);
    gravityTds.setAref(3.3);
    gravityTds.setAdcRange(4096);
    gravityTds.begin();
}

void loop() {
    mqtt.loop();

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        gravityTds.setTemperature(temperature);
        gravityTds.update();
        tdsValue = gravityTds.getTdsValue();

        ldrValue = ldr.readRaw();
        ldrVoltage = ldr.readVoltage();
        isLdrBright = ldr.isBright();

        mqtt.publish((MQTT_TOPIC "sensors/ldr"), String(ldrValue).c_str());
        mqtt.publish((MQTT_TOPIC "sensors/tds"), String(tdsValue).c_str());

        mqtt.publish(MQTT_TOPIC "system/uptime", String(millis()/1000).c_str());
        mqtt.publish(MQTT_TOPIC "system/heap", String(ESP.getFreeHeap()).c_str());
        mqtt.publish(MQTT_TOPIC "system/rssi", String(WiFi.RSSI()).c_str());

        Serial.print("[DATA] LDR: ");
        Serial.print(ldrValue);
        Serial.print(" | TDS: ");
        Serial.println(tdsValue);
    }
}
