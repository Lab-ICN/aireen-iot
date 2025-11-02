//
// Created by yogarn on 8/26/25.
//

#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

class MqttClient {
public:
    MqttClient(const char *server, int port, const char *clientId);

    void begin();

    void loop();

    void reconnect();

    bool publish(const char *topic, const char *payload);

    bool subscribe(const char *topic);

    void setCallback(MQTT_CALLBACK_SIGNATURE);

private:
    const char *_server;
    int _port;
    const char *_clientId;

    WiFiClient _wifiClient;
    PubSubClient _client;
};

#endif //MQTT_CLIENT_H
