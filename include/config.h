#pragma once
#include <IPAddress.h>

#define WIFI_SSID       "Wifi"
#define WIFI_PASSWORD   ""

#define WIFI_STATIC_IP  true

#define WIFI_IP         IPAddress(192,168,1,113)
#define WIFI_GATEWAY    IPAddress(192,168,1,9)
#define WIFI_SUBNET     IPAddress(255,255,255,0)
#define WIFI_DNS        IPAddress(8,8,8,8)

#define WIFI_RETRY_INTERVAL  5000
#define WIFI_RESTART_TIMEOUT 300000

#define MQTT_HOST "192.168.1.215"
#define MQTT_PORT 1883

#define MQTT_CLIENT_ID "lora_base"
#define MQTT_USER "mqtt"
#define MQTT_PASS "moloko12"

#define MQTT_RETRY_INTERVAL 5000