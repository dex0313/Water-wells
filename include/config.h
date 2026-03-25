#pragma once
#include <IPAddress.h>


#define NODE_ID 15
#define BASE_ID 0

//Wifi
#define WIFI_SSID       "Wifi"
#define WIFI_PASSWORD   ""

// #define WIFI_STATIC_IP  true

// #define WIFI_IP         IPAddress(192,168,88,13)
// #define WIFI_GATEWAY    IPAddress(192,168,88,1)
// #define WIFI_SUBNET     IPAddress(255,255,0,0)
// #define WIFI_DNS        IPAddress(8,8,8,8)

#define WIFI_RETRY_INTERVAL  5000
#define WIFI_RESTART_TIMEOUT 300000

//MQTT
#define MQTT_HOST "192.168.1.215"
#define MQTT_PORT 1883

#define MQTT_CLIENT_ID "lora_base"
#define MQTT_USER "mqtt"
#define MQTT_PASS "moloko12"

#define MQTT_RETRY_INTERVAL 5000

//LoRa pins
// #define LORA_RX 16
// #define LORA_TX 17
// #define LORA_AUX 4
// #define LORA_M0 18
// #define LORA_M1 5

#define RELAY_PIN 15
#define RELAY_ON  LOW
#define RELAY_OFF HIGH