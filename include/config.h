#pragma once
#include <IPAddress.h>
#include "secret.h"

#define NODE_ID 15
#define BASE_ID 0

// #define WIFI_STATIC_IP  true

// #define WIFI_IP         IPAddress(192,168,88,13)
// #define WIFI_GATEWAY    IPAddress(192,168,88,1)
// #define WIFI_SUBNET     IPAddress(255,255,0,0)
// #define WIFI_DNS        IPAddress(8,8,8,8)

#define WIFI_RETRY_INTERVAL  5000
#define WIFI_RESTART_TIMEOUT 300000

//MQTT

#define MQTT_CLIENT_ID "lora_base"
#define MQTT_RETRY_INTERVAL 5000
#define MQTT_BUFFER_SIZE    1024

// Relay
#define RELAY_PIN 26
#define RELAY_ON  LOW
#define RELAY_OFF HIGH

// Motor
#define MOTOR_PIN 25
#define MOTOR_THRESHOLD 2000

// Sensor
#define SENSOR_INTERVAL 60000

// LoRa protocol
#define PKT_DATA 1
#define PKT_CMD  2
#define PKT_ACK  3

// LoRa pins
#define LORA_RX 16
#define LORA_TX 17
#define LORA_AUX 4
#define LORA_M0 18
#define LORA_M1 5

// LoRa modes
#define LORA_MODE_NORMAL 0
#define LORA_MODE_SLEEP 1
#define LORA_MODE_WAKEUP 2
#define LORA_MODE_POWER 3