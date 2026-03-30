#include <HardwareSerial.h>
#include <Arduino.h>
#include "lora_manager.h"
#include "mqtt_manager.h"
#include "config.h"
#include "sensor_manager.h"

HardwareSerial loraSerial(2);

// ============================================================
// Packet structures
// ============================================================

#pragma pack(push,1)
struct MeshPacket {
    uint8_t version;
    uint8_t type;
    uint16_t source;
    uint16_t destination;
    uint16_t packet_id;
    uint8_t payload_size;
    uint8_t payload[48];
    uint16_t crc;
};
#pragma pack(pop)

#pragma pack(push,1)
struct DataPayload {
    float t;
    float h;
    float p;
    uint8_t relay;
    uint8_t motor;
};
#pragma pack(pop)

// ============================================================
// CRC-16/Modbus
// ============================================================

static uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

// ============================================================
// LoRa E22 helpers
// ============================================================

static bool waitAux(uint32_t timeout = 1000) {
    uint32_t start = millis();
    while (digitalRead(LORA_AUX) == LOW) {
        if (millis() - start > timeout) {
            Serial.println("[LoRa] AUX timeout");
            return false;
        }
        delay(1);
    }
    return true;
}

static void setMode(uint8_t mode) {
    switch (mode) {
        case LORA_MODE_NORMAL:
            digitalWrite(LORA_M0, LOW);
            digitalWrite(LORA_M1, LOW);
            break;

        case LORA_MODE_WAKEUP:
            digitalWrite(LORA_M0, HIGH);
            digitalWrite(LORA_M1, LOW);
            break;

        case LORA_MODE_POWER:
            digitalWrite(LORA_M0, LOW);
            digitalWrite(LORA_M1, HIGH);
            break;

        case LORA_MODE_SLEEP:
            digitalWrite(LORA_M0, HIGH);
            digitalWrite(LORA_M1, HIGH);
            break;
        
        default:
            return;
    }

    delay(50);
    waitAux();
}


// ============================================================
// Packet TX
// ============================================================


static uint16_t counter = 0;

static void sendPacket(MeshPacket& pkt) {
    pkt.packet_id = counter++;
    pkt.crc = crc16(reinterpret_cast<uint8_t*>(&pkt),
                     sizeof(MeshPacket) - sizeof(uint16_t));

    if (!waitAux()) {
        Serial.println("[LoRa] Not ready (before TX)");
        return;
    }

    loraSerial.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(MeshPacket));
    loraSerial.flush();

    if (!waitAux()) {
        Serial.println("[LoRa] TX not completed");
        return;
    }

    Serial.printf("[LoRa] TX OK (id=%d, type=%d, dst=%d)\n",
                  pkt.packet_id, pkt.type, pkt.destination);
}

// ============================================================
// Init
// ============================================================

void loraInit() {

    pinMode(LORA_AUX, INPUT);
    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);

    setMode(LORA_MODE_NORMAL);

    loraSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);

    delay(100);
    waitAux();

    Serial.println("LoRa init done");
}

// ============================================================
// Send data (from node to base)
// ============================================================

void sendData(float t, float h, float p, uint8_t relay, uint8_t motor) {
#ifndef ROLE_BASE
    MeshPacket pkt{};
    pkt.version = 1;
    pkt.type    = PKT_DATA;
    pkt.source  = NODE_ID;
    pkt.destination = BASE_ID;

    DataPayload payload;
    payload.t     = t;
    payload.h     = h;
    payload.p     = p;
    payload.relay = relay;
    payload.motor = motor;

    memcpy(pkt.payload, &payload, sizeof(payload));
    pkt.payload_size = sizeof(payload);

    sendPacket(pkt);
#endif
}

// ============================================================
// Send command (from base to node)
// ============================================================


void sendCommand(uint16_t node, uint8_t cmd, uint8_t value) {
#ifdef ROLE_BASE
    MeshPacket pkt{};
    pkt.version     = 1;
    pkt.type        = PKT_CMD;
    pkt.source      = BASE_ID;
    pkt.destination = node;

    pkt.payload[0] = cmd;
    pkt.payload[1] = value;
    pkt.payload_size = 2;

    sendPacket(pkt);
#endif
}

// ============================================================
// RX loop with packet boundary detection
// ============================================================

// Fixed-size receive buffer
static uint8_t rxBuffer[sizeof(MeshPacket)];
static uint16_t rxIndex = 0;
static unsigned long rxStartTime = 0;
static constexpr uint32_t RX_TIMEOUT_MS = 500; // timeout to reset stale partial packet

void loraLoop() {
    while (loraSerial.available()) {

        // Reset buffer if we started receiving but timed out
        if (rxIndex > 0 && (millis() - rxStartTime > RX_TIMEOUT_MS)) {
            Serial.println("[LoRa] RX timeout - resetting buffer");
            rxIndex = 0;
        }

        rxBuffer[rxIndex++] = loraSerial.read();

        if (rxIndex == 1) {
            rxStartTime = millis();
        }

        if (rxIndex >= sizeof(MeshPacket)) {
            rxIndex = 0; // always reset after consuming a full frame

            MeshPacket pkt;
            memcpy(&pkt, rxBuffer, sizeof(pkt));

            // Verify CRC
            uint16_t computed = crc16(
                reinterpret_cast<uint8_t*>(&pkt),
                sizeof(MeshPacket) - sizeof(uint16_t));

            if (computed != pkt.crc) {
                Serial.printf("[LoRa] CRC error: expected 0x%04X got 0x%04X\n",
                              pkt.crc, computed);
                continue;
            }

            // Validate version
            if (pkt.version != 1) {
                Serial.printf("[LoRa] Unknown version: %d\n", pkt.version);
                continue;
            }

            // ---- BASE: receive sensor data from nodes ----
#ifdef ROLE_BASE
            if (pkt.type == PKT_DATA) {
                Serial.printf("[LoRa] RX data from node %d\n", pkt.source);

                // Send discovery if this is a new node (safe: uses map, not fixed array)
                static bool discovered[256] = {false};

                if (pkt.source < 256 && !discovered[pkt.source]) {
                    publishDiscovery(pkt.source);
                    discovered[pkt.source] = true;
                } else if (pkt.source >= 256) {
                    Serial.printf("[LoRa] Node ID %d exceeds discovery array\n",
                                  pkt.source);
                }

                if (pkt.payload_size >= sizeof(DataPayload)) {
                    DataPayload data;
                    memcpy(&data, pkt.payload, sizeof(data));

                    char topic[64], buf[32];

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/temperature", pkt.source);
                    dtostrf(data.t, 1, 2, buf);
                    mqttPublish(topic, buf, true);

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/humidity", pkt.source);
                    dtostrf(data.h, 1, 2, buf);
                    mqttPublish(topic, buf, true);

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/pressure", pkt.source);
                    dtostrf(data.p, 1, 2, buf);
                    mqttPublish(topic, buf, true);

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/relay", pkt.source);
                    snprintf(buf, sizeof(buf), "%d", data.relay);
                    mqttPublish(topic, buf, true);

                    snprintf(topic, sizeof(topic),
                             "lora/node_%d/motor", pkt.source);
                    snprintf(buf, sizeof(buf), "%d", data.motor);
                    mqttPublish(topic, buf, true);
                } else {
                    Serial.printf("[LoRa] Payload too small: %d bytes\n",
                                  pkt.payload_size);
                }
            }

#else
            // ---- NODE: receive commands from base ----
            if (pkt.type == PKT_CMD && pkt.destination == NODE_ID) {
                if (pkt.payload[0] == 1 && pkt.payload_size >= 2) {
                    uint8_t value = pkt.payload[1];

                    if (value == 1) {
                        digitalWrite(RELAY_PIN, RELAY_ON);
                        Serial.println("[NODE] Relay ON");
                    } else {
                        digitalWrite(RELAY_PIN, RELAY_OFF);
                        Serial.println("[NODE] Relay OFF");
                    }

                    // Read actual sensor data and send back immediately
                    float t = 0, h = 0, p = 0;
                    uint8_t motor_state = 0;

                    if (sensorAvailable()) {
                        t = bme.readTemperature();
                        h = bme.readHumidity();
                        p = bme.readPressure() / 100.0F;
                    }

                    motor_state = (analogRead(MOTOR_PIN) > MOTOR_THRESHOLD) ? 1 : 0;
                    uint8_t relay_state = (digitalRead(RELAY_PIN) == RELAY_ON) ? 1 : 0;

                    sendData(t, h, p, relay_state, motor_state);
                }
            }

#endif
        }
    }
}
