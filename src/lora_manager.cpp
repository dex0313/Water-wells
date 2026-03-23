#include <HardwareSerial.h>
#include <Arduino.h>
#include "lora_manager.h"
#include "mqtt_manager.h"
#include "config.h"

HardwareSerial loraSerial(2);

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

#define PKT_DATA 1
#define PKT_CMD  2
#define PKT_ACK  3

#define LORA_RX 16
#define LORA_TX 17
#define LORA_AUX 4
#define LORA_M0 18
#define LORA_M1 5

#define MODE_NORMAL 0
#define MODE_SLEEP  1
#define MODE_WAKEUP 2
#define MODE_POWER  3

static uint16_t counter = 0;

uint16_t crc16(uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

bool waitAux(uint32_t timeout = 1000) {

    uint32_t start = millis();

    while (digitalRead(LORA_AUX) == LOW) {
        if (millis() - start > timeout) {
            Serial.println("AUX timeout");
            return false;
        }
        delay(1);
    }

    return true;
}

void setMode(uint8_t mode) {

    switch (mode) {
        case MODE_NORMAL:
            digitalWrite(LORA_M0, LOW);
            digitalWrite(LORA_M1, LOW);
            break;

        case MODE_WAKEUP:
            digitalWrite(LORA_M0, HIGH);
            digitalWrite(LORA_M1, LOW);
            break;

        case MODE_POWER:
            digitalWrite(LORA_M0, LOW);
            digitalWrite(LORA_M1, HIGH);
            break;

        case MODE_SLEEP:
            digitalWrite(LORA_M0, HIGH);
            digitalWrite(LORA_M1, HIGH);
            break;
    }

    delay(50);
    waitAux();
}

void sendPacket(MeshPacket &pkt) {

    pkt.packet_id = counter++;
    pkt.crc = crc16((uint8_t*)&pkt, sizeof(pkt) - 2);

    if (!waitAux()) {
        Serial.println("LoRa not ready (before TX)");
        return;
    }

    loraSerial.write((uint8_t*)&pkt, sizeof(pkt));
    loraSerial.flush();

    if (!waitAux()) {
        Serial.println("LoRa TX not completed");
        return;
    }

    Serial.println("LoRa TX OK");
}

void loraInit() {

    pinMode(LORA_AUX, INPUT);
    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);

    setMode(MODE_NORMAL);

    loraSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);

    delay(100);
    waitAux();

    Serial.println("LoRa init done");
}

void sendData(float t, float h, float p, uint8_t relay, uint8_t motor) {

#ifndef ROLE_BASE

    MeshPacket pkt{};
    pkt.version = 1;
    pkt.type = PKT_DATA;
    pkt.source = NODE_ID;
    pkt.destination = BASE_ID;

    DataPayload payload = {t, h, p, relay, motor};

    memcpy(pkt.payload, &payload, sizeof(payload));
    pkt.payload_size = sizeof(payload);

    sendPacket(pkt);

#endif
}

void sendCommand(uint16_t node, uint8_t cmd, uint8_t value) {

#ifdef ROLE_BASE

    MeshPacket pkt{};
    pkt.version = 1;
    pkt.type = PKT_CMD;
    pkt.source = BASE_ID;
    pkt.destination = node;

    pkt.payload[0] = cmd;
    pkt.payload[1] = value;
    pkt.payload_size = 2;

    sendPacket(pkt);

#endif
}

void loraLoop() {

    static uint8_t buffer[sizeof(MeshPacket)];
    static uint16_t index = 0;

    while (loraSerial.available()) {

        buffer[index++] = loraSerial.read();

        if (index >= sizeof(MeshPacket)) {

            MeshPacket pkt;
            memcpy(&pkt, buffer, sizeof(pkt));
            index = 0;

            uint16_t crc = crc16((uint8_t*)&pkt, sizeof(pkt) - 2);
            if (crc != pkt.crc) {
                Serial.println("CRC error");
                return;
            }

#ifdef ROLE_BASE

            Serial.println("LoRa RX packet");

            if (pkt.type == PKT_DATA) {

                DataPayload data;
                memcpy(&data, pkt.payload, sizeof(data));

                char topic[64], buf[32];

                sprintf(topic, "lora/node_%d/temperature", pkt.source);
                dtostrf(data.t, 1, 2, buf);
                mqttPublish(topic, buf, true);

                sprintf(topic, "lora/node_%d/humidity", pkt.source);
                dtostrf(data.h, 1, 2, buf);
                mqttPublish(topic, buf, true);

                sprintf(topic, "lora/node_%d/pressure", pkt.source);
                dtostrf(data.p, 1, 2, buf);
                mqttPublish(topic, buf, true);

                sprintf(topic, "lora/node_%d/relay", pkt.source);
                sprintf(buf, "%d", data.relay);
                mqttPublish(topic, buf, true);

                sprintf(topic, "lora/node_%d/motor", pkt.source);
                sprintf(buf, "%d", data.motor);
                mqttPublish(topic, buf, true);
            }

#else

            if (pkt.type == PKT_CMD && pkt.destination == NODE_ID) {

                if (pkt.payload[0] == 1) {
                    digitalWrite(25, pkt.payload[1]);
                    Serial.println("Relay command received");
                }
            }

#endif
        }
    }
}