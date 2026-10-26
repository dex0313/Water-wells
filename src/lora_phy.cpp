// ============================================================
// lora_phy.cpp — EBYTE E22/E220 transparent-mode PHY + TX queue (R3)
// ============================================================
// Whole-file role guard (P1-8): compiled for every role.
// The module model is selected in lora_module.h (FR-1): both E22
// and E220 share this runtime path (same pins/modes/UART framing);
// they differ only in the register map used by the optional
// firmware-side configuration write (LORA_MODULE_AUTOCONFIG).
// ============================================================

#include <Arduino.h>
#include "lora_phy.h"
#include "lora_module.h"
#include "config.h"
#include "frame_parser.h"
#include <esp_system.h>

HardwareSerial loraSerial(2);

LoraStats gLoraStats = {};
uint32_t  gLastRxMs = 0;

// ============================================================
// AUX / mode helpers (blocking use allowed only during init)
// ============================================================

bool loraWaitAux(uint32_t timeout_ms) {
    uint32_t start = millis();
    while (digitalRead(LORA_AUX) == LOW) {
        if (millis() - start > timeout_ms) {
            Serial.println("[LoRa] AUX timeout");
            return false;
        }
        delay(1);
    }
    return true;
}

void loraSetMode(uint8_t mode) {
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
    loraWaitAux();
}

// ============================================================
// Optional firmware-side module programming (FR-1)
// ============================================================
#if LORA_MODULE_AUTOCONFIG

// Read one byte from the config UART with a timeout, -1 on timeout.
static int loraCfgReadByte(uint32_t timeout_ms) {
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < timeout_ms) {
        if (loraSerial.available() > 0) return loraSerial.read();
        delay(1);
    }
    return -1;
}

// Program the RF profile in sleep mode (mode 3) and verify it with a
// read-back. On both E22 and E220 the configuration UART is fixed at
// 9600 8N1; frames are [0xC0|0xC1][start][len][data...], where 0xC0
// writes permanently and 0xC1 reads back. Failure is NOT fatal
// (P0-1 style): the module keeps the profile last written by the
// EBYTE configuration tool and the boot log explains what happened.
static void loraModuleApplyConfig() {
    Serial.printf("[LoRa] Programming %s RF profile...\n", LORA_MODULE_NAME);

    loraSetMode(LORA_MODE_SLEEP);
    loraSerial.end();
    loraSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);  // fixed config baud

    uint8_t frame[3 + LORA_CFG_FRAME_LEN];
    frame[0] = 0xC0;                       // permanent write
    frame[1] = 0x00;                       // start register (ADDH)
    frame[2] = LORA_CFG_FRAME_LEN;
    frame[3] = 0x00;                       // ADDH (module-level addressing unused:
    frame[4] = 0x00;                       //  the mesh carries dst/src in-band)
    uint8_t n = 5;
#if LORA_CFG_HAS_NETID
    frame[n++] = LORA_CFG_NETID_VALUE;
#endif
    frame[n++] = LORA_CFG_REG0_VALUE;
    frame[n++] = LORA_CFG_REG1_VALUE;
    frame[n++] = LORA_CFG_REG2_VALUE;
    frame[n++] = LORA_CFG_REG3_VALUE;
    frame[n++] = (uint8_t)(LORA_CFG_CRYPT_VALUE >> 8);
    frame[n++] = (uint8_t)(LORA_CFG_CRYPT_VALUE & 0xFF);

    // Drain anything pending, then write and give the module time to
    // persist the parameters (the write echo, if any, is ignored —
    // verification below is authoritative).
    loraWaitAux();
    while (loraSerial.available() > 0) loraSerial.read();
    loraSerial.write(frame, n);
    loraSerial.flush();
    delay(50);
    while (loraSerial.available() > 0) loraSerial.read();

    // Read-back verify: module answers [start][FRAME_LEN data bytes].
    bool ok = false;
    const uint8_t readCmd[3] = { 0xC1, 0x00, LORA_CFG_FRAME_LEN };
    loraWaitAux();
    loraSerial.write(readCmd, sizeof(readCmd));
    loraSerial.flush();

    uint8_t rb[1 + LORA_CFG_FRAME_LEN];
    for (uint8_t i = 0; i < sizeof(rb); i++) {
        int b = loraCfgReadByte(500);
        if (b < 0) break;
        rb[i] = (uint8_t)b;
        if (i == sizeof(rb) - 1) {
            ok = (rb[0] == 0x00) &&
                 memcmp(&rb[1], &frame[3], LORA_CFG_FRAME_LEN) == 0;
        }
    }

    if (ok) {
        Serial.println("[LoRa] RF profile written and verified");
    } else {
        Serial.println("[LoRa] WARNING: RF profile verify failed — "
                       "module keeps its previous profile");
    }

    loraSerial.end();
    loraSetMode(LORA_MODE_NORMAL);
}

#endif // LORA_MODULE_AUTOCONFIG

// ============================================================
// Init
// ============================================================

void loraPhyInit() {
    pinMode(LORA_AUX, INPUT);
    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);

    loraSetMode(LORA_MODE_NORMAL);

#if LORA_MODULE_AUTOCONFIG
    loraModuleApplyConfig();
#endif

    // P1-2: the default 256-byte RX buffer holds only ~270 ms of the
    // 9600-baud stream; blocking elsewhere (NVS, MQTT connect,
    // discovery bursts) used to overflow it. 1024 bytes ~ 1 s.
    loraSerial.setRxBufferSize(1024);
    loraSerial.begin(LORA_UART_BAUD, SERIAL_8N1, LORA_RX, LORA_TX);

    delay(100);
    loraWaitAux();

    Serial.printf("LoRa init done (%s)\n", LORA_MODULE_NAME);

    // R1: packet_id starts from a random value after every boot.
    // A node rebooting to 0 would collide with keys still cached by
    // the repeater dedup cache. Combined with the 45 s dedup TTL
    // this makes stale-cache drops of freshly rebooted nodes
    // impossible for new firmware and harmless for old nodes.
    txResetCounter((uint16_t)esp_random());
}

void loraNoteRx() {
    gLastRxMs = millis();
}

// ============================================================
// TX queue (R3) — non-blocking transmission state machine
// ============================================================

struct TxItem {
    MeshPacket pkt;
    TxPrio     prio;
    uint32_t   not_before;   // earliest transmission time (millis)
    bool       fwd;          // forwarded frame (for fwd_drop stats)
    bool       used;
};

#define TX_QUEUE_SIZE 8

static TxItem s_queue[TX_QUEUE_SIZE];

// In-flight packet
static MeshPacket s_inflight;
static bool       s_inflight_aux_low_seen = false;
static uint32_t   s_inflight_started_ms = 0;

enum TxState : uint8_t { TX_IDLE = 0, TX_WAIT_DONE };
static TxState s_tx_state = TX_IDLE;

static uint16_t s_packet_id_counter = 0;
static uint32_t s_tx_ok = 0;
static uint32_t s_tx_drop = 0;

void txResetCounter(uint16_t start) {
    s_packet_id_counter = start;
}

uint16_t loraNextPacketId() {
    return s_packet_id_counter++;
}

void loraTxCounters(uint32_t& tx_ok, uint32_t& tx_drop) {
    tx_ok = s_tx_ok;
    tx_drop = s_tx_drop;
}

static void fillQueueSlot(TxItem& slot, const MeshPacket& pkt, TxPrio prio,
                          uint32_t delay_ms, bool fwd) {
    slot.pkt = pkt;
    if (!fwd) {
        // Own packets always go out with a fresh packet_id.
        slot.pkt.packet_id = loraNextPacketId();
        slot.pkt.crc = crc16(reinterpret_cast<uint8_t*>(&slot.pkt),
                             sizeof(MeshPacket) - sizeof(uint16_t));
    }
    slot.prio = prio;
    slot.not_before = millis() + delay_ms;
    slot.fwd = fwd;
    slot.used = true;
}

bool txEnqueue(const MeshPacket& pkt, TxPrio prio, uint32_t delay_ms, bool assign_new_id) {
    // Find a free slot first.
    for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
        if (!s_queue[i].used) {
            fillQueueSlot(s_queue[i], pkt, prio, delay_ms, !assign_new_id);
            return true;
        }
    }

    // Queue full: evict the item with the LOWEST priority (highest
    // TxPrio value) if the newcomer outranks it, otherwise drop the
    // newcomer (R3).
    uint8_t victim = 0;
    for (uint8_t i = 1; i < TX_QUEUE_SIZE; i++) {
        if (s_queue[i].prio > s_queue[victim].prio) {
            victim = i;
        }
    }

    if (prio >= s_queue[victim].prio) {
        s_tx_drop++;
        gLoraStats.tx_drop = s_tx_drop;
        Serial.println("[LoRa] TX queue full — packet dropped");
        return false;
    }

    if (s_queue[victim].fwd) {
        gLoraStats.fwd_drop++;
    }
    s_tx_drop++;
    gLoraStats.tx_drop = s_tx_drop;
    fillQueueSlot(s_queue[victim], pkt, prio, delay_ms, !assign_new_id);
    Serial.println("[LoRa] TX queue full — lower-priority packet evicted");
    return true;
}

// Pick the ready item with the highest priority (lowest TxPrio value).
// Returns -1 when nothing is ready.
static int8_t pickReadyItem(uint32_t now) {
    int8_t best = -1;
    for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
        if (!s_queue[i].used) continue;
        if ((int32_t)(now - s_queue[i].not_before) < 0) continue;  // P1-5 wrap-safe compare
        if (best < 0 || s_queue[i].prio < s_queue[best].prio) {
            best = (int8_t)i;
        }
    }
    return best;
}

void txPoll() {
    uint32_t now = millis();

    switch (s_tx_state) {
        case TX_IDLE: {
            if (digitalRead(LORA_AUX) != HIGH) {
                return;  // module busy
            }
            // Half-duplex gap: do not step on a frame still arriving.
            if (gLastRxMs != 0 &&
                (uint32_t)(now - gLastRxMs) < LORA_TX_AFTER_RX_GAP_MS) {
                return;
            }

            int8_t idx = pickReadyItem(now);
            if (idx < 0) {
                return;
            }

            s_inflight = s_queue[idx].pkt;
            s_queue[idx].used = false;

            s_inflight_aux_low_seen = false;
            s_inflight_started_ms = now;

            loraSerial.write(reinterpret_cast<uint8_t*>(&s_inflight),
                             sizeof(MeshPacket));
            loraSerial.flush();
            s_tx_state = TX_WAIT_DONE;
            return;
        }

        case TX_WAIT_DONE: {
            // The E22 pulls AUX LOW while transmitting and raises it
            // when done. Ignore the ~5 ms right after the write where
            // the pin may still be HIGH before dropping.
            if (digitalRead(LORA_AUX) == LOW) {
                s_inflight_aux_low_seen = true;
            } else if (s_inflight_aux_low_seen) {
                // LOW -> HIGH transition: transmission finished.
                s_tx_ok++;
                gLoraStats.tx_ok = s_tx_ok;
                s_tx_state = TX_IDLE;
                return;
            }

            // Guard timeout: never wait forever for the AUX pulse.
            uint32_t guard = 2 * LORA_AIRTIME_MS + 500;
            if ((uint32_t)(now - s_inflight_started_ms) > guard) {
                Serial.println("[LoRa] TX completion timeout — packet may be lost");
                s_tx_drop++;
                gLoraStats.tx_drop = s_tx_drop;
                s_tx_state = TX_IDLE;
            }
            return;
        }
    }
}

// ============================================================
// Shared payload builders
// ============================================================

// DATA packet (node telemetry / repeater heartbeat).
// t/h/p may be NAN when the BME280 is unavailable (P0-2) — the
// float NaN travels inside the fixed-size payload and old BASE
// firmware simply published "nan"; new BASE filters it (P2-2).
void sendData(float t, float h, float p,
              uint8_t num_relays, uint8_t num_sensors,
              uint8_t relay_states, uint8_t motor_states,
              const float voltages[]) {
    MeshPacket pkt{};
    pkt.version = 1;
    pkt.type    = PKT_DATA;
    pkt.source  = NODE_ID;
    pkt.destination = BASE_ID;

    DataPayload payload;
    payload.t = t;
    payload.h = h;
    payload.p = p;
    payload.num_relays = num_relays;
    payload.num_sensors = num_sensors;
    payload.relay_states = relay_states;
    payload.motor_states = motor_states;

    // Fill voltage array (zero unused slots)
    for (uint8_t i = 0; i < MAX_SENSORS_PER_NODE; i++) {
        payload.voltage[i] = (i < num_sensors) ? voltages[i] : 0.0f;
    }

    memcpy(pkt.payload, &payload, sizeof(payload));
    pkt.payload_size = sizeof(payload);

    txEnqueue(pkt, TXP_DATA, 0, true);
}

// ACK packet (node reply / repeater NAK). seq echoes the command's
// sequence byte (P0-3); legacy commands pass seq = 0.
void sendAck(uint16_t destination, uint8_t cmd,
             uint8_t relay_index, uint8_t result, uint8_t state,
             uint8_t seq) {
    MeshPacket pkt{};
    pkt.version     = 1;
    pkt.type        = PKT_ACK;
    pkt.source      = NODE_ID;
    pkt.destination = destination;

    AckPayload ack;
    ack.cmd         = cmd;
    ack.relay_index = relay_index;
    ack.result      = result;
    ack.state       = state;
    ack.seq         = seq;

    memcpy(pkt.payload, &ack, sizeof(ack));
    pkt.payload_size = sizeof(ack);

    txEnqueue(pkt, TXP_ACK, 0, true);

    Serial.printf("[NODE] ACK sent: cmd=%d relay=%d result=%d state=%d seq=%d\n",
                  cmd, relay_index, result, state, seq);
}
