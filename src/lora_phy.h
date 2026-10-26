#pragma once
// ============================================================
// lora_phy.h — INTERNAL header: UART/AUX/TX-queue layer
// ============================================================
// Not installed with the public API — included only by the LoRa
// role modules (lora_rx / lora_base / lora_node / lora_repeater).
// ============================================================

#include <Arduino.h>
#include "config.h"      // self-contained: LORA_AUX_TIMEOUT_MS etc.
#include "protocol.h"

// ------------------------------------------------------------
// TX priorities (R3): lower value = transmitted sooner.
// ACKs and commands jump ahead of telemetry and forwarded frames.
// ------------------------------------------------------------
enum TxPrio : uint8_t {
    TXP_ACK = 0,
    TXP_CMD = 1,
    TXP_FWD = 2,
    TXP_DATA = 3
};

// Shared LoRa statistics (P1-3), published by BASE every 5 min.
struct LoraStats {
    uint32_t rx_ok;
    uint32_t rx_crc_err;
    uint32_t resync_bytes;
    uint32_t rx_dup;
    uint32_t tx_ok;
    uint32_t tx_drop;
    uint32_t fwd;
    uint32_t fwd_drop;
};

extern LoraStats gLoraStats;
extern HardwareSerial loraSerial;
extern uint32_t gLastRxMs;   // millis() of the last received byte

// ------------------------------------------------------------
// PHY helpers
// ------------------------------------------------------------

bool loraWaitAux(uint32_t timeout_ms = LORA_AUX_TIMEOUT_MS);
void loraSetMode(uint8_t mode);

// Initialize pins/modes/UART (RX buffer 1024, P1-2) and the TX
// queue with a random packet_id start (R1). Blocking calls are
// allowed here (init only).
void loraPhyInit();

// Set the packet_id counter start value (R1: random after boot).
void txResetCounter(uint16_t start);

// Called for every received byte: arms the half-duplex TX gap.
void loraNoteRx();

// ------------------------------------------------------------
// TX queue (R3) — non-blocking, never calls delay()
// ------------------------------------------------------------

// Queue a frame for transmission. With assign_new_id=true the packet
// gets a fresh packet_id (own packets); with false it is forwarded
// byte-for-byte (repeater). delay_ms defers the transmission start.
// Returns false when the packet was dropped (queue overflow with no
// lower-priority victim).
bool txEnqueue(const MeshPacket& pkt, TxPrio prio, uint32_t delay_ms, bool assign_new_id);

// Drive the TX state machine. Call on every loop iteration.
void txPoll();

// Peek/clear aggregated TX counters (used by lora stats publishing).
void loraTxCounters(uint32_t& tx_ok, uint32_t& tx_drop);

// ------------------------------------------------------------
// Shared payload builders (used by several role modules)
// ------------------------------------------------------------

// Build and queue a DATA packet (node / repeater heartbeat).
void sendData(float t, float h, float p,
              uint8_t num_relays, uint8_t num_sensors,
              uint8_t relay_states, uint8_t motor_states,
              const float voltages[]);

// Build and queue an ACK packet (node reply / repeater NAK).
void sendAck(uint16_t destination, uint8_t cmd,
             uint8_t relay_index, uint8_t result, uint8_t state,
             uint8_t seq);
