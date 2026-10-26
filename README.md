# Water-wells

ESP32 + EBYTE E22/E220 (LoRa, transparent mode) + PlatformIO / Arduino.
Mesh of well nodes (relays + optional ZMPT101B mains sensors + optional
BME280) reporting over LoRa to a BASE gateway that bridges everything
into **Home Assistant** via MQTT. A dedicated **repeater** build
extends coverage.

## Environments

| Environment   | Role          | Hardware                             |
|---------------|---------------|--------------------------------------|
| `pio run -e base`     | `ROLE_BASE`     | ESP32 + E22 + WiFi/MQTT (+ BME280)   |
| `pio run -e node`     | `ROLE_NODE`     | ESP32 + E22 + relays + ZMPT + BME280 |
| `pio run -e repeater` | `ROLE_REPEATER` | ESP32 + E22 (+ optional BME280)      |
| `pio run -e node_e220` | `ROLE_NODE`    | ESP32 + **E220** + relays + sensors  |
| `pio run -e node_relay_only` | `ROLE_NODE` | ESP32 + E22 + **relays only**     |
| `pio test -e native`  | host unit tests | pure-logic modules (T-1..T-6)        |

Toolchain and libraries are pinned (P1-9); see `platformio.ini`.
The code also compiles against the Arduino core 3.x line — the
reproduction config is in the comment at the bottom of
`platformio.ini`.

## Radio module selection (FR-1)

Two EBYTE modules are supported; exactly one is selected per build:

- `LORA_MODULE_E22` (default) — E22-400T33D, current hardware on
  base / repeater / directly linked wells;
- `LORA_MODULE_E220` — E220-400T22D, planned for wells behind a
  repeater (see `[env:node_e220]`).

Both share the same pinout (M0/M1/AUX/UART), mode switching and
transparent framing, so the runtime PHY is identical — see
`include/lora_module.h` for the register maps. The transparent UART
speed is `LORA_UART_BAUD` (9600).

Optionally the firmware can program the module's RF profile itself
(`LORA_MODULE_AUTOCONFIG=1` in build_flags): it writes the profile in
sleep mode (permanent `0xC0` write) and verifies it with a read-back
at every boot; a failed verify is non-fatal and only logged. Before
enabling, fill the `LORA_CFG_*_VALUE` bytes in `lora_module.h` from
the EBYTE tool profile and verify them against the datasheet, then
add `-D LORA_CFG_PROFILE_VERIFIED` — until then the build warns and
the bytes are placeholders.

## Relay-only wells (FR-2)

Wells that only switch a pump and have no sensors are built with
`-D FEATURE_SENSORS=0` (see `[env:node_relay_only]`): all local
sensor code (BME280 + ZMPT101B) is compiled out. Relays stay fully
functional — NVS restore, commands, ACKs and the deferred report
(P1-6) are untouched. Telemetry reports carry `t/h/p = NaN` and
`num_sensors = 0`, so BASE creates no temperature/humidity/pressure/
voltage/motor entities for such a node, and the
`HA_DISCOVERY_CLEANUP` pass removes stale entities if the profile
shrinks. The flag is role-independent: it also removes local BME
publishing on BASE and the optional BME probe on REPEATER.

## Secrets (P2-4)

1. Copy `include/secret.h.example` to `include/secret.h`.
2. Fill in WiFi / MQTT credentials.
3. `secret.h` is git-ignored; only `wifi_manager.cpp` and
   `mqtt_manager.cpp` include it, and only in the BASE build, so
   node/repeater firmware needs no secrets.

## Home Assistant notes

- **Switch retain (P0-4):** HA switch entities must NOT have
  `retain: true` for the command topic. A retained command would be
  re-delivered by the broker every time BASE reconnects and re-run an
  old command on the pump.
- Discovery is republished automatically after BASE reconnects and
  when HA announces `homeassistant/status: online` (P1-1).
- Node entities carry `availability` (`lora/base/status` + node
  `online`), so stale telemetry is hidden when a node or the base is
  offline.
- `cmd_status` is published **non-retained** with `pending` at send
  time and the final `ok` / `failed:*` after the ACK.

## Wire format / repeater design

- Wire format v1 is frozen: 59-byte frames, CRC-16/Modbus, verified
  by host tests (T-1 golden packet). Extensions are additive only:
  `seq` byte in CMD/ACK payloads (P0-3) and the `PKT_STATUS=4`
  diagnostics packet (R8), which old firmware silently ignores.
- **packet_id starts from a random value after every boot (R1).**
  Combined with the repeater dedup TTL of 45 s this makes stale-cache
  drops of freshly rebooted nodes impossible for new firmware, while
  for old nodes (counter from 0) the TTL alone is sufficient: a key
  with `id=0` was inserted at least 3 minutes ago and already expired.
  Do NOT increase `DEDUP_TTL_MS` without revisiting this reasoning.
- Every role deduplicates by `(source, packet_id)` (R2/R4) because
  each frame arrives twice (directly + via the repeater). Retried
  commands always carry a NEW `packet_id`, so retries are not
  swallowed; correlation is via `seq`.
- Repeater addresses use the ID range **200..250** (R5); well nodes
  are 1..199. BASE is 0.
- Transmissions are queued non-blocking with priorities
  ACK > CMD > FWD > DATA (R3); the repeater forwards frames
  byte-for-byte after a random jitter and is rate-limited to
  `FWD_MAX_PER_MIN` (R7).

## Rollout order

Deploy to hardware in this order:

1. **BASE first** — the new BASE must work correctly with old nodes.
2. Then the **repeater**.
3. Then the **nodes**.

## Bench checklist (after flashing)

1. BME280 disconnected: relays still switch, voltages and `online`
   still arrive in HA.
2. Reboot a node with a relay ON: no OFF pulse on the relay output.
3. Quick ON, OFF commands: final state OFF, `cmd_status=ok` appears
   only for the ACK of the last command.
4. Publish `ON`, `2`, empty string, `cmd_0`, `cmd_9`, `node_0`:
   nothing goes on air, `cmd_status=failed:bad_request` is published.
5. Stop the broker at the first packet of a new node, then restart:
   entities appear after reconnection; after an HA restart entities
   return without rebooting BASE.
6. Feed UART garbage + valid frames: frames are received,
   `lora/base/stats/resync_bytes` grows.
7. Three quick node reboots next to a repeater: packets are not lost.
8. Power two nodes simultaneously: transmissions are spread in time.
9. A node out of BASE range but in repeater range: full CMD→ACK cycle
   fits into `ACK_TIMEOUT_MS`; BASE publishes each value once
   (dedup); repeater counters grow.
10. WiFi router off for 10 minutes: BASE does not churn the
    connection; after the router returns, MQTT and discovery recover.
11. `node_relay_only` (FR-2) on a relay-only well: boot log shows
    "Relay-only build", relays switch from HA, and no temperature/
    humidity/pressure/voltage/motor entities appear for the node.

## LoRa stats diagnostics

BASE publishes `lora/base/stats/*` every 5 minutes (P1-3):
`rx_ok`, `rx_crc_err`, `resync_bytes`, `rx_dup`, `tx_ok`, `tx_drop`,
`dedup_evicted`. Repeaters send `PKT_STATUS` (R8) and BASE exposes
`lora/node_<id>/stats/*` as diagnostic HA entities.
