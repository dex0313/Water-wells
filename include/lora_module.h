#pragma once
// ============================================================
// lora_module.h — EBYTE radio module selection (FR-1)
// ============================================================
// The air interface runs over EBYTE UART modules in transparent
// mode. Two models are supported; select exactly one from
// platformio.ini:
//   -D LORA_MODULE_E22   (default) E22-400T33D — base / repeater /
//                                  wells with a direct link (current
//                                  hardware)
//   -D LORA_MODULE_E220           E220-400T22D — wells behind a
//                                  repeater (planned)
//
// Both modules share the same 4-mode pinout (M0/M1/AUX), the same
// mode-switch timings, the same transparent framing and the same
// fixed-9600 configuration UART, so the runtime PHY (lora_phy.cpp)
// is identical for both. What differs is the register map and the
// RF-profile byte encoding — they matter only when the optional
// firmware-side module programming is enabled (LORA_MODULE_AUTOCONFIG,
// see below); otherwise the module is configured once with the EBYTE
// configuration tool, exactly as it is done today.
// ============================================================

// Exactly one module (same pattern as roles.h for ROLE_*).
#if !defined(LORA_MODULE_E22) && !defined(LORA_MODULE_E220)
#define LORA_MODULE_E22            // current hardware is the default
#endif

#if (defined(LORA_MODULE_E22) + defined(LORA_MODULE_E220)) != 1
#error "Select exactly one LoRa module: LORA_MODULE_E22 or LORA_MODULE_E220"
#endif

#if defined(LORA_MODULE_E22)

#define LORA_MODULE_NAME "E22-400T33D"

// E22 register map (configuration mode 3, frames start at reg 0x00):
// ADDH, ADDL, REG0, REG1, REG2, REG3, CRYPT_H, CRYPT_L — no NETID.
#define LORA_CFG_REG0_ADDR    0x02   // UART speed + air data rate
#define LORA_CFG_REG1_ADDR    0x03   // packet length + transmit power
#define LORA_CFG_REG2_ADDR    0x04   // channel
#define LORA_CFG_REG3_ADDR    0x05   // RSSI byte / transmission mode / LBT
#define LORA_CFG_CRYPT_H_ADDR 0x06
#define LORA_CFG_CRYPT_L_ADDR 0x07
#define LORA_CFG_FRAME_LEN    8
#define LORA_CFG_HAS_NETID    0

#else // LORA_MODULE_E220

#define LORA_MODULE_NAME "E220-400T22D"

// E220 register map: NETID sits at 0x02 and shifts every following
// register up by one; the config frame is one byte longer.
#define LORA_CFG_REG_NETID_ADDR 0x02
#define LORA_CFG_REG0_ADDR    0x03   // UART speed + air data rate
#define LORA_CFG_REG1_ADDR    0x04   // packet length + transmit power
#define LORA_CFG_REG2_ADDR    0x05   // channel
#define LORA_CFG_REG3_ADDR    0x06   // RSSI byte / transmission mode / LBT
#define LORA_CFG_CRYPT_H_ADDR 0x07
#define LORA_CFG_CRYPT_L_ADDR 0x08
#define LORA_CFG_FRAME_LEN    9
#define LORA_CFG_HAS_NETID    1

#endif // module selection

// ------------------------------------------------------------
// Transparent-mode UART speed of the owner-configured profile.
// The deployed modules run 9600 8N1; if a future E220 well is
// configured at a different baud, override LORA_UART_BAUD for that
// environment instead of editing lora_phy.cpp.
// (The configuration mode UART is fixed 9600 on both modules and
// does not depend on this value.)
// ------------------------------------------------------------
#ifndef LORA_UART_BAUD
#define LORA_UART_BAUD 9600
#endif

// ------------------------------------------------------------
// Optional firmware-side module programming (FR-1), default OFF.
//
// LORA_MODULE_AUTOCONFIG=1 writes the RF profile into the module at
// every boot (mode 3, permanent 0xC0 write + 0xC1 read-back verify)
// so a fresh module dropped into the field works without the EBYTE
// configuration tool. Default 0 keeps the exact current behavior:
// the firmware never touches the radio registers.
//
// TODO(owner): BEFORE enabling, fill the LORA_CFG_*_VALUE bytes
// below from the profile actually programmed into the deployed
// fleet (channel, air data rate, power) and verify every byte
// against the datasheet of the SELECTED module — the register MAPS
// above are implemented per datasheet, but the byte VALUES are
// placeholders. Then add -D LORA_CFG_PROFILE_VERIFIED to the env
// build_flags to silence the build warning.
// ------------------------------------------------------------
#ifndef LORA_MODULE_AUTOCONFIG
#define LORA_MODULE_AUTOCONFIG 0
#endif

#if LORA_MODULE_AUTOCONFIG != 0 && LORA_MODULE_AUTOCONFIG != 1
#error "LORA_MODULE_AUTOCONFIG must be 0 or 1"
#endif

#if LORA_MODULE_AUTOCONFIG

#if !defined(LORA_CFG_PROFILE_VERIFIED)
#warning "LORA_MODULE_AUTOCONFIG=1 with placeholder LORA_CFG_*_VALUE bytes — verify them against the module datasheet, then define LORA_CFG_PROFILE_VERIFIED"
#endif

// TODO(owner): RF profile bytes (values below are placeholders!).
#define LORA_CFG_REG0_VALUE   0x00   // UART 9600 8N1 + air data rate
#define LORA_CFG_REG1_VALUE   0x00   // packet length + power
#define LORA_CFG_REG2_VALUE   0x00   // channel
#define LORA_CFG_REG3_VALUE   0x00   // transparent mode, RSSI byte OFF,
                                     // LBT OFF (the frame parser expects
                                     // raw MeshPacket bytes)
#define LORA_CFG_NETID_VALUE  0x00   // E220 only
#define LORA_CFG_CRYPT_VALUE  0x0000 // encryption OFF — non-zero crypto
                                     // would corrupt the MeshPacket CRC

#endif // LORA_MODULE_AUTOCONFIG
