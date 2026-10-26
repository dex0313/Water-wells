#pragma once
// INTERNAL header — included only by lora_rx.cpp / lora_repeater.cpp
#include "protocol.h"

// Validated-frame handler for the repeater role (lora_repeater.cpp).
// Deduplication is applied centrally in lora_rx.cpp for all roles.
void loraRepeaterHandlePacket(const MeshPacket& pkt);
