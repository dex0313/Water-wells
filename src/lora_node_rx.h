#pragma once
// INTERNAL header — included only by lora_rx.cpp / lora_node.cpp
#include "protocol.h"

// PKT_CMD handler (lora_node.cpp)
void loraNodeHandlePacket(const MeshPacket& pkt);
