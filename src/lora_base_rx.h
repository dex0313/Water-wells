#pragma once
// INTERNAL header — included only by lora_rx.cpp / lora_base.cpp
#include "protocol.h"

// PKT_DATA / PKT_ACK / PKT_STATUS handler (lora_base.cpp)
void loraBaseHandlePacket(const MeshPacket& pkt);

// Reset nodeInfo/pending/discovery state (called from loraInit)
void loraBaseInit();
