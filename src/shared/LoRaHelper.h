#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include "Common.h"

bool initLoRa();
void sendMessage(const String &msg);
String receiveMessage();
