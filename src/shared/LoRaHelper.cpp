#include "LoRaHelper.h"

bool initLoRa() {
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("❌ Ошибка инициализации LoRa");
    return false;
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.enableCrc();
  Serial.println("✅ LoRa инициализирован успешно");
  return true;
}

void sendMessage(const String &msg) {
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
}

String receiveMessage() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) msg += (char)LoRa.read();
    return msg;
  }
  return "";
}

void printHeader(const char* role, const char* mode) {
  Serial.println("=================================");
  Serial.printf("Device Role: %s\n", role);
  Serial.printf("Build Mode : %s\n", mode);
  Serial.println("=================================");
}
