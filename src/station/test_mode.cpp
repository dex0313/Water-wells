#include <Arduino.h>
#include "../shared/LoRaHelper.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);
  printHeader("STATION", "TEST");

  if (!initLoRa()) {
    Serial.println("LoRa init failed!");
    while (1);
  }
}

void loop() {
  String msg = receiveMessage();
  if (msg.length()) {
    Serial.printf("📥 Получено: %s\n", msg.c_str());
    sendMessage("ACK: " + msg);
    Serial.println("📤 Отправлен ACK");
  }
}
