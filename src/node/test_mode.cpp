#include <Arduino.h>
#include "../shared/LoRaHelper.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);
  printHeader("NODE", "TEST");

  if (!initLoRa()) {
    Serial.println("LoRa init failed!");
    while (1);
  }
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > SEND_INTERVAL) {
    String msg = "NODE TEST " + String(millis() / 1000);
    sendMessage(msg);
    Serial.printf("📤 Отправлено: %s\n", msg.c_str());
    last = millis();
  }

  String incoming = receiveMessage();
  if (incoming.length()) {
    Serial.printf("📥 Получено от станции: %s\n", incoming.c_str());
  }
}
