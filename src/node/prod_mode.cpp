#include <Arduino.h>
#include "../shared/LoRaHelper.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);
  printHeader("NODE", "PROD");

  if (!initLoRa()) {
    Serial.println("LoRa init failed!");
    while (1);
  }
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > SEND_INTERVAL) {
    // Здесь можно добавить чтение температуры, влажности и т. д.
    float temp = 25.0 + random(-5, 5) * 0.1;
    String msg = "TEMP:" + String(temp, 1);
    sendMessage(msg);
    Serial.printf("📤 Отправлено: %s\n", msg.c_str());
    last = millis();
  }
}
