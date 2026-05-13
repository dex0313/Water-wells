#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "system_manager.h"
#include "lora_manager.h"
#include "config.h"
#include "persistence.h"


void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Water Wells System ===");

    // Initialize watchdog timer FIRST — protects the rest of setup
    wdtInit();

    // Increment boot counter (diagnostic — visible in serial log)
    nvsIncrementBootCount();

    loraInit();
    sensorInit();

#ifdef ROLE_BASE
    Serial.println("Role: BASE");
    wifiInit();
    mqttInit();
#else
    Serial.printf("Role: NODE (%d relays, %d sensors)\n",
                  NODE_NUM_RELAYS, NODE_NUM_SENSORS);
#endif
}

void loop() {

#ifdef ROLE_BASE
    baseLoop();
#else
    nodeLoop();
#endif
}