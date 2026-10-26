#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "system_manager.h"
#include "lora_manager.h"
#include "config.h"
#include "persistence.h"


void setup() {
    // Serial comes up first so boot diagnostics from relaysInit()/
    // nvsLoadRelayStates() ("[NVS] Relay states restored/loaded") are
    // actually printed: on the ESP32 Arduino core anything printed
    // before begin() is silently discarded. Serial.begin() only
    // configures the UART pads (GPIO1/3) — it cannot touch the relay
    // output latches, so the P0-6 guarantee below still holds: the
    // latch preload remains the first operation on the relay pins.
    Serial.begin(115200);
    delay(10);   // let the UART settle before the first prints

#if defined(ROLE_NODE)
    // P0-6: relays come up FIRST — the NVS-saved level is preloaded
    // into the output latch before the output driver is enabled, so
    // a running pump never sees an OFF pulse while this firmware
    // boots (LoRa/WiFi init take up to several seconds).
    relaysInit();
#endif

    delay(500);
    Serial.println("=== Water Wells System ===");

    // Initialize watchdog timer FIRST — protects the rest of setup
    wdtInit();

    // Increment boot counter (diagnostic — visible in serial log,
    // sent in PKT_STATUS by repeaters, R8.2)
    nvsIncrementBootCount();

    loraInit();
    sensorInit();

#if defined(ROLE_BASE)
    Serial.println("Role: BASE");
    wifiInit();
    mqttInit();
#elif defined(ROLE_REPEATER)
    Serial.printf("Role: REPEATER (id %d)\n", NODE_ID);
#else
    Serial.printf("Role: NODE (%d relays, %d sensors)\n",
                  NODE_NUM_RELAYS, NODE_NUM_SENSORS);
#endif
}

void loop() {

#if defined(ROLE_BASE)
    baseLoop();
#elif defined(ROLE_REPEATER)
    repeaterLoop();
#else
    nodeLoop();
#endif
}
