#pragma once
#include <Adafruit_BME280.h>

extern Adafruit_BME280 bme;

void sensorInit();
void sensorLoop();
bool sensorAvailable();