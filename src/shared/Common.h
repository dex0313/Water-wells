#pragma once

// Общие параметры проекта
#define LORA_CS   10
#define LORA_RST  14
#define LORA_IRQ  26
#define LORA_FREQ 433E6

#if defined(BUILD_MODE) && BUILD_MODE == TEST
  #define SEND_INTERVAL 2000      // каждые 2 секунды
#else
  #define SEND_INTERVAL 3600000   // каждые 60 минут
#endif

// Объявление общих функций
void printHeader(const char* role, const char* mode);
