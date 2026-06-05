#pragma once
#include <Arduino.h>

// APEX_DEBUG  — human-readable output for PlatformIO serial monitor
// APEX_PLOT   — structured output for apex monitor app
//               Log messages still emitted with #LEVEL: prefix so the
//               monitor routes them to its log panel.
// (neither)   — flight build, all macros compile to nothing

#if defined(APEX_DEBUG)
  #define LOG_INFO(fmt, ...)  Serial.printf("[INFO  %6lu] " fmt "\n", millis(), ##__VA_ARGS__)
  #define LOG_WARN(fmt, ...)  Serial.printf("[WARN  %6lu] " fmt "\n", millis(), ##__VA_ARGS__)
  #define LOG_ERROR(fmt, ...) Serial.printf("[ERROR %6lu] " fmt "\n", millis(), ##__VA_ARGS__)
  #define LOG_DEBUG(fmt, ...) Serial.printf("[DEBUG %6lu] " fmt "\n", millis(), ##__VA_ARGS__)
  #define LOG_RAW(fmt, ...)   Serial.printf(fmt, ##__VA_ARGS__)

#elif defined(APEX_PLOT)
  // In plot mode, prefix with # so the monitor routes these to the log panel.
  // No timestamp — the monitor adds its own wall-clock time.
  #define LOG_INFO(fmt, ...)  Serial.printf("#INFO: "  fmt "\n", ##__VA_ARGS__)
  #define LOG_WARN(fmt, ...)  Serial.printf("#WARN: "  fmt "\n", ##__VA_ARGS__)
  #define LOG_ERROR(fmt, ...) Serial.printf("#ERROR: " fmt "\n", ##__VA_ARGS__)
  #define LOG_DEBUG(fmt, ...) do {} while(0)
  #define LOG_RAW(fmt, ...)   do {} while(0)

#else
  // Flight build — zero overhead
  #define LOG_INFO(...)   do {} while(0)
  #define LOG_WARN(...)   do {} while(0)
  #define LOG_ERROR(...)  do {} while(0)
  #define LOG_DEBUG(...)  do {} while(0)
  #define LOG_RAW(...)    do {} while(0)
#endif
