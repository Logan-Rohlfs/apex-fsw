#pragma once
#include <Arduino.h>

// APEX_MONITOR — connected to the Apex monitor app (laptop debug / HIL prep).
//   >key:value  numeric data  → routed to live plots
//   !key:value  state fields  → routed to state panel
//   #LEVEL: msg log output   → routed to log panel
//   Inbound bytes are parsed as newline-terminated commands (ARM, DISARM, …).
//
// (no flag)    — flight build, all macros compile to nothing.

#ifdef APEX_MONITOR
  #define LOG_INFO(fmt, ...)  Serial.printf("#INFO: "  fmt "\n", ##__VA_ARGS__)
  #define LOG_WARN(fmt, ...)  Serial.printf("#WARN: "  fmt "\n", ##__VA_ARGS__)
  #define LOG_ERROR(fmt, ...) Serial.printf("#ERROR: " fmt "\n", ##__VA_ARGS__)
  #define LOG_DEBUG(fmt, ...) do {} while(0)
#else
  // Flight build — zero overhead
  #define LOG_INFO(...)  do {} while(0)
  #define LOG_WARN(...)  do {} while(0)
  #define LOG_ERROR(...) do {} while(0)
  #define LOG_DEBUG(...) do {} while(0)
#endif
