// Platform setup ------------------------------------------------------------------------------------
#pragma once

// We define a more generic symbol, in case more Platform_Name boards based on different lines are supported

// 1/100 second resolution
#define HAL_FRACTIONAL_SEC 100.0

// Most platforms default to 8 bit PWM
#ifndef HAL_ANALOG_WRITE_BITS
  #define HAL_ANALOG_WRITE_BITS 8
#endif
#ifndef HAL_ANALOG_WRITE_RANGE
  #define HAL_ANALOG_WRITE_RANGE 255
#endif

// Lower limit (fastest) step rate in uS for this platform (in SQW mode) and width of step pulse
#ifndef HAL_MAXRATE_LOWER_LIMIT
  #define HAL_MAXRATE_LOWER_LIMIT 60
#endif
#ifndef HAL_PULSE_WIDTH
  #define HAL_PULSE_WIDTH 10000
#endif

// New symbol for the default I2C port -------------------------------------------------------------
#include <Wire.h>
#define HAL_Wire Wire
#define HAL_WIRE_CLOCK 20000

// Non-volatile storage ----------------------------------------------------------------------------
#ifdef NV_DEFAULT
  #include "../lib/nv/NV_EEPROM.h"
#endif

//--------------------------------------------------------------------------------------------------
// General purpose initialize for HAL
#define HAL_INIT { \
  nv.init(2048, true, 0, false); \
}

#define HAL_RESET() { }

//--------------------------------------------------------------------------------------------------
// Internal MCU temperature (in degrees C)
#define HAL_TEMP ( NAN )

// Watchdog control macros --------------------------------------------------------------------------
#if WATCHDOG != OFF
  #define WDT_ENABLE
  #define WDT_RESET
  #define WDT_DISABLE
#else
  #define WDT_ENABLE
  #define WDT_RESET
  #define WDT_DISABLE
#endif
