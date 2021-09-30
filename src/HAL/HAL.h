/*
 * Hardware Abstraction Layer (HAL)
 * 
 * Copyright (C) 2018 Khalid Baheyeldin
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#if defined(__AVR_ATmega328P__)
  #define MCU_STR "AtMega328"
  #include "HAL_AtMega328.h"

#elif defined(__AVR_ATmega1280__)
  #define MCU_STR "Mega1280"
  #include "HAL_Mega2560.h"

#elif defined(__AVR_ATmega2560__)
  #define MCU_STR "Mega2560"
  #include "HAL_Mega2560.h"

#elif defined(_mk20dx128_h_) || defined(__MK20DX128__)
  // Teensy 3.0
  #define MCU_STR "Teensy3.0"
  #include "HAL_Teensy_3.2.h"

#elif defined(__MK20DX256__)
  // Teensy 3.2
  #define MCU_STR "Teensy3.2"
  #include "HAL_Teensy_3.2.h"

#elif defined(__MK64FX512__)
  // Teensy 3.5
  #define MCU_STR "Teensy3.5"
  #include "HAL_Teensy_3.5.h"

#elif defined(__MK66FX1M0__)
  // Teensy 3.6
  #define MCU_STR "Teensy3.6"
  #include "HAL_Teensy_3.6.h"

#elif defined(ARDUINO_TEENSY40)
  // Teensy 4.0
  #define MCU_STR "Teensy4.0"
  #include "HAL_Teensy_4.0.h"

#elif defined(ARDUINO_TEENSY41)
  // Teensy 4.1
  #define MCU_STR "Teensy4.1"
  #include "HAL_Teensy_4.1.h"

#elif defined(STM32F103xB)
  // STM32F103C8/CB: 72MHz, 128K flash, 64K RAM, ARM Cortex M3
  #define MCU_STR "STM32F103"
  #include "HAL_STM32F103.h"

#elif defined(STM32F303xC)
  // RobotDyn BlackPill STM32F303, 256K flash, ARM Cortex M4 (STM32duino board manager)
  #define MCU_STR "STM32F303"
  #include "HAL_STM32F303.h"

#elif defined(STM32F401xC)
  // Blackpill board with STM32F401CC
  #define MCU_STR "STM32F401"
  #include "HAL_STM32F4x1.h"

#elif defined(STM32F411xE)
  // Blackpill board with STM32F411CE
  #define MCU_STR "STM32F411"
  #include "HAL_STM32F4x1.h"

#elif defined(STM32F446xx)
  // FYSETC S6 board with STM32F446
  #define MCU_STR "STM32F446"
  #include "HAL_STM32F446.h"

#elif defined(ESP32)
  // ESP32
  #define MCU_STR "ESP32"
  #include "HAL_ESP32.h"

#elif defined(__SAM3X8E__)
  // Arduino Due
  #define MCU_STR "SAM3X8E (Arduino DUE)"
  #include "HAL_Due.h"  
  
#else
  // Generic
  #warning "Unknown Platform! If this is a new platform, it would probably do best with a new HAL designed for it."
  #define MCU_STR "Generic (Unknown)"
  #include "HAL_MISC.h"  
#endif

// create null decoration for non-ESP processors
#ifndef IRAM_ATTR
  #define IRAM_ATTR
#endif

#ifndef ICACHE_RAM_ATTR
  #define ICACHE_RAM_ATTR
#endif

#ifndef FPSTR
  #define FPSTR
#endif
