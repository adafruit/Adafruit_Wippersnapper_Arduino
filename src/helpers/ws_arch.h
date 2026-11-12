/*!
 * @file src/helpers/ws_arch.h
 *
 * Architecture macros that the Arduino cores don't define themselves.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_ARCH_H
#define WS_ARCH_H

// arduino-pico defines ARDUINO_ARCH_RP2040 for both the RP2040 and the
// RP2350, and marks the RP2350 with PICO_RP2350 (from its
// lib/rp2350/platform_def.txt, for Arduino IDE and PlatformIO builds alike).
// Nothing defines ARDUINO_ARCH_RP2350, which the RP2350 sleep and watchdog
// code is guarded on, so without this it never compiled.
#if defined(ARDUINO_ARCH_RP2040) && defined(PICO_RP2350) &&                    \
    !defined(ARDUINO_ARCH_RP2350)
#define ARDUINO_ARCH_RP2350 ///< RP2350 build of arduino-pico
#endif

#endif // WS_ARCH_H
