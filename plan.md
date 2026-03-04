# V2 Display Drivers: Complete Parity Plan

## Completed
- [x] ST7789 TFT driver — splash, status bar, writeMessage
- [x] Assets copied (splash.h with TFT+EPD bitmaps, icons.h) from V1
- [x] Base class virtual methods + status bar members
- [x] hardware.h/cpp forwarding (showSplash, drawStatusBar, updateStatusBar)
- [x] controller.h/cpp update() timer + splash/statusbar in Add flow
- [x] Main loop wiring (`_display_controller->update()` in `wippersnapper::loop()`)

## Remaining Work Overview

### A. EPD Drivers (6 drivers from `add-uc8253-plus-UC8179-UC8151-SSD1683` branch)
### B. EPD Infrastructure (base class, hardware.cpp, auto-detection)
### C. I2C Output Driver Bugs (already on `displays-v2`, need fixing)
### D. Missing I2C Output Driver (SH1107, dropped during migration)
### E. Splash Asset Update (EPD bitmap in splash.h — already copied, verify)

---

## A. EPD Drivers

Source branch: `add-uc8253-plus-UC8179-UC8151-SSD1683` (has all 6 EPD drivers).
The `displays-spi-experiments-v1` branch only had 2 (SSD1680, ILI0373).

| # | Driver Class | Chip | Panel | ThinkInk Class | Has showSplash? |
|---|---|---|---|---|---|
| 1 | `drvDispThinkInkGrayscale4Eaamfgn` | SSD1680 | 2.9" Grayscale4 (MagTag 2025+) | `ThinkInk_290_Grayscale4_EAAMFGN` | Yes (296x128) |
| 2 | `dispDrvThinkInkGrayscale4T5` | ILI0373 | 2.9" Grayscale4 (MagTag pre-2025) | `ThinkInk_290_Grayscale4_T5` | No |
| 3 | `dispDrvThinkInkGrayscale4MFGN` | SSD1683 | 4.2" Grayscale4 | `ThinkInk_420_Grayscale4_MFGN` | No |
| 4 | `dispDrvThinkInkMonoAAAMFGN` | UC8179 | 5.83" Mono (648x480) | `ThinkInk_583_Mono_AAAMFGN` | No |
| 5 | `dispDrvThinkInkMonoBAAMFGN` | UC8253 | 3.7" Mono (416x240, ADA6395) | `ThinkInk_370_Mono_BAAMFGN` | No |
| 6 | `dispDrvThinkInkMonoM06` | UC8151D | 2.9" Mono (296x128) | `ThinkInk_290_Mono_M06` | No |

### V2 Adaptations needed for each EPD driver:
- **writeMessage() signature**: V1 had `(const char *)`, V2 needs
  `(const char *, bool clear_first, int32_t cursor_x, int32_t cursor_y)`.
  `clear_first` → call `clearBuffer()` for area below status bar.
  `cursor_x`/`cursor_y` → default to status-bar-aware positions (0, STATUS_BAR_HEIGHT+4).
- **EPD constructor**: The UC8253 branch uses simplified constructor `(dc, rst, cs, sram_cs, busy)`.
  V2 should use this same approach (no mosi/sck/miso passthrough for EPDs — use platform
  SPI defaults). This matches V2's proto where `EpdSpiConfig` has no mosi/sck/miso fields.
- **STATUS_BAR defines**: Currently only defined in the EAAMFGN header. Move to `dispDrvBase.h`
  or a shared `statusbar_defs.h` so all EPD drivers can use them without include-order issues.

### Files to create:
```
src/components/display/drivers/dispDrvThinkInkGrayscale4Eaamfgn.h
src/components/display/drivers/dispDrvThinkInkGrayscale4T5.h
src/components/display/drivers/dispDrvThinkInkGrayscale4MFGN.h
src/components/display/drivers/dispDrvThinkInkMonoAAAMFGN.h
src/components/display/drivers/dispDrvThinkInkMonoBAAMFGN.h
src/components/display/drivers/dispDrvThinkInkMonoM06.h
```

---

## B. EPD Infrastructure

### B1. `src/components/display/drivers/dispDrvBase.h` — Add EPD support
- Add EPD constructor (matching UC8253 branch style, NO mosi/sck/miso):
  ```cpp
  dispDrvBase(int16_t dc, int16_t rst, int16_t cs,
              int16_t sram_cs = -1, int16_t busy = -1)
  ```
- Add protected members: `_pin_sram_cs`, `_pin_busy`
- Add virtual `begin(thinkinkmode_t mode, bool reset = true)` for EPD init
- Add `#include "Adafruit_ThinkInk.h"` for `thinkinkmode_t`
- Move STATUS_BAR defines here (shared by all EPD drivers):
  ```cpp
  #define WS_STATUSBAR_HEIGHT 20
  #define WS_STATUSBAR_BORDER 1
  #define WS_STATUSBAR_ICON_SZ 16
  #define WS_STATUSBAR_ICON_SPACING 4
  #define WS_STATUSBAR_ICON_MARGIN 5
  ```

### B2. `src/components/display/hardware.h` — Add EPD methods + includes
- Add includes for all 6 EPD driver headers
- Add private methods:
  - `bool beginSpiEpd(ws_display_Add *msg);`
  - `bool detect_ssd1680(uint8_t cs, uint8_t dc, uint8_t rst);`
  - `bool detect_ssd1683(uint8_t cs, uint8_t dc, uint8_t rst);`
  - `bool detect_uc8151d(uint8_t cs, uint8_t dc, uint8_t rst);`
  - `bool detect_uc8179(uint8_t cs, uint8_t dc, uint8_t rst);`
  - `bool detect_uc8253(uint8_t cs, uint8_t dc, uint8_t rst);`

### B3. `src/components/display/hardware.cpp` — Implement EPD begin + auto-detect
- Add `beginSpiEpd()`:
  - Parse pins from `ws_display_EpdSpiConfig`: pin_dc, pin_rst, pin_cs, pin_sram_cs, pin_busy
  - **No mosi/sck/miso** — V2's EpdSpiConfig doesn't have them; EPDs use platform SPI defaults
  - Driver string dispatch:
    | `msg->driver` | Driver Class |
    |---|---|
    | `"SSD1680"` | `drvDispThinkInkGrayscale4Eaamfgn` |
    | `"ILI0373"` | `dispDrvThinkInkGrayscale4T5` |
    | `"SSD1683"` | `dispDrvThinkInkGrayscale4MFGN` |
    | `"UC8179"` | `dispDrvThinkInkMonoAAAMFGN` |
    | `"UC8253"` | `dispDrvThinkInkMonoBAAMFGN` |
    | `"UC8151"` | `dispDrvThinkInkMonoM06` |
  - Component-name auto-detection (from UC8253 branch):
    | Component name pattern | Driver | Default mode |
    |---|---|---|
    | `"eink-magtag"` | auto-detect SSD1680 vs ILI0373 | GRAYSCALE4 |
    | `"eink-29-flexible-monochrome-296x128"` | UC8151 | MONO |
    | `"eink-37-monochrome-416x240"` | UC8253 | MONO |
    | `"eink-42-grayscale-300x400"` | SSD1683 | GRAYSCALE4 |
    | `"eink-583-monochrome-648x480"` | UC8179 | MONO |
  - Parse EPD mode: GRAYSCALE4 → `THINKINK_GRAYSCALE4`, MONO → `THINKINK_MONO`
  - Set width/height/text_size, call `begin(mode)`
- Add detection functions (bit-bang SPI register reads, from UC8253 branch):
  - `detect_ssd1680()` — register 0x71, returns `status == 0xFF`
  - `detect_ssd1683()` — register 0x2F, checks `(status & 0x03) == 0x01`
  - `detect_uc8151d()` — register 0x70, returns `rev != 0xFF`
  - `detect_uc8179()` — register 0x15, returns `dualspi != 0xFF`
  - `detect_uc8253()` — register 0x71, returns `status != 0xFF`
  - Factor common bit-bang logic into `EpdBitBangReadRegister()` helper
- Add `case ws_display_Add_spi_epd_tag:` to `begin()` switch

---

## C. I2C Output Driver Bugs (existing on `displays-v2`)

These drivers already exist at `src/components/i2c/drivers/` but have bugs in wiring:

### C1. SSD1306 OLED write dispatch broken
**File**: `src/components/i2c/controller.cpp` (Handle_I2cDeviceOutputWrite)
**Bug**: Calls `driver->WriteMessage()` which is a no-op in `drvOutSsd1306`.
The SSD1306 driver only overrides `WriteMessageSSD1306()`, not `WriteMessage()`.
**Fix**: In write handler, check display type and dispatch to `WriteMessageSSD1306()` for OLEDs,
OR make `drvOutSsd1306` override `WriteMessage()` directly (preferred — simpler).

### C2. CharLCD never configured
**File**: `src/components/i2c/controller.cpp` (around line 1017-1041)
**Bug**: For `ws_display_Add_config_char_lcd_tag`, prints debug but never calls
`drv_out->ConfigureCharLcd(rows, cols, backlight)`.
**Fix**: Add the `ConfigureCharLcd()` call.

### C3. CharLCD ConfigureCharLcd signature mismatch
**File**: `src/components/i2c/drivers/drvOutCharLcd.h`
**Bug**: Base declares `ConfigureCharLcd(uint32_t, uint32_t, bool)` but implementation
uses `(uint8_t, uint8_t, bool)` — hides rather than overrides.
**Fix**: Match the base class signature or use `override` keyword.

### C4. 7-Segment hardcodes I2C address
**File**: `src/components/i2c/drivers/drvOut7Seg.h`
**Bug**: `begin()` uses `_matrix->begin(0x70, _i2c)` instead of `_matrix->begin(_address, _i2c)`.
**Fix**: Use `_address`.

### C5. 7-Segment ConfigureI2CBackpack doesn't apply brightness
**File**: `src/components/i2c/drivers/drvOut7Seg.h`
**Bug**: `ConfigureI2CBackpack()` stores alignment but never writes `_brightness` or calls
`_matrix->setBrightness()`.
**Fix**: Store and apply brightness.

### C6. Duplicate LED_BACKPACK defines
**Files**: `drvOut7Seg.h` and `drvOutQuadAlphaNum.h`
**Bug**: Both define `LED_BACKPACK_ALIGNMENT_LEFT`, `LED_BACKPACK_ALIGNMENT_RIGHT`,
`LED_MAX_CHARS` — potential redefinition warnings.
**Fix**: Move to `drvOutputBase.h` or guard with `#ifndef`.

---

## D. Missing I2C Output Driver: SH1107

**Source**: V1 branch `displays-spi-experiments-v1` had `WipperSnapper_I2C_Driver_Out_Sh1107.h`.
It was dropped during the `migrate-api-v2-i2c-output` migration and never brought to `displays-v2`.

| Detail | Value |
|---|---|
| Chip | SH1107 |
| Hardware | 128x64 OLED FeatherWing |
| Library | `Adafruit_SH110X` |
| V1 class | `WipperSnapper_I2C_Driver_Out_SH1107` |
| V2 class (to create) | `drvOutSh1107` |

### File to create: `src/components/i2c/drivers/drvOutSh1107.h`
- Follow same pattern as `drvOutSsd1306.h`
- Use `Adafruit_SH1107` instead of `Adafruit_SSD1306`
- Wire into factory map in `src/components/i2c/controller.cpp`

---

## E. Splash Assets

The `splash.h` already copied from V1 contains three bitmaps:
- `tft_bmp_logo_240135[]` — 240x135 (Feather TFT/RevTFT) ✅ used by ST7789
- `tft_bmp_logo_240240[] PROGMEM` — 240x240 (FunHouse) ✅ used by ST7789
- `epd_bitmap_ws_logo_296128[]` — 296x128 (MagTag EPD) — will be used by EAAMFGN driver

No splash bitmaps exist for the larger EPD panels (4.2", 5.83", 3.7"). Those drivers'
`showSplash()` falls through to the base no-op. Future work could add panel-specific
splash screens.

---

## Implementation Order

### Phase 1: EPD Infrastructure + Drivers
1. Update `dispDrvBase.h` — EPD constructor, `begin(thinkinkmode_t)`, EPD pin members,
   shared STATUS_BAR defines
2. Create all 6 EPD driver files (adapted from UC8253 branch with V2 writeMessage signature)
3. Update `hardware.h` — EPD driver includes, detection method declarations, `beginSpiEpd()`
4. Update `hardware.cpp` — `beginSpiEpd()`, driver string dispatch, component-name mapping,
   all 5 detection functions, `EpdBitBangReadRegister()` helper, wire into `begin()` switch
5. Build `adafruit_feather_esp32s3_reversetft` (TFT regression)
6. Build EPD target if available

### Phase 2: I2C Output Driver Fixes
7. Fix SSD1306 write dispatch (override `WriteMessage()` directly)
8. Fix CharLCD config call + signature mismatch
9. Fix 7-Segment address + brightness bugs
10. Deduplicate LED_BACKPACK defines
11. Build + verify

### Phase 3: Missing Drivers
12. Create `drvOutSh1107.h` for SH1107 OLED
13. Wire into i2c controller factory
14. Build + verify

---

## Reference Commands

```bash
cd /c/dev/arduino/Adafruit_Wippersnapper_Arduino

# UC8253 branch (all 6 EPD drivers + detection + hardware.cpp)
git show add-uc8253-plus-UC8179-UC8151-SSD1683:src/components/display/drivers/dispDrvThinkInkGrayscale4Eaamfgn.h
git show add-uc8253-plus-UC8179-UC8151-SSD1683:src/components/display/drivers/dispDrvThinkInkGrayscale4MFGN.h
git show add-uc8253-plus-UC8179-UC8151-SSD1683:src/components/display/drivers/dispDrvThinkInkGrayscale4T5.h
git show add-uc8253-plus-UC8179-UC8151-SSD1683:src/components/display/drivers/dispDrvThinkInkMonoAAAMFGN.h
git show add-uc8253-plus-UC8179-UC8151-SSD1683:src/components/display/drivers/dispDrvThinkInkMonoBAAMFGN.h
git show add-uc8253-plus-UC8179-UC8151-SSD1683:src/components/display/drivers/dispDrvThinkInkMonoM06.h
git show add-uc8253-plus-UC8179-UC8151-SSD1683:src/components/display/hardware.cpp
git show add-uc8253-plus-UC8179-UC8151-SSD1683:src/components/display/hardware.h

# V1 branch (SH1107 driver for porting)
git show displays-spi-experiments-v1:src/components/i2c/drivers/WipperSnapper_I2C_Driver_Out_Sh1107.h

# V1 branch (SPI pin passthrough reference for DFRobot)
git show displays-spi-experiments-v1:src/components/display/drivers/dispDrvBase.h
```

## Architecture Notes

### V2 Driver Identification
V1 used an enum (`DisplayDriver`), V2 uses a string field `char driver[32]`:
- TFT: `strcmp(msg->driver, "ST7789")`
- EPD: `strcmp(msg->driver, "SSD1680")`, `"ILI0373"`, `"SSD1683"`, `"UC8179"`, `"UC8253"`, `"UC8151"`
- I2C outputs: keyed by string in `I2cFactoryOutput` map: `"ssd1306"`, `"sh1107"`, `"charlcd"`, `"7seg"`, `"quadalphanum"`

### V2 EPD SPI Pins
V2's `ws_display_EpdSpiConfig` has: bus, pin_dc, pin_rst, pin_cs, pin_sram_cs, pin_busy.
No mosi/sck/miso — EPDs use platform SPI defaults. This matches the UC8253 branch approach.

### TFT SPI Pins (DFRobot support)
V2's TFT constructor in `dispDrvBase` already has mosi/sck/miso passthrough.
V2's `ws_display_TftSpiConfig` has: bus, pin_cs, pin_dc, pin_mosi, pin_sck, pin_rst, pin_miso.
This already supports DFRobot-style boards where TFT uses non-default SPI pins.
The `bus != 0` guard in `beginSpiTft()` is a known limitation — non-default SPI bus
selection is future work (needs `SPIClass` instantiation per bus number).

### I2C Output Architecture
I2C output drivers live at `src/components/i2c/drivers/drvOut*.h`, extending `drvOutputBase`
(which extends `drvBase`). They are managed by the I2C controller (`src/components/i2c/controller.cpp`)
via the `I2cFactoryOutput` map. They already use `ws_display_Write` / `ws_display_Add` proto types
from `display.pb.h` — NOT from a separate `i2c_output.pb.h`.


## Latest plan 20:23 2026-03-04
 Plan to implement                                                                                                                                                                │
│                                                                                                                                                                                  │
│ Plan: Add Qualia Board + RGB666 Dotclock Display Drivers                                                                                                                         │
│                                                                                                                                                                                  │
│ Context                                                                                                                                                                          │
│                                                                                                                                                                                  │
│ The displays-v2 branch needs support for the Adafruit Qualia ESP32-S3 board (PID 5800)                                                                                           │
│ and two RGB666 dotclock displays: the 2.1" round (PID 5792, 480x480) and the 3.2" bar                                                                                            │
│ (PID 5797, 320x820). Both use ST7701S with SPI init + RGB dotclock pixel data. The proto                                                                                         │
│ already defines ws_display_Add_ttl_rgb666_tag and TtlRgb666Config.                                                                                                               │
│                                                                                                                                                                                  │
│ Uses Arduino_GFX_Library (moononournation) as a Qualia-only dependency — this is what                                                                                            │
│ Adafruit's own examples use. The lib is guarded behind #ifdef ARDUINO_ADAFRUIT_QUALIA_S3_RGB666.                                                                                 │
│                                                                                                                                                                                  │
│ 1. Add Qualia to ws_boards.h (line ~242, before #else)                                                                                                                           │
│                                                                                                                                                                                  │
│ #elif defined(ARDUINO_ADAFRUIT_QUALIA_S3_RGB666)                                                                                                                                 │
│ #define BOARD_ID "qualia-s3-rgb666"                                                                                                                                              │
│ #define USE_TINYUSB                                                                                                                                                              │
│ #define USE_PSRAM                                                                                                                                                                │
│ #define BOOT_BUTTON 0                                                                                                                                                            │
│                                                                                                                                                                                  │
│ No status LED/NeoPixel — board has none. Status bar on display serves as indicator.                                                                                              │
│                                                                                                                                                                                  │
│ 2. Add PlatformIO env to platformio.ini                                                                                                                                          │
│                                                                                                                                                                                  │
│ [env:adafruit_qualia_s3_rgb666]                                                                                                                                                  │
│ extends = common:esp32                                                                                                                                                           │
│ board = adafruit_qualia_s3_rgb666                                                                                                                                                │
│ build_flags = -DARDUINO_ADAFRUIT_QUALIA_S3_RGB666 -DBOARD_HAS_PSRAM                                                                                                              │
│ board_build.partitions = tinyuf2-partitions-16MB.csv                                                                                                                             │
│ extra_scripts = pre:rename_usb_config.py                                                                                                                                         │
│ lib_deps =                                                                                                                                                                       │
│     ${env.lib_deps}                                                                                                                                                              │
│     moononournation/GFX Library for Arduino                                                                                                                                      │
│                                                                                                                                                                                  │
│ 3. Create dispDrvRgb666.h                                                                                                                                                        │
│                                                                                                                                                                                  │
│ File: src/components/display/drivers/dispDrvRgb666.h                                                                                                                             │
│                                                                                                                                                                                  │
│ Entire file guarded with #ifdef ARDUINO_ADAFRUIT_QUALIA_S3_RGB666.                                                                                                               │
│                                                                                                                                                                                  │
│ Uses Arduino_GFX classes:                                                                                                                                                        │
│ - Arduino_XCA9554SWSPI — PCA9554A IO expander (backlight, reset, SPI for display init)                                                                                           │
│ - Arduino_ESP32RGBPanel — ESP32-S3 RGB dotclock bus                                                                                                                              │
│ - Arduino_RGB_Display — Display with init sequence                                                                                                                               │
│                                                                                                                                                                                  │
│ Panel selection via panel string from Add message:                                                                                                                               │
│                                                                                                                                                                                  │
│ ┌──────────────┬──────┬────────────┬────────────────────────────┐                                                                                                                │
│ │ Panel string │ PID  │ Resolution │       Init ops array       │                                                                                                                │
│ ├──────────────┼──────┼────────────┼────────────────────────────┤                                                                                                                │
│ │ "TL021WVC02" │ 5792 │ 480x480    │ TL021WVC02_init_operations │                                                                                                                │
│ ├──────────────┼──────┼────────────┼────────────────────────────┤                                                                                                                │
│ │ "TL032FWV01" │ 5797 │ 320x820    │ tl032fwv01_init_operations │                                                                                                                │
│ └──────────────┴──────┴────────────┴────────────────────────────┘                                                                                                                │
│                                                                                                                                                                                  │
│ Constructor pattern (from Qualia_S3_Product_Demo.ino):                                                                                                                           │
│ Arduino_XCA9554SWSPI *expander = new Arduino_XCA9554SWSPI(                                                                                                                       │
│     PCA_TFT_RESET, PCA_TFT_CS, PCA_TFT_SCK, PCA_TFT_MOSI, &Wire, 0x3F);                                                                                                          │
│ Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(                                                                                                                     │
│     TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK,                                                                                                                                      │
│     TFT_R1, TFT_R2, TFT_R3, TFT_R4, TFT_R5,                                                                                                                                      │
│     TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,                                                                                                                              │
│     TFT_B1, TFT_B2, TFT_B3, TFT_B4, TFT_B5,                                                                                                                                      │
│     1, 46, 2, 44,  1, 50, 16, 16);                                                                                                                                               │
│ Arduino_RGB_Display *gfx = new Arduino_RGB_Display(                                                                                                                              │
│     480, 480, rgbpanel, 0, true,                                                                                                                                                 │
│     expander, GFX_NOT_DEFINED, TL021WVC02_init_operations,                                                                                                                       │
│     sizeof(TL021WVC02_init_operations));                                                                                                                                         │
│                                                                                                                                                                                  │
│ All TFT_* and PCA_* pin defines from Qualia board variant pins_arduino.h.                                                                                                        │
│ Init operation arrays provided by Arduino_GFX_Library.                                                                                                                           │
│                                                                                                                                                                                  │
│ Class: dispDrvRgb666 : public dispDrvBase                                                                                                                                        │
│ - Uses the no-pin base constructor (pins are board-hardwired)                                                                                                                    │
│ - begin() — creates expander, rgbpanel, display; enables backlight                                                                                                               │
│ - writeMessage() — V2 signature with clear_first/cursor_x/cursor_y                                                                                                               │
│ - showSplash() — no-op (no splash bitmap for these panels yet)                                                                                                                   │
│ - drawStatusBar() / updateStatusBar() — same pattern as TFT, using                                                                                                               │
│ Arduino_GFX's Adafruit_GFX-compatible API (fillRect, drawBitmap, etc.)                                                                                                           │
│                                                                                                                                                                                  │
│ 4. Update dispDrvBase.h                                                                                                                                                          │
│                                                                                                                                                                                  │
│ Add no-pin constructor for board-integrated displays:                                                                                                                            │
│ dispDrvBase() {} // For displays with hardwired pins (e.g., Qualia RGB666)                                                                                                       │
│                                                                                                                                                                                  │
│ 5. Update hardware.h                                                                                                                                                             │
│                                                                                                                                                                                  │
│ - Add #ifdef ARDUINO_ADAFRUIT_QUALIA_S3_RGB666 guarded include of dispDrvRgb666.h                                                                                                │
│ - Add bool beginTtlRgb666(ws_display_Add *msg); declaration                                                                                                                      │
│                                                                                                                                                                                  │
│ 6. Update hardware.cpp                                                                                                                                                           │
│                                                                                                                                                                                  │
│ - Add case ws_display_Add_ttl_rgb666_tag: return beginTtlRgb666(msg); to begin() switch                                                                                          │
│ - Implement beginTtlRgb666():                                                                                                                                                    │
│   - Parse TtlRgb666Config for width/height/rotation/text_size                                                                                                                    │
│   - Select panel init ops from msg->panel string                                                                                                                                 │
│   - Create dispDrvRgb666 instance with panel selection                                                                                                                           │
│   - Call begin(), set dimensions, return                                                                                                                                         │
│                                                                                                                                                                                  │
│ 7. Update controller.cpp                                                                                                                                                         │
│                                                                                                                                                                                  │
│ Add component-name resolution for Qualia displays in resolveEpdDefaults()                                                                                                        │
│ (rename to resolveDisplayDefaults() or add separate function):                                                                                                                   │
│ "qualia-round-480x480" → driver="ST7701S", panel="TL021WVC02"                                                                                                                    │
│ "qualia-bar-320x820"   → driver="ST7701S", panel="TL032FWV01"                                                                                                                    │
│                                                                                                                                                                                  │
│ Files to modify                                                                                                                                                                  │
│                                                                                                                                                                                  │
│ 1. src/ws_boards.h — add Qualia board define                                                                                                                                     │
│ 2. platformio.ini — add Qualia env + Arduino_GFX lib dep                                                                                                                         │
│ 3. src/components/display/drivers/dispDrvBase.h — add no-pin constructor                                                                                                         │
│ 4. src/components/display/drivers/dispDrvRgb666.h — NEW RGB666 driver                                                                                                            │
│ 5. src/components/display/hardware.h — add include + beginTtlRgb666                                                                                                              │
│ 6. src/components/display/hardware.cpp — implement beginTtlRgb666 + wire switch                                                                                                  │
│ 7. src/components/display/controller.cpp — add Qualia component-name resolution                                                                                                  │
│                                                                                                                                                                                  │
│ Verification                                                                                                                                                                     │
│                                                                                                                                                                                  │
│ 1. Build adafruit_qualia_s3_rgb666 — should compile with Arduino_GFX dep                                                                                                         │
│ 2. Build adafruit_feather_esp32s3_reversetft — TFT regression (no Arduino_GFX pulled)                                                                                            │
│ 3. RGB666 driver is #ifdef guarded — other board targets unaffected  