---
name: add-board-v1
description: >
  Guides adding a new board (microcontroller / development board) to Adafruit IO WipperSnapper v1
  firmware and the Wippersnapper_Boards companion definition repo, including adding support for
  new WiFi coprocessors (with an existing main MCU or a new board). Use this skill whenever the
  user wants to add a new board, port WipperSnapper to a new microcontroller, support a new
  development board, add a new PlatformIO environment, create a board definition for
  WipperSnapper, or enable WipperSnapper on a new piece of hardware — even if they just mention
  a board name and "wippersnapper" in the same breath. Covers the full workflow: platformio.ini
  environment, Wippersnapper_Boards.h board ID and status LED config, network interface selection,
  provisioning method (TinyUSB vs LittleFS), Wippersnapper_Networking.h updates if needed, CI
  workflow matrix additions, Wippersnapper_Boards repo definition.json, build verification,
  clang-format, and PR creation for both repos.
---

# Add a New Board to WipperSnapper v1

Changes span **two repositories**:
1. **Adafruit_Wippersnapper_Arduino** — C++ firmware: PlatformIO env, board identification,
   network interface, provisioning, CI matrix
2. **Wippersnapper_Boards** — board `definition.json` (including pin/component definitions and
   optional magic auto-config for onboard components such as non-user-facing SPI pins for a WiFi
   coprocessor or I2C pins for an onboard sensor) used by CI build/flash tooling and the
   Adafruit IO device installer

> **The `Wippersnapper_Boards` folder is git-ignored in the firmware repo** (similar to how
> `Wippersnapper_Components` is git-ignored for the sensor skill). Do not leave board definition
> files as untracked files in the firmware tree — they belong in the separate boards repo. If the
> boards repo is not pushable AND cannot be forked (e.g. running in a GitHub agent session without
> push access), include the **full content** of the board definition files (`definition.json`
> etc.) in the firmware PR summary/response so the user can file them manually in the boards repo.

The user supplies a board name (e.g. "QT Py ESP32-C6"). Research the board thoroughly before
writing any code: find the product page, documentation URLs (learn guide, wiki), associated
schematics and pin diagrams, example sketches, and relevant datasheets (note: datasheets are
often for the MCU/SoC rather than board-specific). Identify the MCU family, network
capabilities, and walk through every file change.

### Key concepts

- **Network interface:** How the board connects to WiFi. There are four existing drivers; most
  new boards reuse one. Only create a new driver if the board uses a completely new networking
  stack.
- **Provisioning:** How credentials (`secrets.json`) reach the board. Two options exist:
  **TinyUSB** (USB Mass Storage — board appears as a flash drive) or **LittleFS** (credentials
  baked into flash via a web-based provisioner).
- **Board ID:** A kebab-case string (e.g. `"feather-esp32s3-tft"`) defined in
  `Wippersnapper_Boards.h` that uniquely identifies the hardware to Adafruit IO.

> **Proto files are off-limits.** Only Adafruit staff modify `.proto` files.

## Arguments

This skill accepts a board name as its argument (e.g. `/add-board-v1 QT Py ESP32-C6`).

Ideally the user will provide the product page URL, learn guide, or PlatformIO board ID. If not,
the skill will research these based on the board name.

## Environment Check (optional, do not block on this)

If Bash is available, quickly check connectivity to adapt your approach. **If Bash is not
available or these commands fail, skip this section and proceed — use WebFetch/WebSearch for
research, and do the code changes with the file tools you have. The user can handle git/PRs.**

```bash
curl -s -o /dev/null -w "%{http_code}" https://www.bbc.com       # general web access
curl -s -o /dev/null -w "%{http_code}" https://api.github.com     # GitHub API (needed for gh)
git ls-remote https://github.com/adafruit/Wippersnapper_Boards HEAD  # git clone access
gh auth status  # see if cli/token/login present
```

| Result | Capability |
|--------|-----------|
| All succeed | Full access — use `gh` for forking, PRs, API queries |
| BBC fails, GitHub API works | Restricted web but `gh` works — skip web fetches for product pages |
| BBC + API fail, git works | Git-only — use `git clone`/`git push`, create PRs manually |
| All fail / no Bash | Offline — write code with file tools, user handles git/PRs |

## CI Checks

PRs to both repos run CI. Key checks to pass before submitting:

**Adafruit_Wippersnapper_Arduino** (`.github/workflows/build-clang-doxy.yml`):
- **Build** — firmware must compile for all target boards. New boards must be added to the
  correct CI build matrix job.
- **clang-format** — code formatting must match `.clang-format` config. Run `clang-format -i`
  on all changed `.h`/`.cpp` files in `src/`. The clang job excludes `src/nanopb`,
  `src/wippersnapper`, `src/pb.h`, and `src/provisioning/tinyusb`.
- **Doxygen** — all public/protected methods need Doxygen-style `/*! @brief ... */` comment
  blocks.

**Wippersnapper_Boards:**
- **Board definition validation** — `definition.json` must include required fields for
  esptool/build tooling.

---

## Reference: Supported Platform Families

Before writing any code, understand which platform family the board belongs to. Each family has
a shared PlatformIO config section and an associated network interface:

| Platform Family | `[common:*]` section | Network Interface | Arch Define |
|---|---|---|---|
| ESP32 / ESP32-S2 / S3 / C3 / C6 / C5 | `common:esp32` | `Wippersnapper_ESP32` | `ARDUINO_ARCH_ESP32` |
| ESP8266 | `common:esp8266` | `Wippersnapper_ESP8266` | `ARDUINO_ARCH_ESP8266` |
| SAMD51 (with AirLift co-processor) | `common:atsamd` | `Wippersnapper_AIRLIFT` | board-specific defines |
| RP2040 / RP2350 (Pico W, Pico 2W, Fruit Jam) | `common:arduinopico` | `ws_networking_pico` or `Wippersnapper_AIRLIFT` | `ARDUINO_ARCH_RP2040` |

**Special case — Fruit Jam RP2350:** Uses `common:arduinopico` platform but the AIRLIFT
network interface (it has an onboard ESP32-C6 co-processor accessed via SPI/NINA-FW).

### Network Interface Decision Tree

The wireless interface type is the key distinguisher when selecting a network driver:

- **Built-in WiFi (main MCU has WiFi)** — e.g. ESP32, ESP32-S2/S3/C3/C6/C5, Pico W (CYW43).
  Use the existing driver for that platform; no new network interface needed.
- **USE_AIRLIFT** — external ESP32 or ESP32-C6 co-processor via SPI running Adafruit's
  Nina-FW (AirLift firmware). Used by SAMD boards (Metro M4 AirLift, PyPortal) and some
  RP2xxx boards (Fruit Jam with onboard ESP32-C6). Uses `Wippersnapper_AIRLIFT`.
- **WIFI_NINA** — external ESP32 co-processor running Arduino's official Nina-FW (NOT
  Adafruit's). Used by Arduino MKR WiFi 1010, Nano 33 IoT. Uses the WiFiNINA library.
  **Different from AirLift** despite similar hardware.
- **CYW43** — Infineon/Cypress CYW43439 or CYW43455 WiFi chip. Used by most RP2xxx boards
  (Pico W, Pico 2W). Uses the built-in WiFi stack via the arduino-pico core
  (`ws_networking_pico`).
- **ESP_HOSTED** — ESP32 as a network co-processor to a non-ESP32 main MCU via SPI/SDIO.
  Newer pattern used by boards like ESP32-P4 with ESP32-C5/C6/S3 as the WiFi co-processor.
  May require a new network interface driver if not yet supported.

## Reference: Provisioning Methods

| Method | `#define` in `Wippersnapper_Boards.h` | Used by | How credentials arrive |
|---|---|---|---|
| **TinyUSB** | `USE_TINYUSB` | ESP32-S2, ESP32-S3, SAMD51, RP2040/RP2350, Xiao ESP32-S3 | Board mounts as USB flash drive; user drops `secrets.json` |
| **LittleFS** | `USE_LITTLEFS` | ESP32 (original), ESP32-V2, ESP32-C3, ESP32-C6, ESP32-C5, ESP8266, ItsyBitsy ESP32, Sparkle Motion | Web-based provisioner writes credentials to LittleFS partition |

**Rule of thumb:** If the board has native USB (USB OTG) and enough flash for a FAT partition,
use TinyUSB. If it only has USB-to-UART (no native USB), use LittleFS.

## Reference: Status LED Types

Each board defines its status indicator in `Wippersnapper_Boards.h`:

| Type | `#define` | Additional defines |
|---|---|---|
| NeoPixel | `USE_STATUS_NEOPIXEL` | `STATUS_NEOPIXEL_PIN`, `STATUS_NEOPIXEL_NUM` |
| DotStar | `USE_STATUS_DOTSTAR` | `STATUS_DOTSTAR_PIN_DATA`, `STATUS_DOTSTAR_PIN_CLK`, `STATUS_DOTSTAR_NUM`, `STATUS_DOTSTAR_COLOR_ORDER` |
| Plain LED | `USE_STATUS_LED` | `STATUS_LED_PIN` |

Most Adafruit boards with NeoPixels use `PIN_NEOPIXEL` and `NEOPIXEL_NUM` from the board's
Arduino core variant header. Check the board's `pins_arduino.h` or variant file.

---

## Step 0 — Research the Board **MUST BE DONE BEFORE WRITING ANY CODE**

Before writing any code, gather this information:

| What | Where to look |
|------|---------------|
| Product page | Search the web for "adafruit <BOARD>" to find `https://www.adafruit.com/product/<ID>`. The product page links to the learn guide. |
| Learn guide | Fetch the `.md?view=all` version. The guide describes the MCU, connectivity, pinout, status LEDs, and Arduino setup. |
| MCU / SoC | Product page or datasheet — determines the platform family (ESP32, SAMD, RP2040, etc.) |
| WiFi capability | Built-in (ESP32, Pico W) vs co-processor (AirLift) vs none (not supported) |
| Flash size & PSRAM | Product page or MCU datasheet — affects partition table and `USE_PSRAM` |
| Native USB | Determines TinyUSB vs LittleFS provisioning |
| Status LED | NeoPixel, DotStar, or plain LED — check the learn guide pinout page |
| PlatformIO board ID | Search `pio boards <BOARD>` or check PlatformIO board registry. For Adafruit boards, often `adafruit_<board_name>`. |
| Arduino preprocessor define | Check the board's Arduino core variant. For ESP32 boards: `ARDUINO_<BOARD_NAME>`. Find this in the board's `boards.txt` or variant header. |
| Existing similar board | Browse `platformio.ini` for a board in the same family with similar specs |

### Finding the Arduino preprocessor define

This is critical — it determines how `Wippersnapper_Boards.h` and `Wippersnapper_Networking.h`
identify the board at compile time.

**For ESP32 boards (pioarduino):**
```bash
# Search the Arduino ESP32 core's boards.txt for the board
# The define is typically set via build.extra_flags or build.defines
# e.g., -DARDUINO_ADAFRUIT_QTPY_ESP32C6
gh api repos/espressif/arduino-esp32/contents/boards.txt --jq '.content' | base64 -d | grep -i "<board_name>"
```

**For RP2040/RP2350 boards (arduino-pico):**
```bash
gh api repos/earlephilhower/arduino-pico/contents/boards.txt --jq '.content' | base64 -d | grep -i "<board_name>"
```

**For SAMD boards (Adafruit SAMD core):**
```bash
gh api repos/adafruit/ArduinoCore-samd/contents/boards.txt --jq '.content' | base64 -d | grep -i "<board_name>"
```

If `gh` is unavailable, check the learn guide or use WebFetch on the raw boards.txt URL.

---

## Step 1 — Add Board Identification in `Wippersnapper_Boards.h`

**File:** `src/Wippersnapper_Boards.h`

Add a new `#elif defined(ARDUINO_<BOARD_DEFINE>)` block **before the final `#else`**. Place it
in a logical position near boards of the same family.

Each block must define:

```cpp
#elif defined(ARDUINO_<BOARD_DEFINE>)
#define BOARD_ID "<board-id>"          // kebab-case, matches Wippersnapper_Boards repo folder
#define USE_TINYUSB                    // or USE_LITTLEFS — see provisioning table above
#define USE_STATUS_NEOPIXEL            // or USE_STATUS_DOTSTAR or USE_STATUS_LED
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL  // from board variant header
#define STATUS_NEOPIXEL_NUM 1          // or NEOPIXEL_NUM if defined
#define USE_PSRAM                      // only if the board has PSRAM
```

**BOARD_ID naming convention:**
- Lowercase kebab-case, based on the board name as shown in the Adafruit IO UI
- Has a maximum length enforced by the Wippersnapper_Boards repo's PR CI validation — check the
  boards repo schema or CI workflow for the exact limit before choosing a name
- Ideally match the Arduino FQBN or PlatformIO target name where possible
- Should include the **board manufacturer** (not the MCU manufacturer) — e.g. `"adafruit-feather"`
  not `"espressif-feather"`
- Examples from existing boards:
  `"feather-esp32s3-tft"`, `"qtpy-esp32c3"`, `"rpi-pico-w"`, `"fruitjam"`, `"magtag"`

**PSRAM:** Only define `USE_PSRAM` if the board has PSRAM. For boards where PSRAM is detected
at runtime, use the conditional pattern:
```cpp
#ifdef BOARD_HAS_PSRAM
#define USE_PSRAM
#endif
```

---

## Step 2 — Add PlatformIO Environment in `platformio.ini`

**File:** `platformio.ini`

Add a new `[env:<board_env_name>]` section. Study existing boards of the same platform family
and copy the pattern.

### ESP32 family template (most common)

```ini
; <Board Description>
[env:<board_env_name>]
extends = common:esp32
board = <platformio_board_id>
build_flags = -DARDUINO_<BOARD_DEFINE>
board_build.filesystem = littlefs              ; for LittleFS boards
board_build.partitions = min_spiffs.csv        ; for LittleFS boards
; --- OR for TinyUSB boards: ---
; build_flags = -DARDUINO_<BOARD_DEFINE> -DBOARD_HAS_PSRAM
; board_build.partitions = tinyuf2-partitions-4MB-noota.csv
; extra_scripts = pre:rename_usb_config.py
```

### Key per-board variations

| Setting | When to use | Example |
|---|---|---|
| `extends` | Always — pick the correct `common:*` section | `common:esp32`, `common:arduinopico` |
| `board` | Always — PlatformIO board identifier | `adafruit_feather_esp32s3` |
| `build_flags` | Always — at minimum the Arduino preprocessor define | `-DARDUINO_ADAFRUIT_FEATHER_ESP32S3` |
| `board_build.filesystem` | LittleFS boards only | `littlefs` |
| `board_build.partitions` | Depends on flash size and provisioning | `min_spiffs.csv`, `tinyuf2-partitions-4MB-noota.csv`, `default_8MB.csv` |
| `extra_scripts` | TinyUSB ESP32-S2/S3 boards only | `pre:rename_usb_config.py` |
| `-DBOARD_HAS_PSRAM` | Boards with PSRAM | Added to `build_flags` |
| `-DUSE_TINYUSB` | Pico/RP2040 boards only (in build_flags) | `-DUSE_TINYUSB` |
| `board_build.core` | RP2040/RP2350 only | `earlephilhower` |
| `board_build.filesystem_size` | RP2040/RP2350 only | `0.5m`, `8m` |

### Partition table selection guide

| Board type | Flash size | Partition table |
|---|---|---|
| LittleFS ESP32 (4MB flash) | 4MB | `min_spiffs.csv` |
| LittleFS ESP32 (8MB flash) | 8MB | `default_8MB.csv` |
| TinyUSB ESP32-S2 (4MB) | 4MB | `tinyuf2-partitions-4MB-noota.csv` |
| TinyUSB ESP32-S3 (4MB) | 4MB | `tinyuf2-partitions-4MB-noota.csv` |
| TinyUSB ESP32-S3 (8MB, no PSRAM) | 8MB | `tinyuf2-partitions-8MB.csv` |
| TinyUSB ESP32-S3 (16MB) | 16MB | `tinyuf2-partitions-16MB.csv` |

### RP2040/RP2350 template

```ini
[env:<board_env_name>]
extends = common:arduinopico
board = <platformio_board_id>
build_flags = -DUSE_TINYUSB
board_build.filesystem_size = 0.5m
; For Fruit Jam (RP2350 with onboard ESP32-C6):
; build_flags = -DUSE_TINYUSB -DARDUINO_ADAFRUIT_FRUITJAM_RP2350
; board_build.filesystem_size = 8m
; lib_ignore = WiFi, WiFi101, Adafruit Zero DMA Library
```

### SAMD template

```ini
[env:<board_env_name>]
extends = common:atsamd
board = <platformio_board_id>
build_flags = -DUSE_TINYUSB
              -D<BOARD_DEFINE>
extra_scripts = pre:rename_usb_config.py
```

---

## Step 3 — Network Interface Selection

**File:** `src/Wippersnapper_Networking.h`

This file selects the network driver via `#if defined(...)` preprocessor checks. In most cases,
**no changes are needed** — the board is identified by its architecture define which already maps
to the correct driver:

| Architecture | Auto-selected driver | Needs Wippersnapper_Networking.h change? |
|---|---|---|
| `ARDUINO_ARCH_ESP32` | `Wippersnapper_ESP32` | **No** — all ESP32-family boards are caught by the existing `#elif defined(ARDUINO_ARCH_ESP32)` |
| `ARDUINO_ARCH_ESP8266` | `Wippersnapper_ESP8266` | **No** |
| `ARDUINO_ARCH_RP2040` | `ws_networking_pico` | **No** — unless the board needs AIRLIFT (like Fruit Jam) |

**When changes ARE needed:**
- If the RP2040/RP2350 board uses an AirLift co-processor instead of built-in CYW43 WiFi
  (e.g. Fruit Jam), add its define to the AIRLIFT condition:

```cpp
#if defined(ADAFRUIT_METRO_M4_EXPRESS) ||                                      \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO) || defined(USE_AIRLIFT) ||            \
    defined(ARDUINO_ADAFRUIT_FRUITJAM_RP2350) ||                               \
    defined(ARDUINO_<YOUR_NEW_BOARD>)  // <-- add here
```

- If the board uses an entirely new networking chip with no existing driver, you would need to
  create a new network interface header in `src/network_interfaces/`. This is rare — see the
  existing four drivers for the required interface:
  - Must extend `Wippersnapper`
  - Must implement: `set_ssid_pass()`, `check_valid_ssid()`, `getMacAddr()`, `getRSSI()`,
    `setupMQTTClient()`, `networkStatus()`, `connectionType()`, `_connect()`, `_disconnect()`

> **In the vast majority of cases, no new network interface is needed.** All ESP32 variants
> (S2, S3, C3, C6, C5, H2, etc.) share the same `Wippersnapper_ESP32` driver.

---

## Step 4 — Example Sketches and Build Configurations

**Files:** `examples/Wippersnapper_demo/` and `examples/wippersnapper_debug/`

Each board needs build configuration files in these example directories. These are dot-files
that the CI `build_platform.py` script reads to know how to build for each board.

### Production build (Wippersnapper_demo)

Create a `.generate` file — this tells CI to build the production firmware for this board:

**File:** `examples/Wippersnapper_demo/.<board_ci_name>.generate`

The file is typically empty or contains build overrides. For most boards, it just needs to exist.
The `<board_ci_name>` corresponds to the CI matrix entry name (underscores, no `env:` prefix).

Look at the naming pattern in the existing `.generate` files — some use a `wippersnapper_`
prefix when the board name could collide with another CI context.

### Debug build (wippersnapper_debug) — optional

If the board has a debug variant in `platformio.ini`, also create:
- `examples/wippersnapper_debug/.<board_ci_name>.generate` (for the debug env)
- `examples/wippersnapper_debug/.<board_ci_name>.test.skip` (to skip test on production env)
- `examples/Wippersnapper_demo/.<board_ci_name>_debug.test.skip` (to skip demo on debug env)

---

## Step 5 — Add Board to CI Workflow Matrix

**File:** `.github/workflows/build-clang-doxy.yml`

Add the board's CI name to the correct build job matrix. Choose based on platform family and
provisioning method:

| Build job | For boards that... |
|---|---|
| `build-esp32sx-esptool` | ESP32-S2/S3 with TinyUSB — uses esptool for combined binary |
| `build-esp32sx` | ESP32-S3 with TinyUSB — produces UF2 + bin |
| `build-esp32` | ESP32/ESP32-C3/C6/C5 with LittleFS — produces combined binary via esptool |
| `build-samd` | SAMD51 boards — produces UF2 + hex |
| `build-rp2040` | RP2040/RP2350 boards — produces UF2 |
| `build-esp8266` | ESP8266 boards |

For LittleFS ESP32 boards (`build-esp32`), you may also need to add a bootloader offset in the
`include` section:
- ESP32 (original): `"0x1000"` (default)
- ESP32-C3, ESP32-C6: `"0x0"`
- ESP32-C5: `"0x2000"`

Example addition for the `build-esp32` job:
```yaml
matrix:
  arduino-platform:
    [
      # ... existing entries ...
      "new_board_name",
    ]
  include:
    - offset: "0x1000"    # default for ESP32
    - offset: "0x0"
      arduino-platform: "new_board_name"  # if ESP32-C6, for example
```

---

## Step 6 — Wippersnapper_Boards Definition (Companion Repo)

This step uses a separate repository: `https://github.com/adafruit/Wippersnapper_Boards`

The CI workflow checks out this repo and reads board definitions to determine flash parameters
for creating combined binaries.

### 6a. Fork and clone

```bash
gh repo fork adafruit/Wippersnapper_Boards --clone=true --remote-name upstream -- Wippersnapper_Boards
```

### 6b. Create board folder

```
Wippersnapper_Boards/boards/<board-id>/
└── definition.json
```

The `<board-id>` folder name uses **kebab-case** and must match the `BOARD_ID` from
`Wippersnapper_Boards.h`.

### 6c. Write definition.json

The schema is at `boards/schema.json` in the Wippersnapper_Boards repo. The **required** top-level
fields are: `boardName`, `mcuName`, `mcuRefVoltage`, `installMethod`, `displayName`, `productURL`,
`documentationURL`, `components`. Additional optional fields: `vendor`, `description`,
`bootloaderBoardName`, `bootDiskName`, `installBoardName`, `published`.

For **LittleFS ESP32 boards** (web installer with esptool):

```json
{
  "boardName": "<board-id>",
  "displayName": "<Human-Friendly Board Name>",
  "mcuName": "<mcu>",
  "mcuRefVoltage": <voltage>,
  "vendor": "Adafruit",
  "productURL": "https://www.adafruit.com/product/<ID>",
  "documentationURL": "https://learn.adafruit.com/<guide-slug>",
  "installBoardName": "<ci-env-name-if-different-from-boardName>",
  "installMethod": "web",
  "esptool": {
    "chip": "<esp32|esp32s2|esp32s3|esp32c3|esp32c6|esp32c5>",
    "flashMode": "dio",
    "flashFreq": "80m",
    "flashSize": "4MB",
    "offset": "<user-filesystem-partition-offset>",
    "fileSystemSize": <partition-size-bytes>,
    "blockSize": 4096,
    "structure": {
      "<bootloader-offset>": "wippersnapper.<installBoardName>.littlefs.VERSION.combined.bin"
    }
  },
  "components": { "digitalPins": [], "analogPins": [], "i2cPorts": [] }
}
```

For **TinyUSB ESP32-S2/S3 boards** (web-native-usb or uf2 installer):

```json
{
  "boardName": "<board-id>",
  "displayName": "<Human-Friendly Board Name>",
  "mcuName": "<mcu>",
  "mcuRefVoltage": <voltage>,
  "vendor": "Adafruit",
  "productURL": "https://www.adafruit.com/product/<ID>",
  "documentationURL": "https://learn.adafruit.com/<guide-slug>",
  "installMethod": "uf2",
  "bootloaderBoardName": "<tinyuf2-board-name>",
  "esptool": {
    "chip": "<esp32s2|esp32s3>",
    "flashMode": "dio",
    "flashFreq": "80m",
    "flashSize": "4MB",
    "offset": "<user-filesystem-partition-offset>",
    "fileSystemSize": <partition-size-bytes>,
    "blockSize": 4096,
    "structure": {
      "0x0": "wippersnapper.<boardName>.fatfs.VERSION.combined.bin"
    }
  },
  "components": { "digitalPins": [], "analogPins": [], "i2cPorts": [] }
}
```

For **RP2040/RP2350 boards** (UF2, no esptool):

```json
{
  "boardName": "<board-id>",
  "displayName": "<Human-Friendly Board Name>",
  "mcuName": "<MCU>",
  "mcuRefVoltage": <voltage>,
  "vendor": "Adafruit",
  "productURL": "https://www.adafruit.com/product/<ID>",
  "documentationURL": "https://learn.adafruit.com/<guide-slug>",
  "installMethod": "uf2",
  "bootDiskName": "RP2",
  "components": { "digitalPins": [], "analogPins": [], "i2cPorts": [] }
}
```

#### Top-level field notes

| Field | Required | Description |
|---|---|---|
| `boardName` | Yes | Kebab-case board ID (max 40 chars). Must match the folder name and `BOARD_ID` in firmware. |
| `displayName` | Yes | Human-friendly name shown in the Adafruit IO UI (max 50 chars). |
| `mcuName` | Yes | Microcontroller name, e.g. `"esp32c6"`, `"esp32s3"`, `"rp2040"` (max 24 chars). |
| `mcuRefVoltage` | Yes | MCU's max analog reference voltage in Volts (e.g. `3.3`, `2.6`, `1.1`). |
| `installMethod` | Yes | One of: `"web"` (LittleFS/esptool), `"uf2"` (TinyUSB/UF2), `"web-native-usb"` (TinyUSB via WebUSB), `"library"`, `"python"`. |
| `productURL` | Yes | Product page URL. |
| `documentationURL` | Yes | Learn guide URL. |
| `components` | Yes | Pin and I2C port definitions (see below). Required sub-objects: `digitalPins`, `analogPins`, `i2cPorts`. |
| `vendor` | No | Board manufacturer name (max 24 chars). |
| `description` | No | Board description (max 255 chars). |
| `bootloaderBoardName` | No | TinyUF2 board name for fetching bootloader releases (max 60 chars). Required for TinyUSB ESP32-S2/S3 boards. Check [tinyuf2 releases](https://github.com/adafruit/tinyuf2/releases). |
| `bootDiskName` | No | Disk name in bootloader mode, e.g. `"RP2"` (max 40 chars). Used for RP2040/RP2350. |
| `installBoardName` | No | Override for `boardName` when looking up firmware assets in CI releases. Use when the CI env name differs from `boardName`. |
| `published` | No | Boolean. When `false`, board won't appear in the firmware installer. |

#### esptool field notes (required for ESP32-family boards)

| Field | Required | Description |
|---|---|---|
| `chip` | Yes | Chip name for `esptool.py --chip` (e.g. `"esp32c6"`, `"esp32s3"`). |
| `offset` | Yes | User filesystem partition start offset as hex string (e.g. `"0x3D0000"`). Find this in the board's partition table CSV. |
| `fileSystemSize` | Yes | Size of the user filesystem partition in bytes (e.g. `131072` for 128KB, `983040` for 960KB). This is where `secrets.json` lives. |
| `blockSize` | Yes | Filesystem block size in bytes — typically `4096`. |
| `structure` | Yes | Object mapping flash offsets to firmware filenames. Key is the bootloader offset (e.g. `"0x0"` for ESP32-C3/C6, `"0x1000"` for original ESP32). |
| `flashMode` | No | Flash mode — typically `"dio"`, sometimes `"qio"`. |
| `flashFreq` | No | Flash frequency — typically `"80m"` for ESP32-S2/S3/C3/C6, `"40m"` for original ESP32. |
| `flashSize` | No | Total flash size — `"4MB"`, `"8MB"`, or `"16MB"`. |
| `baudRate` | No | Fixed baud rate for the web-serial installer (avoids auto-detection). |

#### components field notes

The `components` object must include `digitalPins`, `analogPins`, and `i2cPorts` arrays. See
existing board definitions for examples. Each digital pin needs `name`, `displayName`, `dataType`;
optionally `direction`, `hasPWM`, `hasServo`, `uartTx`/`uartRx`, `isHardwired`. Each analog pin
needs `name`, `displayName`, `dataType`; optionally `maxResolution`, `hasPWM`, `hasServo`,
`direction`, `isHardwired`. Each I2C port needs `i2cPortId`, `SDA`, `SCL`.

> **Tip:** Copy `components` from an existing board in the same family and adjust pin names/numbers
> to match the new board's pinout from its learn guide or schematic.

---

## Step 7 — Build and Verify

Check if windows powershell/cmd first by testing `echo $env:USERNAME %WINDIR%`

windows powershell users:
```powershell
. "$env:userprofile/.platformio/penv/Scripts/activate.ps1"
pio run -e <board_env_name>
```

bash/other users:
```bash
. ~/.platformio/penv/bin/activate.sh
pio run -e <board_env_name>
```

Fix any compilation errors. Common issues:
- **Unrecognized board define** — check `build_flags` matches the Arduino core's define exactly
- **Missing NeoPixel/LED pin** — verify `PIN_NEOPIXEL`, `NEOPIXEL_NUM`, or `LED_BUILTIN`
  exist in the board's variant header
- **Wrong partition table** — ensure the `.csv` file exists for the selected platform
- **Library conflicts** — some platforms need `lib_ignore` overrides (see existing RP2040 and
  SAMD entries for examples)

---

## Step 8 — Format with clang-format, ensure passes doxygen and other CI checks

```bash
clang-format -i src/Wippersnapper_Boards.h
```

Run on all modified `.h` and `.cpp` files in `src/`. The repo's `.clang-format` config is
applied automatically. Note: `src/provisioning/tinyusb/`, `src/nanopb/`, `src/wippersnapper/`,
and `src/pb.h` are excluded from clang checks by CI.

---

## Step 9 — Create Pull Requests

Two separate PRs are needed:

### PR 1: Wippersnapper_Boards
- Branch from main, add the board folder with `definition.json`
- PR title: `Add <BOARD> board definition`

### PR 2: Adafruit_Wippersnapper_Arduino
- Branch from main, include all firmware changes
- PR title: `Add <BOARD> board support`
- Reference the boards PR in the description
- List all files changed

---

## Step 10 — Test (if hardware available)

1. Build and upload firmware to the board:
   - TinyUSB boards: drag-and-drop the `.uf2` file
   - LittleFS boards: flash via esptool or web flasher
2. Provide credentials:
   - TinyUSB: drop `secrets.json` on the mounted drive
   - LittleFS: use the device provisioner at https://io.adafruit.com/devices/new
   - LittleFS (serial alternative): `pio run --target uploadfs -e <board_env_name>` uploads the
     filesystem image containing `secrets.json` directly via serial
3. Connect to Adafruit IO — the board should appear on the device page
4. Verify the board name and type show correctly in the UI
5. Test basic I/O: add a digital GPIO component, toggle a pin
6. Monitor serial output for initialization and connection success

### Alternative: Local testing with protomq

If a live Adafruit IO connection is not available, use **protomq**
(`https://github.com/lorennorman/protomq`) as a local MQTT broker that speaks protobufs and
can simulate the WipperSnapper broker.

1. **Clone and start protomq** on the `wippersnapper-v1` tag:
   ```bash
   git clone https://github.com/lorennorman/protomq.git
   cd protomq
   git checkout wippersnapper-v1
   npm install
   cp .env.example.json .env.json
   # Edit .env.json — set the local path to your WipperSnapper .proto files
   npm run import-protos
   npm run build-web
   npm start
   ```
2. **Configure `secrets.json`** on the board to point at the local broker:
   - Set `io_url` to your machine's local network IP (e.g. `192.168.1.100`)
   - Set `io_port` to `1883`
   - `io_key` and `io_username` can be left as-is or set to dummy values
3. **Flash firmware and monitor** serial output while the board connects to local protomq
4. **Inspect and drive WipperSnapper protocol messages:**
   - Use the **protomq web UI** (typically `http://localhost:5173/` — check the startup output)
     to see connected clients, subscriptions, and decoded protobuf messages in real time
   - Or use **play-scripts** (see the protomq README) to script automated protocol interactions
5. This lets you validate: connection handshake, board registration, component init, sensor
   readings, and pin control — all without a live Adafruit IO account

---

## Worked (abridged) Example: Adafruit QT Py ESP32-C6

The **QT Py ESP32-C6** is an Adafruit board with WiFi 6, Bluetooth 5.3, a RISC-V ESP32-C6,
4MB flash, no PSRAM, and a single NeoPixel. Product ID: 5928.

### Step 0 — Research

- **Product page:** `https://www.adafruit.com/product/5928`
- **Learn guide:** `https://learn.adafruit.com/adafruit-qt-py-esp32-c6`
- **MCU:** ESP32-C6 (RISC-V, single-core, 160 MHz)
- **WiFi:** Built-in WiFi 6 (802.11ax) — uses `Wippersnapper_ESP32` network interface
- **Flash:** 4MB, no PSRAM
- **Native USB:** No (USB-to-UART via CP2102N) → **LittleFS** provisioning
- **Status LED:** 1x NeoPixel on `PIN_NEOPIXEL`
- **PlatformIO board ID:** `adafruit_qtpy_esp32c6` (from PlatformIO registry)
- **Arduino define:** `ARDUINO_ADAFRUIT_QTPY_ESP32C6` (from ESP32 Arduino core boards.txt)
- **Closest existing board:** Adafruit Feather ESP32-C6 (`adafruit_feather_esp32c6_4mbflash_nopsram`)

### Step 1 — Wippersnapper_Boards.h

```cpp
// After the ARDUINO_ADAFRUIT_QTPY_ESP32C3 block:
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32C6)
#define BOARD_ID "qtpy-esp32c6"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
```

### Step 2 — platformio.ini

```ini
; Adafruit QT Py ESP32-C6
[env:adafruit_qtpy_esp32c6]
extends = common:esp32
board = adafruit_qtpy_esp32c6
build_flags =
    -DARDUINO_ADAFRUIT_QTPY_ESP32C6
    -DARDUINO_USB_CDC_ON_BOOT=1
board_build.filesystem = littlefs
board_build.partitions = min_spiffs.csv
```

### Step 3 — Network Interface

No changes needed. `ARDUINO_ARCH_ESP32` is defined by the ESP32 Arduino core, so
`Wippersnapper_Networking.h` already selects `Wippersnapper_ESP32`.

### Step 4 — Example build configs

Create empty files:
- `examples/Wippersnapper_demo/.qtpy_esp32c6.generate`
- `examples/wippersnapper_debug/.qtpy_esp32c6.test.skip`

### Step 5 — CI Workflow

Add `"qtpy_esp32c6"` to the `build-esp32` job matrix and add the bootloader offset:

```yaml
# In build-esp32 matrix:
arduino-platform:
  [
    # ... existing entries ...
    "qtpy_esp32c6",
  ]
include:
  - offset: "0x1000"
  - offset: "0x0"
    arduino-platform: "qtpy_esp32c6"  # ESP32-C6 uses 0x0 offset
```

### Step 6 — Wippersnapper_Boards definition

`Wippersnapper_Boards/boards/qtpy-esp32c6/definition.json`:

```json
{
  "boardName": "qtpy-esp32c6",
  "displayName": "Adafruit QT Py ESP32-C6",
  "mcuName": "esp32c6",
  "mcuRefVoltage": 1.1,
  "vendor": "Adafruit",
  "productURL": "https://www.adafruit.com/product/5928",
  "documentationURL": "https://learn.adafruit.com/adafruit-qt-py-esp32-c6",
  "installBoardName": "wippersnapper_qtpy_esp32c6",
  "installMethod": "web",
  "esptool": {
    "chip": "esp32c6",
    "flashMode": "dio",
    "flashFreq": "80m",
    "flashSize": "4MB",
    "offset": "0x3D0000",
    "fileSystemSize": 131072,
    "blockSize": 4096,
    "structure": {
      "0x0": "wippersnapper.wippersnapper_qtpy_esp32c6.littlefs.VERSION.combined.bin"
    }
  },
  "components": {
    "digitalPins": [],
    "analogPins": [],
    "i2cPorts": []
  }
}
```

> **Note:** The `components` arrays above are left empty for brevity. In the real definition,
> populate `digitalPins`, `analogPins`, and `i2cPorts` based on the board's pinout from its
> learn guide. See `boards/qtpy-esp32c3/definition.json` in the Wippersnapper_Boards repo for
> a complete QT Py example.

### Files changed

| File | Repo | Action |
|------|------|--------|
| `src/Wippersnapper_Boards.h` | Firmware | Modified — new `#elif` block |
| `platformio.ini` | Firmware | Modified — new `[env:adafruit_qtpy_esp32c6]` |
| `examples/Wippersnapper_demo/.qtpy_esp32c6.generate` | Firmware | New — empty CI build config |
| `examples/wippersnapper_debug/.qtpy_esp32c6.test.skip` | Firmware | New — skip debug test |
| `.github/workflows/build-clang-doxy.yml` | Firmware | Modified — added to `build-esp32` matrix |
| `Wippersnapper_Boards/boards/qtpy-esp32c6/definition.json` | Boards | New |
