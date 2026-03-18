---
name: add_sensor_component_v1
description: >
  Guides adding a new I2C sensor component to Adafruit IO WipperSnapper v1 firmware and the
  Wippersnapper_Components definition repo. Use this skill whenever the user wants to add a new
  sensor, create a WipperSnapper driver, register an I2C device, or contribute a component to
  WipperSnapper — even if they just mention a sensor name and "wippersnapper" in the same breath.
  Covers the full workflow: driver code, registration, component definition JSON, library deps,
  build verification, clang-format, and PR creation for both repos.
---

# Add I2C Sensor Component to WipperSnapper v1

Changes span **two repositories**:
1. **Adafruit_Wippersnapper_Arduino** — C++ firmware: new driver + registration
2. **Wippersnapper_Components** — JSON definition + product image

The user supplies a sensor name (e.g. "TMP119"). Research the sensor, find the Adafruit Arduino
library, identify the closest existing driver, and walk through every file change.

### Naming convention

- **PascalCase** for C++: class `WipperSnapper_I2C_Driver_TMP119`, file
  `WipperSnapper_I2C_Driver_TMP119.h`, pointer `_tmp119`
- **lowercase** for component folder and `strcmp` string: `tmp119`

Decide the canonical name in Step 0 and use it everywhere.

> **Proto files are off-limits.** Only Adafruit staff modify `.proto` files.

## Reference

The official Adafruit guide for this process:
- Human-readable (single page): https://learn.adafruit.com/how-to-add-a-new-component-to-adafruit-io-wippersnapper?view=all
- Machine-readable markdown: https://learn.adafruit.com/how-to-add-a-new-component-to-adafruit-io-wippersnapper.md?view=all

Fetch the `.md?view=all` version if you need more detail on any step — particularly for the
Wippersnapper_Components repo setup, image requirements, and testing in Adafruit IO.

## Arguments

This skill accepts a sensor name as its argument (e.g. `/add_sensor_component_v1 TMP119`).

## Environment Check

Before starting, determine connectivity level — this affects whether you can use `gh` commands
or must fall back to plain `git`:

```bash
curl -s -o /dev/null -w "%{http_code}" https://www.bbc.com       # general web access
curl -s -o /dev/null -w "%{http_code}" https://api.github.com     # GitHub API (needed for gh)
git ls-remote https://github.com/adafruit/Wippersnapper_Components HEAD  # git clone access
gh auth status  # see if cli/token/login present

```

| Result | Capability |
|--------|-----------|
| All 3 succeed | Full access — use `gh` for forking, PRs, API queries |
| BBC fails, GitHub API works | Restricted web but `gh` works — skip web fetches for product pages |
| BBC + API fail, git works | Git-only — use `git clone`/`git push` instead of `gh`, create PRs manually via browser |
| All fail | Offline — can only write code, user must handle git/PRs |

## CI Checks

PRs to both repos run CI. Key checks to pass before submitting:

**Adafruit_Wippersnapper_Arduino:**
- **clang-format** — code formatting must match `.clang-format` config. Run `clang-format -i` on all changed files.
- **Doxygen** — all public/protected methods need Doxygen-style `/*! @brief ... */` comment blocks. CI will fail if these are missing or malformed. Follow the existing driver style exactly.
- **Build** — firmware must compile for all target boards in `platformio.ini`.

**Wippersnapper_Components:**
- **JSON schema validation** — `definition.json` must conform to `schema.json` in the repo root.
- **Image validation** — dimensions, file size, and format are checked.

---

## Step 0 — Research the Sensor

Before writing any code, gather this information:

| What | Where to look |
|------|---------------|
| Product page | Search the web for "adafruit <SENSOR>" to find the product page. The product page links to the learn guide and Arduino library. This is the fastest route to all other info. |
| Adafruit Arduino library | The learn guide (linked from product page) shows which library to use. Or: `gh search repos "<SENSOR>" --owner adafruit`. If no dedicated library, check related chips (e.g. TMP119 lives inside `Adafruit_TMP117`). Try partial matches like `TMP11` if exact fails. |
| Library API | Read the library header on GitHub — find `begin()` signature and sensor read methods (`getEvent`, `readTempC`, etc.) |
| I2C addresses | Sensor datasheet or Adafruit product page. Check https://learn.adafruit.com/i2c-addresses/the-list |
| What it measures | Datasheet — map each reading to a subcomponent type (see table below) |
| Closest existing driver | Browse `src/components/i2c/drivers/` for a sensor in the same family or with identical reading types |
| Documentation URL | Prefer: Adafruit learn guide (from product page) > manufacturer datasheet. Non-Adafruit products are accepted — use the manufacturer's product/datasheet URL. Note: third-party domain URLs may initially fail CI URL validation until a maintainer adds the domain to the allowlist. |

### Subcomponent type reference

Valid `subcomponents` values in `definition.json`, mapping 1:1 to base driver `getEvent*()` methods:

| Subcomponent | getEvent method | `sensors_event_t` field | SI unit |
|---|---|---|---|
| `ambient-temp` | `getEventAmbientTemp` | `.temperature` | °C |
| `ambient-temp-fahrenheit` | `getEventAmbientTempF` | `.temperature` | °F |
| `humidity` | `getEventRelativeHumidity` | `.relative_humidity` | %RH |
| `pressure` | `getEventPressure` | `.pressure` | hPa |
| `altitude` | `getEventAltitude` | `.altitude` | m |
| `co2` | `getEventCO2` | `.CO2` | ppm |
| `eco2` | `getEventECO2` | `.eCO2` | ppm |
| `tvoc` | `getEventTVOC` | `.tvoc` | ppb |
| `gas-resistance` | `getEventGasResistance` | `.gas_resistance` | Ω |
| `light` | `getEventLight` | `.light` | lux |
| `proximity` | `getEventProximity` | `.data[0]` | unitless |
| `voltage` | `getEventVoltage` | `.voltage` | V |
| `current` | `getEventCurrent` | `.current` | A |
| `raw` | `getEventRaw` | `.data[0]` | unitless |
| `pm10-std` | `getEventPM10_STD` | `.data[0]` | µg/m³ |
| `pm25-std` | `getEventPM25_STD` | `.data[0]` | µg/m³ |
| `pm100-std` | `getEventPM100_STD` | `.data[0]` | µg/m³ |
| `unitless-percent` | `getEventUnitlessPercent` | `.data[0]` | % |
| `object-temp` | `getEventObjectTemp` | `.temperature` | °C |
| `object-temp-fahrenheit` | `getEventObjectTempF` | `.temperature` | °F |

When using raw reads (not Unified Sensor `getEvent()`), assign to the correct field above, e.g.
`tempEvent->temperature = _sensor->readTempC();`

Temperature sensors almost always include both `ambient-temp` and `ambient-temp-fahrenheit`.

**Fahrenheit:** `getEventAmbientTempF` and `getEventObjectTempF` are in the base class — they
call the Celsius method and convert. Only implement the Celsius version. Never implement °F.

**Read-and-cache requirement:** The calling order of `getEvent*()` methods is not guaranteed (°F
may be called before °C). Every `getEvent*()` must go through a shared `_readSensor()` with a
millis-based time guard so only the first call per cycle does the I2C read; subsequent calls
return cached data. See the driver template in Step 1 and `WipperSnapper_I2C_Driver_SCD30.h` and SGP30 
for the canonical patterns.

This is especially important for multi-reading sensors but also applies to temperature sensors
where both °C and °F subcomponents are enabled.

---

## Step 1 — Create the Driver Header

**File:** `src/components/i2c/drivers/WipperSnapper_I2C_Driver_<SENSOR>.h`

### First: Read the library's example sketch

Before writing any driver code, find and read the library's `simpletest` or `basic_test` or some example not using interrupts (data ready flags are okay)
on GitHub. Check all matches for suitable usage suggestions. This is your source of truth for how the sensor is meant to be used:

```bash
gh api repos/adafruit/<Library_Repo>/contents/examples --jq '.[].name'
# then read the relevant .ino file
```

Study the example for:
- Which `begin()` overload is used and what arguments it takes
- Any setup calls after `begin()` (mode, averaging, resolution, range, etc.)
- Whether `dataReady()` is polled before reading
- How readings are obtained (`getEvent()` vs `readTempC()` vs `read()` etc.)
- Any delays or timing requirements

### Then: Read the library's source code

Also read the library's `begin()` / `_init()` implementation to see what default configuration
it applies — measurement mode, averaging count, resolution, conversion time, etc. These defaults
are often not visible in the example sketch but they affect sensor behavior. If a future library
update changes any of these defaults, it would silently change WipperSnapper's behavior too.

When writing the WipperSnapper driver, **explicitly set every configuration parameter that the
library sets as a default in its `begin()` or `_init()`** chain. This pins the behavior so that library
updates cannot break WipperSnapper without a deliberate driver change on our side.

For example, if the library's `_init()` sets continuous mode with 8x averaging as defaults:
```cpp
bool begin() {
    _sensor = new Adafruit_Sensor();
    if (!_sensor->begin((uint8_t)_sensorAddress, _i2c))
      return false;
    _sensor->setMeasurementMode(CONTINUOUS);  // explicit, was implicit default
    _sensor->setAveragedSampleCount(AVERAGE_8X);  // explicit, was implicit default
    return true;
}
```

### Then: Write the driver using this template

This is a header-only class. Use the closest existing driver as a template, but **do not blindly
copy** — older drivers may lack the caching and explicit-defaults patterns described above.
Always apply the patterns from this skill even if the closest driver doesn't use them.

> **Note:** Some existing drivers (e.g. TMP117, MCP9808) use simpler patterns that predate
> current best practices. Follow this template, not those older drivers.

```cpp
/*!
 * @file WipperSnapper_I2C_Driver_<SENSOR>.h
 *
 * Device driver for the <SENSOR> <description> sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) <AUTHOR> <YEAR> for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_<SENSOR>_H
#define WipperSnapper_I2C_Driver_<SENSOR>_H

#include "WipperSnapper_I2C_Driver.h"
#include <<Adafruit_Library_Header>.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a <SENSOR> sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_<SENSOR> : public WipperSnapper_I2C_Driver {
public:
  WipperSnapper_I2C_Driver_<SENSOR>(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  ~WipperSnapper_I2C_Driver_<SENSOR>() { delete _<sensor_ptr>; }

  bool begin() {
    _<sensor_ptr> = new <Adafruit_Class>();
    if (!_<sensor_ptr>->begin((uint8_t)_sensorAddress, _i2c))
      return false;
    // Pin library defaults explicitly (found by reading library _init/begin):
    // _<sensor_ptr>->setMeasurementMode(...);
    // _<sensor_ptr>->setAveragedSampleCount(...);
    return true;
  }

  // --- One getEvent*() per sensor reading type ---
  // All go through _readSensor() for caching

  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!_readSensor())
      return false;
    *tempEvent = _cachedTemp;
    return true;
  }

protected:
  <Adafruit_Class> *_<sensor_ptr>; ///< Pointer to <SENSOR> sensor object

  // Cached readings and time guard
  sensors_event_t _cachedTemp = {0};
  unsigned long _lastRead = 0;

  /*******************************************************************************/
  /*!
      @brief    Reads sensor data, with 1-second cache to avoid redundant
                I2C reads when multiple getEvent*() calls occur per cycle
                (e.g. °C then °F, or temp then humidity). The calling order
                of getEvent methods is not guaranteed, so every method must
                go through this function.
      @returns  True if cached data is available or a fresh read succeeded.
  */
  /*******************************************************************************/
  bool _readSensor() {
    if (_lastRead != 0 && millis() - _lastRead < 1000)
      return true; // use cached values
    // Do actual I2C read — adapt to library API:
    if (!_<sensor_ptr>->getEvent(&_cachedTemp))
      return false;
    _lastRead = millis();
    return true;
  }
};

#endif // WipperSnapper_I2C_Driver_<SENSOR>_H
```

### Key decisions when writing the driver:

- **Library API style:** Some Adafruit libraries use Unified Sensor (`getEvent(sensors_event_t*)`)
  which fills the event struct directly. Others expose raw read methods like `readTempC()`. If the
  library uses raw reads, assign to the appropriate field (e.g. `tempEvent->temperature = readTempC()`)
  and return `true`, or return `false` if the read fails.

- **Multiple sensor types:** If the sensor measures more than one thing (e.g. BME280 does temp +
  humidity + pressure + altitude), implement a `getEvent*()` for each. Each usually needs its own
  `Adafruit_Sensor*` pointer obtained from the library (e.g. `_bme->getTemperatureSensor()`).

- **begin() signature:** Check the library header — some take `(address, wire)`, others take
  `(wire)` with address set separately. Match exactly.

- **Minimum polling interval (fastTick):** Check the sensor datasheet for a recommended or
  required minimum polling period. Some sensors (like gas sensors) need to be read at a fixed
  cadence to keep their internal algorithms running correctly — even when WipperSnapper's publish
  interval is much longer. If the datasheet specifies a required polling interval, you need the
  `fastTick()` pattern:

  1. Override `fastTick()` — this is called once per main loop iteration by the I2C component
     manager, so it runs much more frequently than the publish interval.
  2. Use a `millis()`-based guard to enforce the datasheet cadence (non-blocking, no `delay()`).
  3. Cache the latest reading in member variables.
  4. Have `getEvent*()` return the cached values instead of doing a fresh I2C read.

  See `WipperSnapper_I2C_Driver_SGP30.h` for a complete example — the SGP30 datasheet requires
  ~1 Hz polling to maintain its IAQ algorithm, so the driver defines:

  ```cpp
  #define SGP30_FASTTICK_INTERVAL_MS 1000

  void fastTick() override {
      if (!_sgp30) return;
      uint32_t now = millis();
      if (now - _lastFastMs >= SGP30_FASTTICK_INTERVAL_MS) {
          if (_sgp30->IAQmeasure()) {
              _eco2 = (uint16_t)_sgp30->eCO2;
              _tvoc = (uint16_t)_sgp30->TVOC;
          }
          _lastFastMs = now;
      }
  }

  bool getEventECO2(sensors_event_t *senseEvent) override {
      senseEvent->eCO2 = _eco2;  // return cached value
      return true;
  }
  ```

  Most simple sensors (temperature, pressure, humidity) do NOT need this — they can be read
  on-demand at whatever interval WipperSnapper requests. Only use `fastTick()` when the
  datasheet explicitly requires a minimum polling cadence for correct operation.

---

## Step 2 — Register in WipperSnapper_I2C.h

**File:** `src/components/i2c/WipperSnapper_I2C.h`

Two changes, both in **alphabetical order** among existing entries:

```cpp
// With other driver includes:
#include "drivers/WipperSnapper_I2C_Driver_<SENSOR>.h"

// In private section:
WipperSnapper_I2C_Driver_<SENSOR> *_<sensor> = nullptr;
```

---

## Step 3 — Add Initialization in WipperSnapper_I2C.cpp

**File:** `src/components/i2c/WipperSnapper_I2C.cpp`

Find the `initI2CDevice()` method. Add a new `else if` block in **alphabetical order** by device
name string. The device name **must exactly match** the folder name you'll create in the
Wippersnapper_Components repo.

```cpp
} else if (strcmp("<sensor_name>", msgDeviceInitReq->i2c_device_name) == 0) {
    _<sensor> = new WipperSnapper_I2C_Driver_<SENSOR>(this->_i2c, i2cAddress);
    if (!_<sensor>->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize <SENSOR>!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _<sensor>->configureDriver(msgDeviceInitReq);
    drivers.push_back(_<sensor>);
    WS_DEBUG_PRINTLN("<SENSOR> Initialized Successfully!");
}
```

`configureDriver()` is inherited — reads sensor periods from the init request. Never reimplement.

---

## Step 4 — Add Library Dependencies in library.properties and platformio.ini

Skip if the sensor class lives in an already-listed library (e.g. TMP119 in Adafruit_TMP117).

### 4a. platformio.ini

Add to `[env]` `lib_deps` in **alphabetical order** ignoring any purpose specific sections of the lib_deps:
```ini
    adafruit/Adafruit <Library Name>              # released Arduino library
    https://github.com/<owner>/<repo>.git         # unreleased / third-party
```

### 4b. library.properties

Add the library's **Arduino Library Manager name** to the comma-separated `depends=` line. This
is the name as it appears in the Arduino IDE Library Manager + library.properties, not the GitHub repo name. Add it in
a logical position among the existing entries.

Example: to add a library called "Adafruit FooBar":
```
depends=..., Adafruit FooBar, ...
```
GitHub-only libraries can't go in `library.properties` — note this in the PR description (and advise to fork n release ardu lib).

---

## Step 5 — Wippersnapper_Components Definition

This step uses a separate repository: `https://github.com/adafruit/Wippersnapper_Components`

### 5a. Fork and clone

Clone inside the firmware repo (it's in `.gitignore`):
```bash
# If using a specific GitHub account:
gh auth switch --user <username>

# Fork and clone into the firmware repo root:
gh repo fork adafruit/Wippersnapper_Components --clone=true --remote-name upstream -- Wippersnapper_Components

### 5b. Create component folder

```
Wippersnapper_Components/components/i2c/<sensor_name>/
├── definition.json
└── image.jpg (or .png, .gif, .svg)
```

The `<sensor_name>` folder name is lowercase and must exactly match the string used in
`strcmp()` in Step 3.

### 5c. Write definition.json

```json
{
  "displayName": "<Sensor Display Name>",
  "vendor": "<Vendor Name>",
  "productURL": "https://www.adafruit.com/product/<ID>",
  "documentationURL": "https://learn.adafruit.com/<guide-slug>",
  "published": false,
  "i2cAddresses": ["0xNN"],
  "subcomponents": ["ambient-temp", "ambient-temp-fahrenheit"]
}
```

Field notes:
- `published` — always `false` for new contributions. Adafruit sets it to `true` after release.
- `productURL` — Adafruit product page if available, place of sale, otherwise the manufacturer's product page.
- `documentationURL` — Adafruit learn guide (preferred), or manufacturer docs page / wiki, or datasheet URL as
  fallback. Third-party domain URLs may initially fail CI URL validation until a maintainer adds
  the domain to the allowlist — note this in the PR if using a non-Adafruit/non-TI URL.
- `i2cAddresses` — hex strings, all addresses the chip can use (check datasheet for ADDR pin
  configurations).
- `subcomponents` — mixed array of simple strings or objects. Use exact type strings from the Step 0 table.

### Subcomponent formats

**Simple format** — when the sensor type name is self-explanatory:
```json
"subcomponents": ["ambient-temp", "ambient-temp-fahrenheit", "pressure"]
```

**Object format** — when you need a custom display name for clarity:
```json
"subcomponents": [
  { "displayName": "Ambient Light", "sensorType": "light" },
  { "displayName": "UV Count", "sensorType": "raw" }
]
```

Use objects when:
1. **Type name is ambiguous** — "light" could mean visible, UV, or IR. `displayName` clarifies in the UI.
2. **Two readings share the same physical type** — v1 schema forbids duplicate `sensorType`. Use
   `"raw"` for the second with a descriptive `displayName` (e.g. LTR-329: `"light"` for ambient,
   `"raw"` with `displayName: "Infrared"` for IR).

Examples:
- **LTR-390** (UV + light): `[{"displayName": "Ambient Light", "sensorType": "light"}, {"displayName": "UV Count", "sensorType": "raw"}]`
- **LTR-329** (visible + IR): `[{"displayName": "Ambient Light", "sensorType": "light"}, {"displayName": "Infrared", "sensorType": "raw"}]`
- **INA219** (no ambiguity): `["voltage", "current"]`

When using `"raw"` as a stand-in, the driver must implement `getEventRaw()` for that reading.
3. **Non-standard units** — if the sensor reports in a non-SI unit (or doesn't match Adafruit_Sensor type SI unit) then use the appropriate unitless type or raw with a descriptive `displayName` including units.
4. **Clarity compared to the auto UI labels or between subcomponents** — compare other components using the same types for reference.

### 5d. Add product image

Requirements:
- **Dimensions:** 400px × 300px (4:3 ratio)
- **File size:** 3 KB – 100 KB
- **Formats:** jpg, jpeg, gif, png, svg
- **Filename:** `image.<ext>`

You can usually grab the product image from the Adafruit product API (http://www.adafruit.com/api/product/[pid]) and resize it [4:3] and compress it  (using compressjpeg.com) to pass CI validation. Ideally pick the straight-on product shot with a plain background, not angled or lifestyle images. Include a small amount of dead space around the product so it doesn't get cropped in the UI (go with existing examples for reference).

---

## Step 6 — Build and Verify

```bash
# PlatformIO build for a common target
pio run -e adafruit_feather_esp32s3
```

Fix any compilation errors. Common issues:
- Wrong `begin()` signature — check the library header
- Missing include — verify the `#include` path is correct
- Library not found — verify `platformio.ini` `lib_deps` entry

---

## Step 7 — Format with clang-format, ensure passes doxygen and other CI checks

```bash
clang-format -i src/components/i2c/drivers/WipperSnapper_I2C_Driver_<SENSOR>.h
```

Run on all modified files. The repo's `.clang-format` should be applied.

Ensure all doxygen style is consistent with existing drivers. 

---

## Step 8 — Create Pull Requests

Two separate PRs are needed:

### PR 1: Wippersnapper_Components
- Branch from main, add the component folder with `definition.json` and `image`
- PR title: `Add <SENSOR> component definition`
- Wait for CI checks to pass

### PR 2: Adafruit_Wippersnapper_Arduino
- Branch from main, include all firmware changes (driver, registration, platformio.ini)
- PR title: `Add <SENSOR> I2C driver`
- Reference the components PR in the description
- See https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/pull/228 for a good example PR

---

## Step 9 — Test (if hardware available)

1. Upload firmware to a WipperSnapper-compatible board
2. Go to Adafruit IO device page → New Component → check "Show Dev"
3. The new component appears with "In Development" badge
4. Configure I2C address and reading period
5. Monitor serial output for initialization success
6. Verify sensor readings appear on the device page

---

## Worked Example: TMP119

TMP119: TI high-accuracy temperature sensor, TMP117 variant (chip ID 0x2117 vs 0x0117).

### Step 0 — Research

- **Search:** Web search "adafruit TMP119" → product page https://www.adafruit.com/product/6482
  which links to the learn guide https://learn.adafruit.com/adafruit-tmp119-high-precision-temperature-sensor
  which shows the Arduino library is `Adafruit_TMP117`. Alternatively,
  `gh search repos "Adafruit TMP119" --owner adafruit` only finds the PCB repo — no dedicated
  library. Trying `TMP11` or checking `Adafruit_TMP117` repo contents reveals
  `Adafruit_TMP119.h/.cpp` — TMP119 inherits from TMP117.
- **Library:** `Adafruit_TMP117` (contains `Adafruit_TMP119` class)
- **Product URL:** `https://www.adafruit.com/product/6482`
- **Docs URL:** `https://learn.adafruit.com/adafruit-tmp119-high-precision-temperature-sensor`
- **I2C addresses:** 0x48, 0x49, 0x4A, 0x4B (same as TMP117, datasheet Table 7-1)
- **Measures:** Temperature only → subcomponents: `ambient-temp`, `ambient-temp-fahrenheit`
- **Closest driver:** `WipperSnapper_I2C_Driver_TMP117.h`

### Step 1 — Read the example, then the library source

**Example** (`examples/TMP119_basic_test/TMP119_basic_test.ino`):
```cpp
Adafruit_TMP119 tmp11x;
tmp11x.begin();                    // default addr 0x48, Wire
while (!tmp11x.dataReady()) delay(10);  // polls data-ready flag
tmp11x.getEvent(&temp);           // fills sensors_event_t
```

**Library source** (`Adafruit_TMP117.h` / `Adafruit_TMP119.cpp`):
- `_init()` calls `reset()` which restores factory defaults:
  - Continuous conversion mode (`TMP117_MODE_CONTINUOUS`)
  - 8x averaging (`TMP117_AVERAGE_8X`)
  - 1000ms conversion delay (`TMP117_DELAY_1000_MS`)
- `getEvent()` internally calls `waitForData()` which blocks until `dataReady()` is true
- `begin(addr, wire)` — address first, wire second

**Decisions:**
- The library's `getEvent()` handles data-ready blocking internally, so no `fastTick()`.
- Explicitly set mode and averaging in `begin()` to pin the defaults.
- Only implement `getEventAmbientTemp` (Celsius) — the base class handles °F conversion.
- Since the calling order of °C and °F methods is not guaranteed, use a shared read-and-cache
  pattern with a time guard so only the first call per cycle hits the I2C bus:

```cpp
protected:
  Adafruit_TMP119 *_tmp119;
  sensors_event_t _cachedTemp = {0};
  unsigned long _lastRead = 0;

  bool _readSensor() {
    if (_lastRead != 0 && millis() - _lastRead < 1000)
      return true; // recently read, use cached value
    if (!_tmp119->getEvent(&_cachedTemp))
      return false;
    _lastRead = millis();
    return true;
  }

public:
  bool begin() {
    _tmp119 = new Adafruit_TMP119();
    if (!_tmp119->begin((uint8_t)_sensorAddress, _i2c))
      return false;
    // Pin defaults explicitly — library reset() sets these, but we don't
    // want a future library change to silently alter WipperSnapper behavior
    _tmp119->setMeasurementMode(TMP117_MODE_CONTINUOUS);
    _tmp119->setAveragedSampleCount(TMP117_AVERAGE_8X);
    return true;
  }

  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!_readSensor())
      return false;
    *tempEvent = _cachedTemp;
    return true;
  }
  // getEventAmbientTempF is inherited — it calls getEventAmbientTemp and converts
```

### Step 2 — Register in WipperSnapper_I2C.h

```cpp
// After TMP117 include (alphabetical)
#include "drivers/WipperSnapper_I2C_Driver_TMP119.h"

// In private section, after _tmp117
WipperSnapper_I2C_Driver_TMP119 *_tmp119 = nullptr;
```

### Step 3 — Init block in WipperSnapper_I2C.cpp

```cpp
// After the tmp117 block, before tsl2591 (alphabetical)
} else if (strcmp("tmp119", msgDeviceInitReq->i2c_device_name) == 0) {
    _tmp119 = new WipperSnapper_I2C_Driver_TMP119(this->_i2c, i2cAddress);
    if (!_tmp119->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize TMP119!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _tmp119->configureDriver(msgDeviceInitReq);
    drivers.push_back(_tmp119);
    WS_DEBUG_PRINTLN("TMP119 Initialized Successfully!");
}
```

### Step 4 — Library dependency

`platformio.ini` already has `adafruit/Adafruit TMP117` and `library.properties` already has
`Adafruit TMP117` — no changes needed since TMP119 lives in that package.

### Step 5 — Component definition

`Wippersnapper_Components/components/i2c/tmp119/definition.json`:
```json
{
  "displayName": "TMP119",
  "vendor": "Texas Instruments",
  "productURL": "https://www.adafruit.com/product/6482",
  "documentationURL": "https://learn.adafruit.com/adafruit-tmp119-high-precision-temperature-sensor",
  "published": false,
  "i2cAddresses": ["0x48", "0x49", "0x4A", "0x4B"],
  "subcomponents": ["ambient-temp", "ambient-temp-fahrenheit"]
}
```

Simple strings — unambiguous for a temperature-only sensor. Image: product API, 400x300, compress.

### Files changed

| File | Action |
|------|--------|
| `src/components/i2c/drivers/WipperSnapper_I2C_Driver_TMP119.h` | New — driver with explicit mode/averaging config |
| `src/components/i2c/WipperSnapper_I2C.h` | Modified — include + private pointer |
| `src/components/i2c/WipperSnapper_I2C.cpp` | Modified — init block in `initI2CDevice()` |
| `platformio.ini` | No change — TMP117 library already listed |
| `library.properties` | No change — TMP117 library already listed |
| `Wippersnapper_Components/components/i2c/tmp119/definition.json` | New |
| `Wippersnapper_Components/components/i2c/tmp119/image.jpg` | New |
