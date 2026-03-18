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

You are guiding the user through adding a new I2C sensor to Adafruit IO WipperSnapper. This
involves changes in **two repositories**:

1. **Adafruit_Wippersnapper_Arduino** — C++ firmware: new driver + registration
2. **Wippersnapper_Components** — JSON definition + product image

The user will supply a sensor name (e.g. "TMP119"). Your job is to research the sensor, identify
the right Adafruit Arduino library, find the closest existing driver as a template, and then walk
through every file change needed.

> **Proto files are off-limits.** Contributors never touch `.proto` files — only Adafruit staff
> modify those. The existing `SensorType` enum already covers all common readings.

## Reference

The official Adafruit guide for this process:
- Human-readable (single page): https://learn.adafruit.com/how-to-add-a-new-component-to-adafruit-io-wippersnapper?view=all
- Machine-readable markdown: https://learn.adafruit.com/how-to-add-a-new-component-to-adafruit-io-wippersnapper.md?view=all

Fetch the `.md?view=all` version if you need more detail on any step — particularly for the
Wippersnapper_Components repo setup, image requirements, and testing in Adafruit IO.

## Arguments

This skill accepts a sensor name as its argument (e.g. `/add_sensor_component_v1 TMP119`).

---

## Step 0 — Research the Sensor

Before writing any code, gather this information:

| What | Where to look |
|------|---------------|
| Sensor's Adafruit Arduino library | `gh search repos "Adafruit <SENSOR>" --owner adafruit` or check if it lives inside another library (e.g. TMP119 is in `Adafruit_TMP117`) |
| Library API | Read the library header on GitHub — find `begin()` signature and sensor read methods (`getEvent`, `readTempC`, etc.) |
| I2C addresses | Sensor datasheet or Adafruit product page. Check https://learn.adafruit.com/i2c-addresses/the-list |
| What the sensor measures | Datasheet — temperature? humidity? pressure? Map each to a subcomponent type (see list below) |
| Closest existing driver | Browse `src/components/i2c/drivers/` for a sensor in the same family or with identical reading types |
| Adafruit product URL | `https://www.adafruit.com/product/<ID>` |

### Subcomponent type reference

These are the valid values for `subcomponents` in `definition.json` and map 1:1 to `getEvent*()`
methods in the base driver class:

| Subcomponent | getEvent method | SI unit |
|---|---|---|
| `ambient-temp` | `getEventAmbientTemp` | °C |
| `ambient-temp-fahrenheit` | `getEventAmbientTempF` | °F |
| `humidity` | `getEventRelativeHumidity` | %RH |
| `pressure` | `getEventPressure` | hPa |
| `altitude` | `getEventAltitude` | m |
| `co2` | `getEventCO2` | ppm |
| `eco2` | `getEventECO2` | ppm |
| `tvoc` | `getEventTVOC` | ppb |
| `gas-resistance` | `getEventGasResistance` | Ω |
| `light` | `getEventLight` | lux |
| `proximity` | `getEventProximity` | unitless |
| `voltage` | `getEventVoltage` | V |
| `current` | `getEventCurrent` | A |
| `raw` | `getEventRaw` | unitless |
| `pm10-std` | `getEventPM10_STD` | µg/m³ |
| `pm25-std` | `getEventPM25_STD` | µg/m³ |
| `pm100-std` | `getEventPM100_STD` | µg/m³ |
| `unitless-percent` | `getEventUnitlessPercent` | % |
| `object-temp` | `getEventObjectTemp` | °C |
| `object-temp-fahrenheit` | `getEventObjectTempF` | °F |

Temperature sensors almost always include both `ambient-temp` and `ambient-temp-fahrenheit`.

---

## Step 1 — Create the Driver Header

**File:** `src/components/i2c/drivers/WipperSnapper_I2C_Driver_<SENSOR>.h`

### First: Read the library's example sketch

Before writing any driver code, find and read the library's `simpletest` or `basic_test` or some example not using interrupts (data ready flags are okay)
on GitHub. This is your source of truth for how the sensor is meant to be used:

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
library sets as a default in its `begin()` or `_init()`**. This pins the behavior so that library
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

This is a header-only class. Use the closest existing driver as a template. The pattern is:

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
  /*******************************************************************************/
  /*!
      @brief    Constructor for a <SENSOR> sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_<SENSOR>(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a <SENSOR> sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_<SENSOR>() {
    delete _<sensor_ptr>;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the <SENSOR> sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _<sensor_ptr> = new <Adafruit_Class>();
    return _<sensor_ptr>->begin((uint8_t)_sensorAddress, _i2c);
  }

  // --- One getEvent*() per sensor reading type ---

  /*******************************************************************************/
  /*!
      @brief    Gets the <SENSOR>'s current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // Use the library's getEvent() if it fills sensors_event_t directly,
    // otherwise set tempEvent->temperature = _<sensor_ptr>->readTempC();
    return _<sensor_ptr>->getEvent(tempEvent);
  }

protected:
  <Adafruit_Class> *_<sensor_ptr>; ///< Pointer to <SENSOR> sensor object
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

1. Add the include near the top, with the other driver includes:
   ```cpp
   #include "drivers/WipperSnapper_I2C_Driver_<SENSOR>.h"
   ```

2. Add a private member pointer:
   ```cpp
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

`configureDriver()` is inherited from the base class — it reads the sensor periods from the init
request message. You never need to implement it yourself.

---

## Step 4 — Add Library Dependencies

Two files need updating when adding a new library. If the sensor class lives inside an existing
library that's already listed (like TMP119 inside Adafruit_TMP117), skip this step entirely.

### 4a. platformio.ini

In the `[env]` section's `lib_deps`, add the library in **alphabetical order** among existing
entries. The format depends on whether the library is published on the Arduino Library Manager:

**Released Arduino library** (most Adafruit libraries):
```ini
    adafruit/Adafruit <Library Name>
```

**Unreleased / third-party library** (use full GitHub URL):
```ini
    https://github.com/<owner>/<repo>.git
```

Examples from the current file:
```ini
    adafruit/Adafruit TMP117
    https://github.com/Sensirion/arduino-i2c-scd4x.git
    https://github.com/tyeth/omron-devhub_d6t-arduino.git
```

### 4b. library.properties

Add the library's **Arduino Library Manager name** to the comma-separated `depends=` line. This
is the name as it appears in the Arduino IDE Library Manager, not the GitHub repo name. Add it in
a logical position among the existing entries.

Example: to add a library called "Adafruit FooBar":
```
depends=..., Adafruit FooBar, ...
```

For non-Arduino-Library-Manager libraries (GitHub-only), they won't be in `library.properties`
since the Arduino IDE can't auto-install them — note this in the PR description.

---

## Step 5 — Wippersnapper_Components Definition

This step uses a separate repository: `https://github.com/adafruit/Wippersnapper_Components`

### 5a. Fork and clone

If the user needs to fork:
```bash
gh repo fork adafruit/Wippersnapper_Components --clone=false --remote-name origin
gh repo clone <username>/Wippersnapper_Components -- Wippersnapper_Components
```

If they want to use a specific GitHub account (e.g. `tyeth-ai-assisted`), switch with:
```bash
gh auth switch --user <username>
```

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
- `i2cAddresses` — hex strings, all addresses the chip can use (check datasheet for ADDR pin
  configurations).
- `subcomponents` — can be either simple strings or objects. Use the exact type strings from the
  table in Step 0.

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

Use the object format when:
1. **The sensor type name is ambiguous** — e.g. "light" could mean visible, UV, or IR. Give it a
   descriptive `displayName` so the user knows what they're looking at in the Adafruit IO UI.
2. **The sensor reports two readings of the same physical type** — the v1 schema forbids
   duplicate `sensorType` values in one component. Use `"raw"` as the sensorType for the second
   reading and set `displayName` to describe what it actually is. For example, the LTR-329 reports
   both ambient light and infrared light — both are "light", but the definition uses `"light"` for
   ambient and `"raw"` for infrared with `displayName: "Infrared"`.

Real examples:
- **LTR-390** (UV + light): `[{"displayName": "Ambient Light", "sensorType": "light"}, {"displayName": "UV Count", "sensorType": "raw"}]`
- **LTR-329** (visible + IR): `[{"displayName": "Ambient Light", "sensorType": "light"}, {"displayName": "Infrared", "sensorType": "raw"}]`
- **INA219** (no ambiguity): `["voltage", "current"]`

When using `"raw"` as a stand-in, the corresponding driver must implement `getEventRaw()` to
return that second reading.

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

## Step 7 — Format with clang-format

```bash
clang-format -i src/components/i2c/drivers/WipperSnapper_I2C_Driver_<SENSOR>.h
```

Also run on any other modified files. The repo's `.clang-format` config will be used
automatically.

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

The TMP119 is a high-accuracy temperature sensor from Texas Instruments. It's a variant of the
TMP117 with a different chip ID (0x2117 vs 0x0117).

- **Library:** `Adafruit_TMP117` (contains both `Adafruit_TMP117` and `Adafruit_TMP119` classes)
- **API:** `begin(uint8_t addr, TwoWire *wire)`, `getEvent(sensors_event_t *)`
- **I2C addresses:** 0x48, 0x49, 0x4A, 0x4B
- **Measures:** Temperature (ambient-temp, ambient-temp-fahrenheit)
- **Closest driver:** `WipperSnapper_I2C_Driver_TMP117.h`
- **platformio.ini:** Already has `adafruit/Adafruit TMP117` — no change needed

### Files changed:

1. **New:** `src/components/i2c/drivers/WipperSnapper_I2C_Driver_TMP119.h`
2. **Modified:** `src/components/i2c/WipperSnapper_I2C.h` (include + pointer)
3. **Modified:** `src/components/i2c/WipperSnapper_I2C.cpp` (init block)
4. **New (components repo):** `components/i2c/tmp119/definition.json`
5. **New (components repo):** `components/i2c/tmp119/image.jpg`
