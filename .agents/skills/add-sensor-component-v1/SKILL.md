---
name: add-sensor-component-v1
description: >
  Guides adding a new I2C sensor component to Adafruit IO WipperSnapper v1 firmware and the
  Wippersnapper_Components definition repo. Use this skill whenever the user wants to add a new
  sensor, create a WipperSnapper driver, register an I2C device, or contribute a component to
  WipperSnapper — even if they just mention a sensor name and "wippersnapper" in the same breath.
  Covers the full workflow: driver code, registration, component definition JSON, library deps,
  build verification, clang-format, and PR creation for both repos.
postRun: >
  Automatically use run_in_terminal to execute `clang-format -i` on the modified files, 
  and run  `doxygen` (e.g., `doxygen Doxyfile` or manually) to validate documentation blocks.
  Review the terminal output, identify any warnings or errors on the modified files, 
  and apply fixes before continuing. You can pip install clang-format 20 like GH runners (venv).
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

Fetch the `.md?view=all` single page version (not subpage) if you need more detail on any step — particularly for the products learn guide, along with the
Wippersnapper_Components repo setup, image requirements, and testing in Adafruit IO.

## Arguments

This skill accepts a sensor name as its argument (e.g. `/add-sensor-component-v1 TMP119`).

Ideally the user will provide the datasheet, product purchase url, wiki / learn guide url, and arduino driver library name. If not, the skill will research these based on the sensor name, attempting to load the adafruit product page to then find the almost-always-present learn guide link which mentions the needed arduino library (GH repo link and example code sketch link in GH).

## Environment Check (optional, do not block on this)

If Bash is available, quickly check connectivity to adapt your approach. **If Bash is not
available or these commands fail, skip this section and proceed — use WebFetch/WebSearch for
research, or playwright if necessary, and do the code changes with the file tools you have. 
The user can handle git/PRs. NOTE: Do not leave files in wippersnapper_components as untracked - instead if unable to fork / PR components repo then attach image to summary and component definition**

```bash
curl -s -o /dev/null -w "%{http_code}" https://www.bbc.com       # general web access
curl -s -o /dev/null -w "%{http_code}" https://api.github.com     # GitHub API (needed for gh)
git ls-remote https://github.com/adafruit/Wippersnapper_Components HEAD  # git clone access
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

**Adafruit_Wippersnapper_Arduino:**
- **clang-format** — code formatting must match `.clang-format` config. Run `clang-format -i` on all changed files.
- **Doxygen** — all public/protected methods need Doxygen-style `/*! @brief ... */` comment blocks. CI will fail if these are missing or malformed. Follow the existing driver style exactly.
- **Build** — firmware must compile for all target boards in `platformio.ini`.

**Wippersnapper_Components:**
- **JSON schema validation** — `definition.json` must conform to `schema.json` in the repo root.
- **Image validation** — dimensions, file size, and format are checked.

---

## Step 0 — Research the Sensor **MUST BE DONE BEFORE WRITING ANY CODE**

Before writing any code, gather this information:

| What | Where to look |
|------|---------------|
| Product page | Search the web for "adafruit <SENSOR>" to find the product page. The product page (`https://www.adafruit.com/product/<PRODUCT_ID>`) html data has links to the learn guide (do not use the API or guess URLs). Use a tool like wget or curl: `curl -sL https://www.adafruit.com/product/5817 \| grep "learn.adafruit.com"` (Note: replace `5817` with the actual product ID found via search). |
| Learn guide | **Always read the learn guide before writing any code.** Fetch the text view `.md?view=all` version of the guide found on the product page (e.g., `curl -sL "https://learn.adafruit.com/<guide-slug>.md?view=all"`). The Arduino section (`## Arduino`) names the exact library to use. Do NOT skip this step even if you think you recognise a sub-component. There is also a `## Example Code` section showing the basic example sketch. |
| Adafruit Arduino library | Named in the learn guide. Or: `gh search repos "<SENSOR>" --owner adafruit`. If no dedicated library from adafruit, check related chips (e.g. TMP119 lives inside `Adafruit_TMP117`). Try partial matches like `TMP11` if exact fails. Fall back to 3rd party or ask user. |
| Library API | Read the library header on GitHub — find `begin()` signature and sensor read methods (`getEvent`, `readTempC`, etc.) |
| I2C addresses | Sensor datasheet or Adafruit product page or learn guide or driver. Check https://learn.adafruit.com/i2c-addresses/the-list |
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
return cached data. **Cache `millis()` once** at the top of the function (e.g.
`unsigned long now = millis();`) and use the cached value for both the time guard check and
setting `_lastRead` — never call `millis()` twice.
See the driver template in Step 1 and `WipperSnapper_I2C_Driver_SCD30.h` and SGP30 
for the canonical patterns.

This is especially important for multi-reading sensors but also applies to temperature sensors
where both °C and °F subcomponents are enabled.

---

## Step 1 — Create the Driver Header

**File:** `src/components/i2c/drivers/WipperSnapper_I2C_Driver_<SENSOR>.h`

### First: Read the library's example sketch — MANDATORY before writing any code

**Do NOT assume the library API based on other sensors.** Every library is different. You must
read the actual example code to know the real API. Guessing from similar sensors (e.g. assuming
SCD30-style `getEvent()` for an STCC4) will produce a driver that does not compile.

Before writing any driver code, find and read the library's `simple test` or `basic_usage` or all examples not using interrupts (data ready flags are okay)
on GitHub. Check all matches for suitable usage suggestions. This is your source of truth for how the sensor is meant to be used:

```bash
gh api repos/adafruit/<Library_Repo>/contents/examples --jq '.[].name'
# then read the .ino file for the simpletest/basic_test/singleshot example
```

**Fallback routes** (try in order if `gh`/Bash is unavailable):

1. **WebFetch raw GitHub:** `https://raw.githubusercontent.com/adafruit/<Library_Repo>/main/examples/<example>/<example>.ino`  (you may need to access the web version (not raw) to view default branch and example related folder structure).
2. **WebFetch learn guide:** `https://learn.adafruit.com/<guide-slug>.md?view=all` — the Arduino
   section usually contains example code link showing the exact API (or embedded code if non-markdown version).
3. **Ask the user:** If tools are restricted, ask them to paste the library header and example.

From the example, extract the **exact method signatures** used:
- `begin()` — what arguments, what return type
- How readings are triggered (`getEvent()`, `readMeasurement()`, `measureSingleShot()`, etc.)
- What the return values look like (Unified Sensor `sensors_event_t`? Raw floats? uint16_t pointers?)
- Any required setup calls (continuous mode, conditioning, etc.)
- Any delays or polling (`dataReady()`, fixed delays between reads)

### Then: Read the library header — MANDATORY

Read the `.h` file to see **all public methods and their exact signatures**. This is essential
because:
- The example may only show one usage pattern; the header shows everything available
- You need the exact types (float vs uint16_t, pointer args vs return values)
- You need to identify default configuration set in `begin()`/`_init()`

```bash
gh api repos/adafruit/<Library_Repo>/contents/<Library_Name>.h --jq '.content' | base64 -d
```

Or via WebFetch: `https://raw.githubusercontent.com/adafruit/<Library_Repo>/main/<Library_Name>.h`

If GitHub is blocked, the learn guide `.md?view=all` may contain enough API detail from code
snippets. If not, ask the user for the header content.

**Explicitly set every configuration parameter that the library defaults in `begin()`/`_init()`.**
This pins behavior so library updates can't silently change WipperSnapper.

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

### Then: Write the driver

This is a header-only class. Use the closest existing driver as a template. Always apply the patterns from this skill!

> **Good References:** 
> - Look at `WipperSnapper_I2C_Driver_SCD30.h` for a clean example of the new style, including read caching and setting explicit defaults in `begin()`.
> - Check `WipperSnapper_I2C_Driver_SGP30.h` for a complete example of `fastTick()` usage, which is needed when the datasheet explicitly requires a minimum polling cadence for correct operation.

### Key decisions when writing the driver:

- **Handling `dataReady()` or Data Availability:** Some sensors (like the SCD30) require you to check a flag before reading to avoid stale data or blocked I2C buses. 
  - If the library requires polling a `dataReady()` method, implement a short, limited test in your read loop. 
  - Never use infinite `while (!sensor.dataReady()) delay(10);` loops. Since WipperSnapper runs many components concurrently, infinite loops crash the whole system if a sensor hangs. 
  - Instead, use a brief check (optionally with a small finite `delay` and subsequent retry, like `if(!dataReady) { delay(100); if(!dataReady) return false; }`). See `WipperSnapper_I2C_Driver_SCD30.h` for a safe implementation.

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
  ~1 Hz polling to maintain its IAQ algorithm.

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

You can usually grab the product image url details from the Adafruit product API (http://www.adafruit.com/api/product/<pid>) and resize it [4:3] and compress it  (using compressjpeg.com) to pass CI validation. Ideally pick the straight-on product shot with a plain background, not angled or lifestyle images. Include a small amount of dead space around the product so it doesn't get cropped in the UI (go with existing examples for reference).

---

## Step 6 — Build and Verify
Check if windows powershell/cmd first by testing `echo $env:USERNAME %WINDIR%`

windows powershell users:
```powershell
. "$env:userprofile/.platformio/penv/Scripts/activate.ps1"
pio run -e adafruit_feather_esp32s3
```

bash/other users:
```bash
# often the env needs sourcing first
. ~/.platformio/penv/bin/activate.sh

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

Two separate PRs are needed (mention the model used for the PRs in the description):

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

## Worked (abridged) Example: TMP119 - Do not follow this example step-by-step, it's only an abbreviated demonstration of the above process

TMP119: TI high-accuracy temperature sensor, TMP117 variant (chip ID 0x2117 vs 0x0117).

### Step 0 — Research

- **Search Process:** Web search "adafruit TMP119" → product page https://www.adafruit.com/product/6482
  which links to the learn guide https://learn.adafruit.com/adafruit-tmp119-high-precision-temperature-sensor
  which shows the Arduino library is `Adafruit_TMP117`. Alternatively,
  `gh search repos "Adafruit TMP119" --owner adafruit` only finds the PCB repo — no dedicated
  library. Trying `TMP11` finds arduino lib, or checking `Adafruit_TMP117` repo contents reveals
  `Adafruit_TMP119.h/.cpp` — TMP119 inherits from TMP117.
- **Library:** `Adafruit_TMP117` (contains `Adafruit_TMP119` class)
- **Product URL:** `https://www.adafruit.com/product/6482`
- **Docs URL:** `https://learn.adafruit.com/adafruit-tmp119-high-precision-temperature-sensor`
- **Markdown learn guide:** `https://learn.adafruit.com/adafruit-tmp119-high-precision-temperature-sensor.md?view=all` (mentions `## Arduino` section with driver repo link `https://github.com/adafruit/Adafruit_TMP117` and `## Example Code` using `https://github.com/adafruit/Adafruit_TMP117/blob/master/examples/TMP119_basic_test/TMP119_basic_test.ino`.)
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
    unsigned long now = millis();
    if (_lastRead != 0 && now - _lastRead < 1000)
      return true; // recently read, use cached value
    if (!_tmp119->getEvent(&_cachedTemp))
      return false;
    _lastRead = now;
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
