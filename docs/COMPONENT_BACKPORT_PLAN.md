# Component Backport Plan: `migrate-api-v2`

Bring the **API v2** firmware (`migrate-api-v2`) in line with the full component
set that already exists in:

- **v1 / `main`** (`upstream/main`) — the canonical, complete driver set using the
  legacy `WipperSnapper_I2C_Driver_*` class style.
- **`offline-mode`** (pre-v2 alpha) — already uses the **v2 `drv*` driver style**
  and the `drvBase` interface, so most of its drivers port across with little/no
  change.

Working branch: **`migrate-api-v2-backport-components`** (cut from `migrate-api-v2`).

---

## 1. How a driver is wired in API v2

A v2 I2C driver is a header-only class in `src/components/i2c/drivers/` that
inherits `drvBase`:

1. **Header** `drvXxx.h` — class `drvXxx : public drvBase`, with a constructor
   forwarding to `drvBase`, a `begin()` override, and one `getEventYyy()` override
   per metric.
2. **Registration (include)** — add `#include "drivers/drvXxx.h"` to
   [`src/components/i2c/controller.h`](../src/components/i2c/controller.h).
3. **Registration (factory)** — add an entry to the `I2cFactory` map in
   [`src/components/i2c/controller.cpp`](../src/components/i2c/controller.cpp),
   keyed by the component's string id (matches the folder name in the
   `Wippersnapper_Components` repo, e.g. `"sgp30"`).
4. **Library dependency** — ensure the Arduino library is listed in
   `library.properties` **and** `platformio.ini` (most are already present).

### Key difference from `offline-mode` / v1

- **No `ConfigureDefaultSensorTypes()` in v2.** In v2 the enabled sensor types
  come from the cloud/offline config via `drvBase::EnableSensorReads()`; drivers
  only implement `getEventXxx()` handlers. When porting an `offline-mode` driver,
  **drop** the `ConfigureDefaultSensorTypes()` override.
- **Fahrenheit is free.** `drvBase::getEventAmbientTempF()` /
  `getEventObjectTempF()` default to converting the Celsius handler — implement
  only the Celsius `getEventAmbientTemp()` / `getEventObjectTemp()`.
- Metric handlers write into the matching `sensors_event_t` field
  (`temperature`, `relative_humidity`, `eCO2`, `tvoc`, `voc_index`, `nox_index`,
  `CO2`, `lux`, `voltage`, `current`, `pressure`, `altitude`, `data[0]` for raw…).
  The `SensorEventHandlers` map in `drvBase.h` routes each
  `SENSOR_TYPE_*` to its handler.

### Note on "custom settings"

API v2 has begun adding support for **per-device custom settings** (user-tunable
properties such as gain, integration time, measurement mode, averaging, etc.).
Until the proto/config plumbing for arbitrary properties is finalised, each new
driver below should carry an inline comment block enumerating the custom
properties a user would plausibly want to tune, so they are easy to surface later
(see the existing `TODO` in `drvVl6180x.h` re: gain). Those candidate properties
are listed per-sensor in the tables below.

---

## 2. Missing components (date-ordered, oldest first)

Date = first commit that introduced the driver in `upstream/main`. "Source"
indicates the easiest port origin: **offline** = a v2-style `drv*` already exists
in `offline-mode`; **v1-only** = must be freshly converted from the legacy
`WipperSnapper_I2C_Driver_*` style.

| # | Date | Sensor | Driver class | Arduino library | Metrics (sensor types) | Source | Candidate custom settings |
|---|------|--------|--------------|-----------------|------------------------|--------|---------------------------|
| 1 | 2023-07-14 | **SGP30** | `drvSgp30` | `Adafruit SGP30 Sensor` | eCO2, TVOC | offline | Baseline calibration (eCO2/TVOC baseline words), humidity compensation (abs. humidity from a paired RH/T sensor), measurement interval |
| 2 | 2023-09-27 | **HTU31D** | `drvHtu31d` | `Adafruit HTU31D Library` | ambient-temp (+°F), humidity | offline | Heater on/off, measurement resolution (temp/humidity OSR) |
| 3 | 2024-08-23 | **HDC302x** | `drvHdc302x` | `Adafruit HDC302x` | ambient-temp (+°F), humidity | offline | Auto-measurement rate, heater mode/power, offset/threshold alerts |
| 4 | 2024-12-19 | **VCNL4200** | `drvVncl4200` | `Adafruit VCNL4200` | light (lux), proximity | offline | ALS integration time, ALS gain, proximity LED current/duty, proximity integration time |
| 5 | 2024-12-23 | **SEN6X / SEN66** | `drvSen6x` | `Sensirion I2C SEN66` | temp (+°F), humidity, PM1.0/2.5/10 (std), VOC index, NOx index, CO2 | offline | Temp offset, ambient pressure / altitude (CO2 comp.), VOC/NOx algorithm tuning, fan auto-clean interval |
| 6 | 2025-02-19 | **LPS28DFW** | `drvLps28dfw` | `Adafruit LPS28` | ambient-temp (+°F), pressure | offline | Full-scale range (1260 vs 4060 hPa), output data rate, averaging, low-pass filter |
| 7 | 2025-04-04 | **INA260** | `drvIna260` | `Adafruit INA260` | voltage, current (+power) | offline | Averaging count, V/I conversion times, operating/trigger mode |
| 8 | 2025-06-29 | **INA237** | `drvIna237` | `Adafruit INA237` | voltage, current | offline | Shunt resistance & max current (calibration), ADC range, averaging, conversion time |
| 9 | 2025-06-29 | **INA238** | `drvIna238` | `Adafruit INA238` | voltage, current | offline | Shunt resistance & max current (calibration), ADC range, averaging, conversion time |
| 10 | 2025-07-02 | **D6T-1A** | `drvD6t1a` | `OmronD6T` | ambient-temp (+°F), object-temp (+°F) | offline | (fixed-function; emissivity/averaging if exposed by lib) |
| 11 | 2025-08-13 | **INA228** | `drvIna228` | `Adafruit INA228` | voltage, current (+power, energy, charge) | offline | Shunt resistance & max current (calibration), ADC range, averaging, conversion time |
| 12 | 2025-08-20 | **MLX90632** | `drvMlx90632` | `Adafruit MLX90632` | ambient-temp (+°F), object-temp (+°F) | offline | Emissivity, measurement mode (medical/extended range), refresh rate |
| 13 | 2025-08-21 | **AS5600** | `drvAs5600` | `Adafruit AS5600` | raw (angle) | offline | Angle range / zero (start/stop position), output stage, slow filter / fast-filter threshold, hysteresis |
| 14 | 2025-08-26 | **QMC5883P** | `drvQmc5883p` | `Adafruit QMC5883P` | raw (magnetic magnitude, gauss) | offline | Full-scale range (gauss), output data rate, oversampling, set/reset mode |
| 15 | 2025-08-27 | **BMP5xx** | `drvBmp5xx` | `Adafruit BMP5xx` | ambient-temp (+°F), pressure, altitude | v1-only | Pressure/temp oversampling, IIR filter coefficient, output data rate, power mode |
| 16 | 2025-09-24 | **SPA06-003** | `drvSpa06_003` | `Adafruit SPA06_003` | ambient-temp (+°F), pressure | v1-only | Pressure/temp oversampling, measurement rate, FIFO/IIR if exposed |
| 17 | 2026-02-20 | **SGP41** | `drvSgp41` | `Adafruit SGP41` | VOC index, NOx index, raw | v1-only | Humidity compensation (RH/T), conditioning duration, sampling interval |
| 18 | 2026-03-24 | **APDS9999** | `drvApds9999` | `Adafruit APDS9999` | light (lux), proximity | v1-only | ALS gain, ALS integration/measurement rate, proximity LED current/pulses, proximity gain |
| 19 | 2026-03-24 | **TMP119** | `drvTmp119` | `Adafruit TMP119` | ambient-temp (+°F) | v1-only | Averaging (conversion averaging), conversion cycle time, one-shot vs continuous |
| 20 | 2026-04-09 | **AS7331** | `drvAs7331` | `Adafruit AS7331` | raw (UVA/UVB/UVC) | v1-only | Gain, integration time, measurement mode (cont/cmd/sync), divider |
| 21 | 2026-04-14 | **STCC4** | `drvStcc4` | `Adafruit STCC4` | CO2, ambient-temp, humidity | v1-only | Ambient pressure / RH-T compensation, measurement mode, ASC (auto self-cal) |
| 22 | 2026-04-14 | **VCNL4030** | `drvVcnl4030` | `Adafruit VCNL4030` | light (lux), proximity | v1-only | ALS integration time, ALS gain, proximity LED current, proximity integration time |
| 23 | 2026-04-15 | **MAX44009** | `drvMax44009` | `Adafruit MAX44009` | light (lux) | v1-only | Integration time, manual vs auto mode, current division ratio |

### Also missing (non-sensor / out of immediate scope)

- **I2C output components** (v1 `*_Out*`, offline `drvOut*`): generic I2C output,
  7-segment, quad alphanumeric, char LCD, SSD1306, SH1107. The output base class
  (`drvOutputBase`) and the v2 output controller path are required first.
- **Displays** (`displays-v2` branch / v1 `display/drivers`): TFT/EPD/SSD1306
  display drivers — tracked separately on the `displays-v2` work.
- **ENS161 variant** — v2 has `drvEns160`; v1 `ENS16X` also covers the ENS161.
  Verify the ENS161 product id maps to the existing driver.

These are listed for completeness; the sensor backport above is the primary track.

---

## 3. Library dependency status

`library.properties` and `platformio.ini` on `migrate-api-v2` already declare the
libraries for the **offline-sourced** sensors (SGP30, HTU31D, etc. — verified for
items 1–2). For each **v1-only** sensor (items 15–23) confirm the corresponding
`Adafruit <X>` library is added to **both** manifests before enabling its build.

---

## 4. Execution order

1. ✅ Plan (this document).
2. **Port offline-sourced sensors first** (items 1–14) — lowest risk, drivers
   already exist in `drvBase` form; mechanical edits + drop
   `ConfigureDefaultSensorTypes()`, fix any metric-field mismatches (e.g. SGP30
   TVOC must use `event->tvoc`, not `voc_index`).
3. **Convert v1-only sensors** (items 15–23) from the legacy class style.
4. Add **custom-settings comment blocks** to every new driver (table column 9).
5. Outputs/displays as a follow-up track.

### Progress

- ✅ Items 1–12 ported and registered (SGP30, HTU31D, HDC302x, VCNL4200,
  SEN66/SEN6X, LPS28DFW, INA260, INA237, INA238, D6T-1A, INA228, MLX90632),
  with library deps added to `platformio.ini` + `library.properties` and all
  new files clang-format clean.
- ⏳ Items 13–23 (QMC5883P, BMP5xx, SPA06-003, SGP41, APDS9999, TMP119, AS7331,
  STCC4, VCNL4030, MAX44009) remain — items 15–23 are v1-only conversions.

### Notes discovered during porting

- v2's existing VCNL keys are misspelled `vncl4020` / `vncl4040` while the
  component ids are `vcnl4020` / `vcnl4040`. VCNL4200 is registered under both
  `vcnl4200` (correct) and `vncl4200` (legacy convention) to be safe.
- `offline-mode` shipped `drvSen6x` / `drvVncl4200` / `drvLps28dfw` headers that
  were never wired into its factory, so they were previously uncompiled; minor
  fixes were applied during the port (standard integer types, `isnan()` checks,
  `_lastRead` init).
