---
name: hil-display-arduino
description: "Run a real-hardware (HIL) display/sequence proof for a WipperSnapper *Arduino firmware* board via the usbip-hil-controller's firmware-bench path — flash a combined image to a real MCU, confirm a v2 broker check-in, inject a display Add/Write (and/or other components) over the broker, and capture a camera proof of the lit panel. Use when adding/verifying on-board display support (i8080/SPI ST7789, ILI9341, SSD1306, EPD…) on a real Arduino board (ESP32-S3 etc.), driving it through .github/scripts/hil_display_test.py with a spec under .github/scripts/specs/, and surfacing serial/protomq/command logs + a visual proof in the PR. This is the MCU FIRMWARE path — NOT the python-snapper/Blinka path (that is the controller's hil-display-pytest skill), and NOT for authoring a brand-new controller stage (see the controller's hil-author-test skill)."
---

# hil-display-arduino

Prove a WipperSnapper **Arduino firmware** display works on real hardware and
attach the evidence (camera image + logs) to the PR. The device under test is a
real MCU on a HIL bench; the controller flashes it, stands up a broker, drives a
test sequence, and a CSI camera photographs the lit panel.

The whole test is **data**: a JSON spec describes the sequence, and
[`.github/scripts/hil_display_test.py`](../../../.github/scripts/hil_display_test.py)
turns it into a controller `firmware-bench` job. Adding a board/display/flow
means writing a spec, not editing the driver.

## The shape of a run

1. **Build + merge firmware.** `pio run -e <board_env>` then merge with the
   board's tinyuf2 bootloader so the FATFS provisioning volume persists secrets:
   `esptool merge_bin -o combined.bin 0x0 bootloader.bin 0x8000 partition-table.bin
   0xe000 boot_app0.bin 0x10000 firmware.bin 0x410000 tinyuf2.bin`. The
   `0xe000 boot_app0.bin` is essential — it marks ota_0 active so the board boots
   WipperSnapper, not the tinyuf2 factory app. (See
   [`.github/workflows/hil-lilygo-display.yml`](../../../.github/workflows/hil-lilygo-display.yml)
   for the exact build+merge steps.)
2. **Write/choose a spec** under `.github/scripts/specs/`. See
   `specs/lilygo_t_display_s3.json` as the template.
3. **Run the driver** (needs `HIL_API_BASE`+`HIL_API_TOKEN`; the runner must reach
   the controller's LAN IP — use a self-hosted bench runner):
   ```bash
   python3 .github/scripts/hil_display_test.py \
     --combined combined.bin \
     --spec .github/scripts/specs/<board>.json \
     --out-dir hil-out
   ```
4. **Read the proof** in `hil-out/`: `hil-report.md` (the PR report),
   `hil-run.log` (controller command transcript), `serial.log`, `protomq.log`,
   and the capture JPEG. Exit 0 ⇔ check-in OK + every inject published + every
   capture produced an image.

## Spec format

```jsonc
{
  "device": "mcu-<board>-hil006",       // controller device id
  "uid": "<boardId><macUID>",           // WS device uid (the broker topic suffix)
  "camera_url": "http://rpi-hil006:8080/",
  "window_minutes": 12,
  "sequence": [
    { "step": "display_add", "interface": "i8080", "name": "tft", "driver": "ST7789",
      "data_pins": ["D39","D40","D41","D42","D45","D46","D47","D48"],
      "cs": "D6", "dc": "D7", "rst": "D5",
      "width": 320, "height": 170, "rotation": 1, "text_size": 2, "status_bar": true },
    { "step": "display_write", "name": "tft", "message": "line1\nline2" },
    { "step": "capture", "exposure_us": 32000, "gain": 3.0,
      "roi": [1270,770,235,135,2304,1296], "out": "/tmp/hil-display-capture.jpg" }
  ]
}
```

Step kinds (each takes optional `settle_s`):
- **`display_add`** — `interface: i8080` uses the controller's
  `display_add_i8080` builder; for SPI or others pass an explicit controller
  `kind` or a raw `payload_hex`.
- **`display_write`** — `{name, message}`; the driver encodes the
  `ws.signal` → `ws.display.Write` protobuf locally.
- **`component_add`** / **`inject`** — any `ws.signal.BrokerToDevice`. Give a
  controller builder `kind` + `params` (e.g. add an I2C sensor between display
  writes for a fuller test, once a builder exists) **or** raw `payload_hex`.
- **`capture`** — camera proof: `exposure_us`, `gain`, `roi`
  `[x,y,w,h,frame_w,frame_h]`, `focus_lock`, `white_balance`, `out`.

The driver always prepends the flash/secrets/check-in prologue
(`enter_bootloader → erase → flash → power_cycle → write_secrets_msc →
power_cycle → verify_checkin`); set `"skip_flash": true` to re-use an
already-flashed board.

## Camera proof gotchas (hard-won)

- A **bright self-lit TFT on a dark bench** is crushed to near-black by
  auto-exposure. Use a long manual exposure: a 170×320 i8080 ST7789 reads well at
  **`exposure_us: 32000, gain: 3.0`** (6000/1.0 came out black).
- **Lock focus.** `focus_lock` (default on) pins the autofocus-converged dioptre
  before the grab — continuous AF drifts mid-still and blurs text.
- **White balance is post-only.** The camera-server ignores `awb`/`colour_gains`;
  the `capture_display` stage white-patch-balances off the lit text.
- **ROI is frame-relative.** `?full=1` returns the sensor-native frame (4608×2592
  on the rpi-hil006 imx708) while ROIs are calibrated against the warm 2304×1296
  frame — always include `frame_w,frame_h` in the ROI so it scales.

## CI / PR report

The driver's logging **conforms to the `hil-test-suite.yml` convention** (PR
#930's `hil-lib.sh`): it polls the job to terminal so firmware-bench's
auto-registered `serial.log`/`protomq.log`/`flash.log` (`kind=log`) assets and
the capture (`kind=file`) are downloadable, saves them as
`hil-out/<target>-<label>-<type>.log`, and appends a section to a shared
`hil-out/comment.md`:

- a result row (check-in / injects / capture),
- the **visual proof** image (hosted on the `hil-proof` release),
- a **serial.log** proof window (from boot to just after the matched evidence
  phrase) and a **protomq.log** window **time-aligned** to it (shared UTC-ms
  clock), exactly like `append_proof`,
- an invocation + submitted-stages block (process + args + the controller plan).

`hil-lilygo-display.yml` (workflow_dispatch) builds+merges, runs the driver,
uploads per-type log artifacts, hosts the proof image, and — when given a
`pr_number` — posts `comment.md` as a new PR comment per run. Point `spec` at
any spec to reuse it for another board. When the #930 suite reaches
`migrate-api-v2`, this driver drops into `hil-test-suite.yml` as one more test
step (it already emits the suite's `comment.md` + log-naming convention).

## Related skills (in the controller repo, `usbip-hil-controller`)

These live in the bench controller repo (separate repo); reference them when the
work crosses into controller territory. If you have that repo checked out, you
can `@`-import the file; otherwise read it there.

- **`hil-author-test`** (`.claude/skills/hil-author-test/SKILL.md`) — authoring a
  `firmware-bench` test + stages (this skill's controller-side counterpart; its
  "Visual verification" section documents the `capture_display` stage).
- **`hil-display-pytest`** (`.claude/skills/hil-display-pytest/SKILL.md`) — the
  **python-snapper/Blinka** display path (host runs WipperSnapper-Python against
  a real panel), for when the DUT is an SBC rather than an MCU.
