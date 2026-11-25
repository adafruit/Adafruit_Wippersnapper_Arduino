---
name: bump-esp32-bsp
description: >
  Bumps the ESP32 Arduino BSP (espressif/arduino-esp32 / pioarduino) that WipperSnapper builds
  against, across every place that pins it: the Adafruit arduino-esp32 fork branch, adafruit/ci-arduino's
  all_platforms.py, the pioarduino platform pin in platformio.ini (v1 main and v2 migrate-api-v2),
  per-example .boards.local.txt overrides, adafruit/Wippersnapper_Boards partition offsets, and the
  CI_ARDUINO_REF used by the v1/v2 workflows for test runs. Includes the mandatory WiFi country table
  sync check against the new ESP-IDF esp_wifi.h. Use this skill whenever the user wants to move to a
  new arduino-esp32 / ESP-IDF / pioarduino release, "bump the BSP", update the ESP32 core, pin a new
  pioarduino version, or asks why the WiFi country check failed after a core change.
---

# Bump the ESP32 Arduino BSP

A BSP bump touches **four repositories**. Do them in this order, because each later step tests
against the earlier ones:

1. **adafruit/arduino-esp32** (fork): a `bsp-X-Y-Z` branch that mirrors the espressif tag.
2. **adafruit/ci-arduino**: `all_platforms.py` on a `ci-wippersnapper-bsp-X-Y-Z` branch, PR into `ci-wippersnapper`.
3. **adafruit/Wippersnapper_Boards**: only if a partition layout moved (`main` for v1, `api-v2` for v2).
4. **adafruit/Adafruit_Wippersnapper_Arduino** (this repo): the pioarduino pin, `.boards.local.txt`
   overrides, the WiFi country table, and DO-NOT-MERGE test PRs. v1 (`main`) and v2 (`migrate-api-v2`)
   get **separate** PRs.

The 3.3.11 → 3.3.13 bump (Nov 2026) is the worked example:
adafruit/ci-arduino#254, adafruit/Wippersnapper_Boards#283,
adafruit/Adafruit_Wippersnapper_Arduino#1130 and adafruit/Adafruit_Wippersnapper_Arduino#1133
(follow-ups and pin), and adafruit/Adafruit_Wippersnapper_Arduino#1131 and
adafruit/Adafruit_Wippersnapper_Arduino#1132 (throwaway CI test PRs, closed after CI).

## House rules

- **Never build with arduino-cli locally**, and never write into an installed Arduino15 core (not even a
  temporary `boards.local.txt`). The arduino-cli / ci-arduino path is verified by CI only.
- Local PlatformIO builds only from **PowerShell with the penv**, and only if asked. Default to CI.
  Never `pip install` into the penv, swap the pioarduino platform package, or `pio pkg install --force`
  without asking tyeth first. The penv is shared with other sessions: if a `pio` process is running,
  leave it alone.
- Push branches to the **adafruit upstream repos**, not a fork. Fork PRs don't run CI.
- Write PR and issue references as `owner/repo#num` or full URLs, never a bare `#num`.
- Pick the BSP version with tyeth. Prefer waiting for an upstream tag over patching the fork.

## Step 0: Inventory (before changing anything)

Record the current state so you know what is moving and what is deliberately staying put:

```bash
# Fork refs ci-arduino uses for ESP32 targets, with target counts
gh api -H "Accept: application/vnd.github.raw" \
  "repos/adafruit/ci-arduino/contents/all_platforms.py?ref=ci-wippersnapper" \
  | grep -oE '"[A-Za-z0-9_-]+/[A-Za-z0-9_.-]+"\]' | sort | uniq -c
# pioarduino pins and other version strings on both firmware lines
for b in origin/main origin/migrate-api-v2; do
  git grep -n -E "pioarduino/platform-espressif32|bsp-3-3|3\.3\.[0-9]+|idf-release" $b -- \
    platformio.ini .github 'examples/**/.*.boards.local.txt'
done
```

On 2026-11-25 most targets were on `adafruit/bsp-3-3-13`. The outliers are pinned on purpose and do
**not** move with a routine bump unless tyeth says so:
- ESP32-S31 targets: `adafruit/4.0.0-RC1-wipper` (arduino-esp32 4.x, IDF 6.1).
- ESP32-P4 targets: `adafruit/wipper-3.3.12-idf5.5.5` and `adafruit/master`.

## Step 1: Fork branch in adafruit/arduino-esp32

Fork `bsp-X-Y-Z` branches are plain mirrors of the espressif tag with **no patches**
(`bsp-3-3-11` and `bsp-3-3-13` both were). adafruit/arduino-esp32 is a GitHub fork of
espressif/arduino-esp32, so it shares objects and the branch can be made from the tag's commit:

```bash
SHA=$(gh api repos/espressif/arduino-esp32/commits/X.Y.Z --jq .sha)
gh api repos/adafruit/arduino-esp32/git/refs -f ref=refs/heads/bsp-X-Y-Z -f sha=$SHA
gh api repos/adafruit/arduino-esp32/branches/bsp-X-Y-Z --jq .commit.sha   # must equal $SHA
```

This creates a branch in an Adafruit repo, so confirm with tyeth before running it.

## Step 2: adafruit/ci-arduino `all_platforms.py`

Branch `ci-wippersnapper-bsp-X-Y-Z` off `ci-wippersnapper`.

1. Replace the old fork ref with `adafruit/bsp-X-Y-Z` on every target that used it. Leave the
   outliers from Step 0 alone.
2. **Find dead refs.** Check that every fork ref still in the file exists
   (`gh api repos/adafruit/arduino-esp32/branches/<ref>`). In 3.3.13, `arduino_nesso_n1` pointed at
   `wipper-esp3.3.5-pre-c0395df`, which had been deleted. It moved to the new branch.
3. **Validate every changed FQBN against the new tag's `boards.txt`.** Each board ID and each
   `menu.<opt>.<value>` in the FQBN must exist:
   ```bash
   gh api -H "Accept: application/vnd.github.raw" \
     "repos/espressif/arduino-esp32/contents/boards.txt?ref=X.Y.Z" > boards.txt
   grep -E "^<board_id>\.menu\.PartitionScheme\.<value>=" boards.txt
   ```
   A missing option fails the build with "Invalid FQBN". New menu options can also make an
   override unnecessary (see Step 4).
4. **Check app-slot sizes** for each WipperSnapper target's `PartitionScheme` against the latest v2
   CI binary sizes. In 3.3.13, FireBeetle 2 ESP32-C5 v2 was 1,972,665 B, which overflows
   `min_spiffs` (1,966,080 B), so it moved to `huge_app`. That moved its LittleFS too (Step 3).
5. Open the PR into `ci-wippersnapper` as a draft. List what moved, what stayed and why, the FQBN
   validation, and the companion PRs. **Don't merge it until both firmware test PRs (Step 6) are green.**

## Step 3: adafruit/Wippersnapper_Boards (only when a partition layout moved)

If a target's partition scheme changed, or the new BSP changed that scheme's CSV
(`tools/partitions/*.csv` at the tag), update the board's `definition.json` `offset` and
`fileSystemSize` to the new LittleFS/SPIFFS partition. Otherwise the web flasher writes the
filesystem to the wrong place and it won't mount. v2 boards live on the `api-v2` branch.
Example: adafruit/Wippersnapper_Boards#283 moved the C5 to `0x310000` / `917504`.
Merge it together with the ci-arduino PR.

## Step 4: This repo, per line (v1 `main` and v2 `migrate-api-v2`, separate PRs)

1. **pioarduino pin** in `[common:esp32]` of `platformio.ini`. Pin a numbered release whose
   `platform.json` ships the same arduino-esp32 tag as ci-arduino. Never pin the moving `stable`
   release.
   ```bash
   curl -sL -o pa.zip https://github.com/pioarduino/platform-espressif32/releases/download/<release>/platform-espressif32.zip
   python -c "import zipfile,json;z=zipfile.ZipFile('pa.zip');n=min((x for x in z.namelist() if x.endswith('platform.json')),key=len);print(json.loads(z.read(n))['packages']['framework-arduinoespressif32']['version'])"
   # expect .../arduino-esp32/releases/download/X.Y.Z/...
   ```
   Example: `55.03.313` ships 3.3.13 (adafruit/Adafruit_Wippersnapper_Arduino#1133 on v1,
   commit `08941a82` on v2).
2. **Per-example `.boards.local.txt` overrides** in `examples/*/`. Each one exists to work around
   a BSP gap and usually says when to drop it ("drop this once arduino-esp32 ships ..."). Re-check
   each one against the new `boards.txt`, and delete those that are no longer needed (3.3.13 deleted
   `.dfrobot_firebeetle2_esp32c5.boards.local.txt`). Keep the rest.
3. **Stale version comments.** Grep `platformio.ini` and `.github` for the old version string
   (for example `bsp-3-3-10` in a parity comment) and update any that describe the current pin.
4. **Workflows: usually nothing to change.** Both build workflows (`build-clang-doxy.yml` on v1,
   `build-migrate-api-v2.yml` on v2) take the ci-arduino ref from the `CI_ARDUINO_REF` env.
   `.github/scripts/resolve_bsp_refs.py` reads the fork refs from `ci/all_platforms.py` at run time,
   so the BSP and tool caches re-key themselves and need no edits. `membrowse-onboard.yml` clones
   `ci-wippersnapper` by name, and `wifi-country-sync.yml` has its own `CI_ARDUINO_REF`.
   Grep for anything new that hardcodes a BSP version.

## Step 5: WiFi country table sync check (MANDATORY)

`esp_wifi_set_country_code()` only accepts the codes listed in its doc comment in the ESP-IDF that
the BSP ships (`components/esp_wifi/include/esp_wifi.h`, `@attention 6. Supported country codes are
...`). The firmware mirrors that list in `src/helpers/ws_helper_wifi_country.h`:
- the `ch13` string: 2.4GHz channels 1-13;
- the `ch11` string: in IDF's list but limited to channels 1-11 (US, CA, MX, TW);
- the `strcmp(cc, "JP")` case: channels 1-14.

ESP8266 has no list of its own and takes its channel range from this table. ESP32 hands the code
to IDF, which rejects anything outside its list, so the two must match.
Run the check against the **new** pins, before opening the firmware PRs:

```bash
python .github/scripts/check_wifi_country_table.py \
  --platformio platformio.ini \
  --all-platforms <path to ci-arduino checkout of ci-wippersnapper-bsp-X-Y-Z>/all_platforms.py
# or, before the platformio.ini change: --platformio "" --bsp adafruit/bsp-X-Y-Z
```

The script follows the chain: pioarduino zip `platform.json` → arduino-esp32 tag →
`package/package_esp32_index.template.json` `esp32-arduino-libs` version
(`idf-release_v5.5-a45de8cc-v1`) → ESP-IDF commit `a45de8cc` → `esp_wifi.h`.
It exits 1 on drift against the pioarduino pin or ci-arduino's main ESP32 ref. One-off refs only warn.
CI runs the same check in `.github/workflows/wifi-country-sync.yml` on every v2 push and PR.

If it reports drift:
- **"IDF supports, table lacks: XX"**: add XX to `ch13` or `ch11`, depending on whether the country
  allows channels 12-13. Check the regulatory domain; don't guess. Update the doc comment that lists
  the ch11 countries too.
- **"table has, IDF rejects: XX"**: remove XX from the table, because ESP32 would now reject it.
  Mention it in the PR, since users with that `network_country` in secrets.json fall back to the
  world-safe default.
- If two BSP refs that both still build disagree, keep the table on the pioarduino pin / main ref
  and tell tyeth about the outlier in the PR.

Put the table change in the v2 BSP-bump PR (the table is v2-only), and re-run the script until it
passes.

## Step 6: Test through DO-NOT-MERGE PRs, then land

1. In this repo, open throwaway draft PRs that only point CI at the ci-arduino branch.
   Each is a one-line `CI_ARDUINO_REF: ci-wippersnapper-bsp-X-Y-Z` change, in:
   - v1: `.github/workflows/build-clang-doxy.yml`, PR into `main` (like adafruit/Adafruit_Wippersnapper_Arduino#1132);
   - v2: `.github/workflows/build-migrate-api-v2.yml` and `.github/workflows/wifi-country-sync.yml`,
     stacked on the v2 bump PR (like adafruit/Adafruit_Wippersnapper_Arduino#1131).
   Title them `DO NOT MERGE: build vN against ci-arduino ci-wippersnapper-bsp-X-Y-Z`.
2. Wait for CI. Read failures properly: compare against a recent `main` / `migrate-api-v2` run before
   blaming the BSP. Known pre-existing failures (for example membrowse TinyUSB link errors) aren't
   caused by the bump.
3. When both are green: merge the Boards PR and the ci-arduino PR together, then the firmware bump
   PRs. Close the DO-NOT-MERGE PRs with a comment that links the runs.
   `migrate-api-v2` has no required checks, so auto-merge merges at once. Wait for green, then merge by hand.

## Done checklist

- [ ] `adafruit/arduino-esp32` `bsp-X-Y-Z` equals the espressif tag's commit
- [ ] ci-arduino PR: refs moved, dead refs fixed, FQBNs validated, outliers listed
- [ ] Boards PR(s) for any moved partition, or a note saying none moved
- [ ] v1 and v2 `platformio.ini` pinned to a numbered pioarduino release that ships the same tag
- [ ] Obsolete `.boards.local.txt` overrides removed, stale version comments updated
- [ ] `check_wifi_country_table.py` passes against the new pins (table updated if needed)
- [ ] DO-NOT-MERGE v1/v2 test PRs green, then closed
- [ ] Every PR body cross-links the others with `owner/repo#num`
