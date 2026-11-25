#!/usr/bin/env python3
"""Check the firmware's WiFi country table against ESP-IDF's supported list.

ESP-IDF's esp_wifi_set_country_code() only accepts the country codes listed in
its doc comment in components/esp_wifi/include/esp_wifi.h ("Supported country
codes are ..."). The firmware keeps its own copy of that list in
src/helpers/ws_helper_wifi_country.h, as `static const char chNN[]` strings of
two-letter codes plus `strcmp(cc, "XX")` special cases (JP). This script
compares the two sets, so a BSP bump that changes IDF's list is caught.

The IDF commit is found from each BSP like this:
  pioarduino platform.json  -> arduino-esp32 release tag
  arduino-esp32 package/package_esp32_index.template.json
                            -> esp32-arduino-libs version, e.g.
                               idf-release_v5.5-a45de8cc-v1
  the hex part of that      -> ESP-IDF commit -> esp_wifi.h

Usage:
  check_wifi_country_table.py [--table PATH] [--platformio PATH]
                              [--all-platforms PATH] [--bsp OWNER/REF ...]
                              [--esp-wifi-h PATH ...]

  --table          firmware table (default src/helpers/ws_helper_wifi_country.h)
  --platformio     check the pioarduino pin in this platformio.ini
                   (default platformio.ini, pass "" to skip)
  --all-platforms  ci-arduino's all_platforms.py: check every arduino-esp32
                   fork ref it uses. The ref most targets use is strict, the
                   rest (S31, P4 and other one-off targets) only warn.
  --bsp            an arduino-esp32 ref to check, "owner/ref" (strict)
  --esp-wifi-h     a local esp_wifi.h to check (strict)

Exits 1 if the table file is missing, if a strict source drifts, or if no
strict source could be checked. Fetches use raw.githubusercontent.com and send
GH_TOKEN / GITHUB_TOKEN when set. Stdlib only, so it runs on Windows too.
"""
import argparse
import collections
import io
import json
import os
import re
import runpy
import sys
import time
import urllib.error
import urllib.request
import zipfile

GHA = os.environ.get("GITHUB_ACTIONS") == "true"
TEMPLATE = "package/package_esp32_index.template.json"
ESP_WIFI_H = "components/esp_wifi/include/esp_wifi.h"


def annotate(level, msg):
    """Prints msg as a GitHub annotation in CI, or plainly elsewhere."""
    if GHA:
        print("::%s::%s" % (level, msg.replace("\n", "%0A")))
    else:
        print("%s: %s" % (level.upper(), msg))


def fetch_raw(owner_repo, ref, path):
    """Returns a file from GitHub as text, retrying transient failures."""
    url = "https://raw.githubusercontent.com/%s/%s/%s" % (owner_repo, ref, path)
    headers = {"User-Agent": "wippersnapper-ci"}
    token = os.environ.get("GH_TOKEN") or os.environ.get("GITHUB_TOKEN")
    if token:
        headers["Authorization"] = "token " + token
    for attempt in range(1, 4):
        try:
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req, timeout=60) as r:
                return r.read().decode("utf-8")
        except urllib.error.HTTPError as e:
            if e.code == 404:
                raise RuntimeError("not found: %s" % url)
            err = e
        except (urllib.error.URLError, TimeoutError) as e:
            err = e
        time.sleep(attempt * 5)
    raise RuntimeError("fetching %s failed: %s" % (url, err))


def idf_supported(header_text):
    """Returns the country codes in esp_wifi.h's "Supported country codes"."""
    m = re.search(r"Supported country codes(.*?)(?:@attention|\*/)", header_text,
                  re.S)
    if not m:
        raise RuntimeError('no "Supported country codes" comment in esp_wifi.h')
    codes = set(re.findall(r'"([0-9A-Z]{2})"', m.group(1)))
    codes.discard("01")  # world safe mode, not a country
    if not codes:
        raise RuntimeError("empty supported country list in esp_wifi.h")
    return codes


def firmware_table(path):
    """Returns {code: source} from the firmware's chNN[] strings and strcmp()s."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    # Drop comments, so codes named in doc comments aren't picked up
    text = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)
    table = {}
    arrays = re.findall(
        r"static\s+const\s+char\s+(ch\d+)\s*\[\s*\]\s*(?:PROGMEM\s*)?=\s*"
        r"((?:\"[^\"]*\"\s*)+);", text)
    for name, literals in arrays:
        s = "".join(re.findall(r'"([^"]*)"', literals))
        if len(s) % 2 or not re.fullmatch(r"[A-Z]*", s):
            raise RuntimeError("%s in %s isn't a run of two-letter codes: %r"
                               % (name, path, s))
        for i in range(0, len(s), 2):
            code = s[i:i + 2]
            if code in table:
                raise RuntimeError("%s is in both %s and %s" % (code, table[code], name))
            table[code] = name
    for code in re.findall(r'strcmp\s*\(\s*\w+\s*,\s*"([A-Z]{2})"\s*\)', text):
        if code in table:
            raise RuntimeError("%s is in both %s and a strcmp()" % (code, table[code]))
        table[code] = "strcmp"
    if not arrays:
        raise RuntimeError("no static const char chNN[] strings in %s" % path)
    return table


def idf_commit_from_bsp(owner_repo, ref):
    """Returns (libs version, IDF commit) from an arduino-esp32 ref."""
    tools = json.loads(fetch_raw(owner_repo, ref, TEMPLATE))["packages"][0]["tools"]
    for t in tools:
        if t["name"] == "esp32-arduino-libs":
            m = re.search(r"-([0-9a-f]{7,40})-v\d+$", t["version"])
            if not m:
                raise RuntimeError("no IDF commit in esp32-arduino-libs version %r"
                                   % t["version"])
            return t["version"], m.group(1)
    raise RuntimeError("no esp32-arduino-libs tool in %s@%s" % (owner_repo, ref))


def arduino_tag_from_platformio(path):
    """Returns (pioarduino release, arduino-esp32 tag) for platformio.ini's pin.

    Reads platform.json from the pinned release zip itself (about 2MB), so a
    moving release name such as "stable" resolves to what PlatformIO gets.
    """
    with open(path, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"^\s*platform\s*=\s*(https://github\.com/pioarduino/"
                  r"platform-espressif32/releases/download/([^/\s]+)/\S+\.zip)",
                  text, re.M)
    if not m:
        raise RuntimeError("no pioarduino release pin in %s" % path)
    zip_url, release = m.groups()
    if not re.fullmatch(r"\d+\.\d+\.\d+", release):
        annotate("warning", "platformio.ini pins pioarduino release %r, which "
                 "moves; pin a numbered release such as 55.03.313" % release)
    with urllib.request.urlopen(
            urllib.request.Request(zip_url, headers={"User-Agent": "wippersnapper-ci"}),
            timeout=120) as r:
        z = zipfile.ZipFile(io.BytesIO(r.read()))
    name = min((n for n in z.namelist() if n.endswith("platform.json")), key=len)
    pkgs = json.loads(z.read(name))["packages"]
    url = pkgs["framework-arduinoespressif32"]["version"]
    t = re.search(r"arduino-esp32/releases/download/([^/]+)/", url)
    if not t:
        raise RuntimeError("can't find the arduino-esp32 tag in %s" % url)
    return release, t.group(1)


def esp32_refs(all_platforms_path):
    """Returns Counter({"owner/branch": targets}) for arduino-esp32 forks."""
    platforms = runpy.run_path(all_platforms_path)["ALL_PLATFORMS"]
    refs = collections.Counter()
    for entry in platforms.values():
        if len(entry) > 2 and entry[2] and str(entry[0]).startswith("espressif:esp32:"):
            refs[entry[2]] += 1
    return refs


def compare(label, idf, table, strict):
    """Reports drift between IDF's list and the table. Returns True if equal."""
    missing = sorted(idf - set(table))
    extra = sorted(set(table) - idf)
    if not missing and not extra:
        print("OK   %s: %d codes match" % (label, len(idf)))
        return True
    lines = ["WiFi country table drift against %s" % label]
    if missing:
        lines.append("  IDF supports, table lacks: %s (add each to ch13 or "
                     "ch11 by its 2.4GHz channel range)" % " ".join(missing))
    if extra:
        lines.append("  table has, IDF rejects: %s" % " ".join(
            "%s(%s)" % (c, table[c]) for c in extra))
    annotate("error" if strict else "warning", "\n".join(lines))
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--table", default="src/helpers/ws_helper_wifi_country.h")
    ap.add_argument("--platformio", default="platformio.ini")
    ap.add_argument("--all-platforms")
    ap.add_argument("--bsp", action="append", default=[])
    ap.add_argument("--esp-wifi-h", action="append", default=[])
    args = ap.parse_args()

    if not os.path.isfile(args.table):
        annotate("error", "WiFi country table file not found: %s" % args.table)
        return 1
    try:
        table = firmware_table(args.table)
    except RuntimeError as e:
        annotate("error", str(e))
        return 1
    print("Table %s: %d codes" % (args.table, len(table)))

    # (label, owner/repo, ref, strict) BSP sources, then local headers
    bsps = []
    failed = False
    if args.platformio:
        try:
            release, tag = arduino_tag_from_platformio(args.platformio)
            bsps.append(("pioarduino %s (arduino-esp32 %s)" % (release, tag),
                         "espressif/arduino-esp32", tag, True))
        except (RuntimeError, KeyError, ValueError, OSError,
                zipfile.BadZipFile) as e:
            annotate("error", "platformio.ini: %s" % e)
            failed = True
    for spec in args.bsp:
        owner, _, ref = spec.partition("/")
        bsps.append(("%s/arduino-esp32@%s" % (owner, ref),
                     owner + "/arduino-esp32", ref, True))
    if args.all_platforms:
        refs = esp32_refs(args.all_platforms)
        main_ref = refs.most_common(1)[0][0] if refs else None
        for spec, n in refs.most_common():
            owner, _, ref = spec.partition("/")
            bsps.append(("ci-arduino %s/arduino-esp32@%s (%d targets)" % (owner, ref, n),
                         owner + "/arduino-esp32", ref, spec == main_ref))

    checked = 0
    idf_cache = {}
    for label, repo, ref, strict in bsps:
        try:
            libs, idf = idf_commit_from_bsp(repo, ref)
            if idf not in idf_cache:
                idf_cache[idf] = idf_supported(fetch_raw("espressif/esp-idf", idf, ESP_WIFI_H))
        except (RuntimeError, KeyError, ValueError) as e:
            annotate("error" if strict else "warning", "%s: %s" % (label, e))
            failed = failed or strict
            continue
        ok = compare("%s, %s" % (label, libs), idf_cache[idf], table, strict)
        failed = failed or (strict and not ok)
        checked += strict
    for path in args.esp_wifi_h:
        try:
            with open(path, encoding="utf-8") as f:
                idf = idf_supported(f.read())
        except (OSError, RuntimeError) as e:
            annotate("error", "%s: %s" % (path, e))
            failed = True
            continue
        failed = failed or not compare(path, idf, table, True)
        checked += 1

    if not checked:
        annotate("error", "no strict BSP source could be checked")
        return 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
