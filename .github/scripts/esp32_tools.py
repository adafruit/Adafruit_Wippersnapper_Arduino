#!/usr/bin/env python3
"""Cache helper for the arduino-esp32 toolchain archives that tools/get.py installs.

get.py only downloads an archive when tools/dist/<archiveFileName> is missing.
When the file is present it checks the sha256 against
package/package_esp32_index.template.json and then extracts it. On a checksum
mismatch it downloads once more. So caching tools/dist (the archives) is
enough. The extracted toolchains don't need caching.

Usage:
  esp32_tools.py key   <template.json>
      Print the cache key for the archive set (host-specific, content-addressed).
  esp32_tools.py fetch <template.json> <dist_dir>
      Make sure every archive is in <dist_dir> with the right sha256. Download
      the missing ones with curl, retrying and resuming. Exit 1 if any archive
      is still missing or bad, so a warm job never saves an incomplete set.
  esp32_tools.py stage <bsp_dir> <branch> <cache_dir>
      Build jobs: if <bsp_dir> is a git checkout of <branch>, move the restored
      archives from <cache_dir> into <bsp_dir>/tools/dist and fetch any that are
      missing. Always exits 0. get.py stays the last line of defence.
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

HOST = os.environ.get("ESP32_TOOLS_HOST", "x86_64-pc-linux-gnu")
TEMPLATE = os.path.join("package", "package_esp32_index.template.json")


def tools_for_host(template_path, host=HOST):
    """Same selection as get.py's load_tools_list() for a Linux host."""
    with open(template_path, encoding="utf-8") as f:
        tools = json.load(f)["packages"][0]["tools"]
    out = []
    for t in tools:
        systems = [s for s in t["systems"] if s["host"] == host]
        if systems:
            out.append(systems[0])
    return out


def tools_key_from_list(tools, host=HOST):
    pairs = sorted((t["archiveFileName"], t["checksum"]) for t in tools)
    digest = hashlib.sha256(json.dumps(pairs).encode()).hexdigest()[:16]
    return "esp32-tools-%s-%s" % (host, digest)


def sha256sum(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def is_good(path, tool):
    return os.path.isfile(path) and sha256sum(path) == tool["checksum"].split(":", 1)[1]


def download(tool, dist_dir, attempts=3):
    path = os.path.join(dist_dir, tool["archiveFileName"])
    size = int(tool.get("size") or 0)
    for attempt in range(1, attempts + 1):
        # A full-size file that fails the checksum can't be resumed, so start over
        if os.path.isfile(path) and size and os.path.getsize(path) >= size and not is_good(path, tool):
            os.remove(path)
        start = time.time()
        r = subprocess.run(["curl", "-fsSL", "--retry", "8", "--retry-all-errors", "--retry-delay", "15",
                            "--connect-timeout", "30", "--max-time", "1200", "-C", "-",
                            "-o", path, tool["url"]])
        if r.returncode == 0 and is_good(path, tool):
            print("  fetched %s (%.0fs, attempt %d)" % (tool["archiveFileName"], time.time() - start, attempt))
            return True
        print("::warning::%s: attempt %d failed (curl exit %d)" % (tool["archiveFileName"], attempt, r.returncode))
        if attempt < attempts:
            time.sleep(30 * attempt)
    return False


def fetch(template_path, dist_dir):
    os.makedirs(dist_dir, exist_ok=True)
    ok = True
    for tool in tools_for_host(template_path):
        path = os.path.join(dist_dir, tool["archiveFileName"])
        if is_good(path, tool):
            print("  cached  %s" % tool["archiveFileName"])
        elif not download(tool, dist_dir):
            print("::error::could not fetch %s" % tool["url"])
            ok = False
    return ok


def stage(bsp_dir, branch, cache_dir):
    head = subprocess.run(["git", "-C", bsp_dir, "symbolic-ref", "--short", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    if not os.path.isdir(os.path.join(bsp_dir, ".git")) or head != branch:
        # ci-arduino would rmtree a mismatched checkout, taking tools/dist with it
        print("::warning::%s is not a checkout of %s (HEAD=%r); leaving tools to get.py" % (bsp_dir, branch, head))
        return
    dist_dir = os.path.join(bsp_dir, "tools", "dist")
    os.makedirs(dist_dir, exist_ok=True)
    moved = 0
    if os.path.isdir(cache_dir):
        for name in os.listdir(cache_dir):
            shutil.move(os.path.join(cache_dir, name), os.path.join(dist_dir, name))
            moved += 1
    print("Staged %d cached archive(s) into %s" % (moved, dist_dir))
    if not fetch(os.path.join(bsp_dir, TEMPLATE), dist_dir):
        print("::warning::some tool archives are still missing; get.py will try to download them")


if __name__ == "__main__":
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == "key":
        print(tools_key_from_list(tools_for_host(args[0])))
    elif cmd == "fetch":
        sys.exit(0 if fetch(args[0], args[1]) else 1)
    elif cmd == "stage":
        stage(args[0], args[1], args[2])
    else:
        sys.exit("unknown command " + cmd)
