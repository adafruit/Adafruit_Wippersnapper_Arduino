#!/usr/bin/env python3
"""Resolve the source BSP refs needed by every build target in a workflow.

Usage: resolve_bsp_refs.py <workflow.json> <ci-arduino/all_platforms.py>

The workflow must already be converted to JSON (e.g. `yq -o=json`). Every
arduino-platform target in every job matrix is looked up in ci-arduino's
ALL_PLATFORMS; the optional third field ("owner/branch") names the
arduino-esp32 fork and branch build_platform.py clones for that target.
Each unique ref is resolved to its commit SHA so the cache key changes
whenever the branch moves.

For each ref it also fetches package/package_esp32_index.template.json at
that SHA (via `gh api`, so GH_TOKEN must be set) and derives the cache key of
the toolchain archives that tools/get.py would download. If the fetch fails,
that ref simply gets no tools_key and the tools cache is skipped.

CACHE_SALT (env, optional) is appended to every key. Use it to force a cold cache.

Writes GitHub Actions outputs to stdout:
  refs=[{"owner", "branch", "sha", "key", "tools_key"}, ...]  one per unique ref
  keys={"<target>": "<cache key>", ...}                       only targets with a ref
  bsp={"<target>": {"owner", "branch", "sha", "key", "tools_key"}, ...}
  tools=[{"tools_key", "owner", "branch", "sha"}, ...]        one per unique tools_key
"""
import json
import os
import runpy
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32_tools import TEMPLATE, tools_for_host, tools_key_from_list  # noqa: E402

SALT = os.environ.get("CACHE_SALT", "").strip()


def salted(key):
    return key + "-" + SALT if SALT else key


def tools_key(owner, sha):
    url = "repos/%s/arduino-esp32/contents/%s?ref=%s" % (owner, TEMPLATE.replace(os.sep, "/"), sha)
    for attempt in range(1, 4):
        r = subprocess.run(["gh", "api", "-H", "Accept: application/vnd.github.raw", url],
                           capture_output=True, text=True)
        if r.returncode == 0:
            with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as f:
                f.write(r.stdout)
            try:
                return salted(tools_key_from_list(tools_for_host(f.name)))
            finally:
                os.remove(f.name)
        print("::warning::fetching %s failed (attempt %d): %s" % (url, attempt, r.stderr.strip()), file=sys.stderr)
        time.sleep(10 * attempt)
    return None

workflow = json.load(open(sys.argv[1], encoding="utf-8"))
platforms = runpy.run_path(sys.argv[2])["ALL_PLATFORMS"]

targets = []
for job in workflow.get("jobs", {}).values():
    matrix = (job.get("strategy") or {}).get("matrix") or {}
    if isinstance(matrix, dict):
        targets += matrix.get("arduino-platform") or []
        targets += [i["arduino-platform"] for i in matrix.get("include") or [] if "arduino-platform" in i]

refs, keys, bsp, failed = {}, {}, {}, False
for target in sorted(set(targets)):
    entry = platforms.get(target)
    if entry is None:
        print("::warning::%s is not in ALL_PLATFORMS" % target, file=sys.stderr)
        continue
    # Only "owner/branch" strings are BSP refs; ALL_PLATFORMS also holds group tuples
    if len(entry) < 3 or not isinstance(entry[2], str) or "/" not in entry[2]:
        continue
    owner, branch = entry[2].split("/")[0], entry[2].split("/")[1]
    if entry[2] not in refs:
        out = subprocess.run(
            ["git", "ls-remote", "https://github.com/%s/arduino-esp32.git" % owner, "refs/heads/" + branch],
            capture_output=True, text=True).stdout.split()
        if not out:
            print("::error::%s (used by %s) does not resolve to a branch of %s/arduino-esp32"
                  % (entry[2], target, owner), file=sys.stderr)
            failed = True
            continue
        refs[entry[2]] = {"owner": owner, "branch": branch, "sha": out[0],
                          "key": salted("bsp-arduino-esp32-%s-%s-%s" % (owner, branch, out[0])),
                          "tools_key": tools_key(owner, out[0])}
    if entry[2] in refs:
        keys[target] = refs[entry[2]]["key"]
        bsp[target] = refs[entry[2]]

for ref, info in refs.items():
    used_by = sorted(t for t, k in keys.items() if k == info["key"])
    print("%s @ %s: %d targets, tools %s" % (ref, info["sha"][:8], len(used_by), info["tools_key"]), file=sys.stderr)
tools = {}
for info in refs.values():
    if info["tools_key"] and info["tools_key"] not in tools:
        tools[info["tools_key"]] = {k: info[k] for k in ("tools_key", "owner", "branch", "sha")}
print("refs=" + json.dumps(list(refs.values())))
print("keys=" + json.dumps(keys))
print("bsp=" + json.dumps(bsp))
print("tools=" + json.dumps(list(tools.values())))
sys.exit(1 if failed else 0)
