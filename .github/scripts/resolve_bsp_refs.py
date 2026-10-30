#!/usr/bin/env python3
"""Resolve the source BSP refs needed by every build target in a workflow.

Usage: resolve_bsp_refs.py <workflow.json> <ci-arduino/all_platforms.py>

The workflow must already be converted to JSON (e.g. `yq -o=json`). Every
arduino-platform target in every job matrix is looked up in ci-arduino's
ALL_PLATFORMS; the optional third field ("owner/branch") names the
arduino-esp32 fork and branch build_platform.py clones for that target.
Each unique ref is resolved to its commit SHA so the cache key changes
whenever the branch moves.

Writes GitHub Actions outputs to stdout:
  refs=[{"owner", "branch", "sha", "key"}, ...]  one per unique ref
  keys={"<target>": "<cache key>", ...}          only targets with a ref
"""
import json
import runpy
import subprocess
import sys

workflow = json.load(open(sys.argv[1], encoding="utf-8"))
platforms = runpy.run_path(sys.argv[2])["ALL_PLATFORMS"]

targets = []
for job in workflow.get("jobs", {}).values():
    matrix = (job.get("strategy") or {}).get("matrix") or {}
    if isinstance(matrix, dict):
        targets += matrix.get("arduino-platform") or []
        targets += [i["arduino-platform"] for i in matrix.get("include") or [] if "arduino-platform" in i]

refs, keys, failed = {}, {}, False
for target in sorted(set(targets)):
    entry = platforms.get(target)
    if entry is None:
        print("::warning::%s is not in ALL_PLATFORMS" % target, file=sys.stderr)
        continue
    if len(entry) < 3 or not entry[2]:
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
                          "key": "bsp-arduino-esp32-%s-%s-%s" % (owner, branch, out[0])}
    if entry[2] in refs:
        keys[target] = refs[entry[2]]["key"]

for ref, info in refs.items():
    used_by = sorted(t for t, k in keys.items() if k == info["key"])
    print("%s @ %s: %d targets" % (ref, info["sha"][:8], len(used_by)), file=sys.stderr)
print("refs=" + json.dumps(list(refs.values())))
print("keys=" + json.dumps(keys))
sys.exit(1 if failed else 0)
