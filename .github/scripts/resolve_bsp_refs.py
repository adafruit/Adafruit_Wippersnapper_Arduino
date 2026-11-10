#!/usr/bin/env python3
"""Resolve the source BSP refs needed by every build target in a workflow.

Usage: resolve_bsp_refs.py <workflow.json> <ci-arduino/all_platforms.py>

The workflow must already be converted to JSON (e.g. `yq -o=json`). Every
arduino-platform target in every job matrix is looked up in ci-arduino's
ALL_PLATFORMS; the optional third field ("owner/branch") names the
arduino-esp32 fork and branch (or tag) that build_platform.py clones for that
target. Each unique ref is resolved to its commit SHA, so the cache key changes
whenever the branch moves. A single matrix may mix refs, because the keys are
per target. A ref that doesn't resolve is reported as an ::error:: annotation
but doesn't fail the job: its targets just aren't cached and fall back to
build_platform.py's own clone, so the other refs are still cached and
downstream jobs aren't skipped.

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

def ls_remote(owner, name):
    """SHA that `git clone -b <name>` would check out: a branch, else a tag (peeled).
    Returns None if neither exists. Network failures are retried; if they persist,
    also None, with a warning instead of the not-found error."""
    url = "https://github.com/%s/arduino-esp32.git" % owner
    for attempt in range(1, 4):
        # The peeled "^{}" line (an annotated tag's commit) is only listed when asked for
        try:
            r = subprocess.run(["git", "ls-remote", url, name, name + "^{}"],
                               capture_output=True, text=True, timeout=120)
        except subprocess.TimeoutExpired:
            print("::warning::git ls-remote %s timed out (attempt %d)" % (url, attempt), file=sys.stderr)
            time.sleep(10 * attempt)
            continue
        if r.returncode == 0:
            found = dict(reversed(line.split("\t")) for line in r.stdout.splitlines() if "\t" in line)
            for ref in ("refs/heads/" + name, "refs/tags/%s^{}" % name, "refs/tags/" + name):
                if ref in found:
                    return found[ref]
            print("::error::%s/%s (used by %s) is not a branch or tag of %s/arduino-esp32; "
                  "these targets fall back to build_platform.py's own clone"
                  % (owner, name, ", ".join(users[owner + "/" + name]), owner), file=sys.stderr)
            return None
        print("::warning::git ls-remote %s failed (attempt %d): %s" % (url, attempt, r.stderr.strip()), file=sys.stderr)
        time.sleep(10 * attempt)
    print("::warning::could not reach %s; %s/%s targets are not cached this run" % (url, owner, name), file=sys.stderr)
    return None


users = {}
for target in sorted(set(targets)):
    entry = platforms.get(target)
    if entry is None:
        print("::warning::%s is not in ALL_PLATFORMS" % target, file=sys.stderr)
        continue
    # Only "owner/branch" strings are BSP refs; ALL_PLATFORMS also holds group tuples
    if len(entry) < 3 or not isinstance(entry[2], str) or "/" not in entry[2]:
        continue
    users.setdefault(entry[2], []).append(target)

# An unresolvable ref is reported, but doesn't fail the job: that would skip the
# merge/release jobs downstream, and the other refs can still be cached.
refs, keys, bsp = {}, {}, {}
for ref_info, used_by in users.items():
    # Same split as ci-arduino's manually_install_esp32_bsp()
    owner, branch = ref_info.split("/")[0], ref_info.split("/")[1]
    sha = ls_remote(owner, branch)
    if not sha:
        continue
    refs[ref_info] = {"owner": owner, "branch": branch, "sha": sha,
                      "key": salted("bsp-arduino-esp32-%s-%s-%s" % (owner, branch, sha)),
                      "tools_key": tools_key(owner, sha)}
    for target in used_by:
        keys[target] = refs[ref_info]["key"]
        bsp[target] = refs[ref_info]

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
