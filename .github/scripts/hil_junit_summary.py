# SPDX-FileCopyrightText: Brent Rubell for Adafruit Industries, 2026
#
# SPDX-License-Identifier: MIT
"""Summarize a board's HIL junit-*.xml for build-hil.yml.

Prints a Markdown job summary to stdout: counts, then each failure with
its message, source line, and traceback, and each skip with its reason.
Also prints an ::error annotation per failure to stderr, so the failing
line is marked in the run and the PR diff.

    hil_junit_summary.py LOG_DIR BOARD >> "$GITHUB_STEP_SUMMARY"

Links use these environment variables when they're set:
    REPO_URL      https://github.com/OWNER/REPO
    SHA           commit the tests ran at
    JOB_URL       this job's log
    ARTIFACT_URL  the uploaded HIL logs (per-test serial/protomq logs)
"""
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# The tests run from hil/, so tracebacks' paths are relative to it
TEST_ROOT = "hil"
# A traceback's frame line, e.g. "firmware/pwm/test_pwm.py:356: AssertionError"
FRAME = re.compile(r"^(\S+\.py):(\d+): ", re.M)
MAX_TRACE_LINES = 40


def suites(log_dir):
    found = []
    for path in sorted(Path(log_dir).glob("junit-*.xml")):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            continue
        suite = path.stem.removeprefix("junit-")
        tags = [root] if root.tag == "testsuite" else root.findall("testsuite")
        found += [(suite, s) for s in tags]
    return found


def test_path(case):
    """firmware/pwm/test_pwm.py, from a classname like
    Repo.Repo.hil.firmware.pwm.test_pwm"""
    parts = case.get("classname", "").split(".")
    if TEST_ROOT in parts:
        parts = parts[parts.index(TEST_ROOT) + 1:]
    return "/".join(parts) + ".py"


def location(case, text):
    """(path under hil/, line) of the deepest frame in the test's own
    file, else the deepest frame anywhere under hil/, else the file."""
    path = test_path(case)
    frames = [(p, int(n)) for p, n in FRAME.findall(text or "")
              if not p.startswith("/")]
    own = [f for f in frames if f[0] == path]
    if own:
        return own[-1]
    if frames:
        return frames[-1]
    return path, None


def source_link(path, line):
    label = f"{path}:{line}" if line else path
    repo, sha = os.environ.get("REPO_URL"), os.environ.get("SHA")
    if not (repo and sha):
        return f"`{label}`"
    anchor = f"#L{line}" if line else ""
    return f"[`{label}`]({repo}/blob/{sha}/{TEST_ROOT}/{path}{anchor})"


def one_line(text, limit=300):
    text = " ".join((text or "").split())
    return text if len(text) <= limit else text[:limit - 1] + "…"


def escape_annotation(text):
    return text.replace("%", "%25").replace("\r", "%0D").replace("\n", "%0A")


def main():
    log_dir, board = sys.argv[1], sys.argv[2]
    found = suites(log_dir)
    if not found:
        print(f"### HIL {board}: no results (junit-*.xml missing)")
        return

    n = {k: sum(int(s.get(k, 0)) for _, s in found)
         for k in ("tests", "failures", "errors", "skipped")}
    passed = n["tests"] - n["failures"] - n["errors"] - n["skipped"]
    icon = "❌" if n["failures"] or n["errors"] else "✅"
    print(f"### {icon} HIL {board}\n")
    print("| Tests | Passed | Failures | Errors | Skipped |")
    print("|---|---|---|---|---|")
    print(f"| {n['tests']} | {passed} | {n['failures']} | {n['errors']} "
          f"| {n['skipped']} |\n")

    links = []
    # job.check_run_id is empty on runners too old to set it
    job_url = os.environ.get("JOB_URL", "").removesuffix("/job/")
    if job_url:
        links.append(f"[full job log]({job_url})")
    if os.environ.get("ARTIFACT_URL"):
        links.append(f"[per-test serial/protomq logs]"
                     f"({os.environ['ARTIFACT_URL']})")
    if links:
        print(" · ".join(links) + "\n")

    failed, skipped = [], []
    for suite, s in found:
        for case in s.iter("testcase"):
            bad = case.find("failure")
            if bad is None:
                bad = case.find("error")
            if bad is not None:
                failed.append((suite, case, bad))
            elif case.find("skipped") is not None:
                skipped.append((suite, case, case.find("skipped")))

    if failed:
        print(f"#### Failures ({len(failed)})\n")
    for suite, case, bad in failed:
        name = f"{test_path(case)}::{case.get('name')}"
        path, line = location(case, bad.text)
        message = one_line(bad.get("message"))
        kind = "error" if bad.tag == "error" else "failure"
        print(f"<details><summary>❌ <code>{name}</code> ({suite}"
              f"{', ' + kind if kind == 'error' else ''})</summary>\n")
        print(f"**{message or kind}**\n")
        print(f"at {source_link(path, line)}\n")
        trace = (bad.text or "").rstrip().splitlines()
        if len(trace) > MAX_TRACE_LINES:
            trace = ["…"] + trace[-MAX_TRACE_LINES:]
        if trace:
            print("```")
            print("\n".join(trace).replace("```", "``​`"))
            print("```")
        print("\n</details>\n")
        line_part = f",line={line}" if line else ""
        print(f"::error file={TEST_ROOT}/{path}{line_part},"
              f"title={escape_annotation(name)}::"
              f"{escape_annotation(message or kind)}", file=sys.stderr)

    if skipped:
        print(f"#### Skipped ({len(skipped)})\n")
        for suite, case, skip in skipped:
            reason = one_line(skip.get("message"), 200)
            print(f"- ⏭️ `{test_path(case)}::{case.get('name')}`: {reason}")


if __name__ == "__main__":
    main()
