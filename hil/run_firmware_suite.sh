#!/usr/bin/env bash
# SPDX-FileCopyrightText: Brent Rubell for Adafruit Industries, 2026
#
# SPDX-License-Identifier: MIT
#
# Runs one suite of hil/firmware tests ("api" or "system": the board
# entry's api_tests or system_tests in hil-boards.json) against BOARD, for
# build-hil.yml. Expects BOARD and LOG_DIR in the environment, and runs from
# hil/. FIRMWARE_DIR, if set, overrides the board entry's firmware_dir. A suite the board leaves empty (pytest exit code 5, no
# tests collected) passes.
set -uo pipefail

suite="$1"
creds=~/.config/hil/creds.env
if [ -f "$creds" ]; then
  set -a; . "$creds"; set +a
else
  echo "::warning::$creds not found; tests that need credentials will fail"
fi

fw_args=()
if [ -n "${FIRMWARE_DIR:-}" ]; then
  fw_args=(--firmware-dir "$FIRMWARE_DIR")
fi

venv/bin/pytest firmware --suite "$suite" \
  --board "$BOARD" \
  "${fw_args[@]}" \
  --serial-log-dir "$LOG_DIR" \
  --junitxml "$LOG_DIR/junit-$suite.xml" \
  --log-cli-level=INFO \
  --timeout=600
rc=$?
if [ "$rc" -eq 5 ]; then
  echo "No $suite tests selected for $BOARD"
  exit 0
fi
exit "$rc"
