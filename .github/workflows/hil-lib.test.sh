#!/usr/bin/env bash
# Unit tests for hil-lib.sh (DUT-host-reboot tolerance helpers). Stubs curl/date/
# sleep so the wait/poll/classify logic runs deterministically with no controller
# and no real waiting. Run: bash .github/workflows/hil-lib.test.sh
set -uo pipefail
# Resolve to an ABSOLUTE dir before cd'ing away (CI invokes us with a relative
# path, so a relative source would break once cwd changes to the tmp dir).
HERE="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
cd "$TMP"

API="http://stub"; AUTH=(-H "x")

# --- mock clock: `date +%s` reads $NOW; `date -d @<epoch>` echoes that epoch ---
NOW=1000
date() {
  if [ "${1:-}" = "+%s" ]; then echo "$NOW"; return 0; fi
  if [ "${1:-}" = "-d" ]; then case "$2" in @*) echo "${2#@}";; *) echo 0;; esac; return 0; fi
  command date "$@"
}
# advance the mock clock instead of sleeping (truncate any fractional seconds)
sleep() { NOW=$(( NOW + ${1%.*} )); }

# --- mock curl: each call pops the next body from MOCK_BODIES (last one sticks).
# The call index lives in a file: the lib invokes curl inside $(...) subshells,
# so a plain variable wouldn't persist the increment across fetches.
MOCK_BODIES=(); MOCK_FAIL=0; MOCK_IFILE="$TMP/mock_i"; echo 0 > "$MOCK_IFILE"
curl() {
  [ "$MOCK_FAIL" = 1 ] && return 1
  local n=${#MOCK_BODIES[@]} i; i=$(cat "$MOCK_IFILE" 2>/dev/null || echo 0)
  local idx=$i; [ "$idx" -ge "$n" ] && idx=$((n-1))
  printf '%s' "${MOCK_BODIES[$idx]}"
  echo $((i+1)) > "$MOCK_IFILE"
}

source "$HERE/hil-lib.sh"
# fast, bounded budgets for the test
HIL_WAIT_BUDGET_S=360; HIL_WAIT_MARGIN_S=15; HIL_WAIT_POLL_S=15

PASS=0; FAILC=0
check() {  # desc, expected, actual
  if [ "$2" = "$3" ]; then PASS=$((PASS+1)); else FAILC=$((FAILC+1)); echo "FAIL: $1 — expected [$2] got [$3]"; fi
}
_avail='{"targets":[{"target":"qtpy","device_id":"d-1","available":true,"status":"available","kind":null,"reason":null,"retry_after":null}]}'
_temp() { echo "{\"targets\":[{\"target\":\"qtpy\",\"device_id\":\"d-1\",\"available\":false,\"status\":\"unavailable\",\"kind\":\"temporary\",\"reason\":\"host rebooting\",\"retry_after\":\"@$1\"}]}"; }
_perm='{"targets":[{"target":"qtpy","device_id":"d-1","available":false,"status":"unavailable","kind":"permanent","reason":"hardware removed","retry_after":null}]}'

# 1) available immediately
NOW=1000; echo 0 > "$MOCK_IFILE"; MOCK_BODIES=("$_avail")
out=$(wait_for_target_available qtpy 2>/dev/null); rc=$?
check "available: return code" 0 "$rc"
check "available: status line" "available d-1" "$out"

# 2) temporary then available (sleeps until retry_after, re-polls)
NOW=1000; echo 0 > "$MOCK_IFILE"; MOCK_BODIES=("$(_temp 1060)" "$_avail")
out=$(wait_for_target_available qtpy 2>/dev/null); rc=$?
check "temp-then-available: return code" 0 "$rc"
check "temp-then-available: status line" "available d-1" "$out"

# 3) permanent outage -> skip (return 2)
NOW=1000; echo 0 > "$MOCK_IFILE"; MOCK_BODIES=("$_perm")
out=$(wait_for_target_available qtpy 2>/dev/null); rc=$?
check "permanent: return code" 2 "$rc"
check "permanent: status starts skip permanent" "skip permanent hardware removed" "$out"

# 4) budget exceeded -> timeout (return 3); always temporary, retry_after far out
NOW=1000; echo 0 > "$MOCK_IFILE"; MOCK_BODIES=("$(_temp 99999)")
out=$(wait_for_target_available qtpy 2>/dev/null); rc=$?
check "budget-exceeded: return code" 3 "$rc"
check "budget-exceeded: status starts timeout" "timeout host rebooting" "$out"

# 5) classifier: clean finish is never transient (even with a scary events file)
echo "No route to host" > ev.log
is_host_offline_failure ev.log finished; check "classifier: finished not transient" 1 "$?"

# 6) classifier: errored + host-offline signature => transient (retry)
echo "stage enter_bootloader: No route to host" > ev.log
is_host_offline_failure ev.log error; check "classifier: no-route transient" 0 "$?"
echo "device d-1 unavailable: host USB stack wedged — reboot required, back ~300s" > ev.log
is_host_offline_failure ev.log error; check "classifier: unavailable-gate transient" 0 "$?"

# 7) classifier: errored on a real failure (no host signature) => NOT transient
echo "PIXELWRITE_VERDICT rebooted=false" > ev.log
is_host_offline_failure ev.log error; check "classifier: real failure not transient" 1 "$?"

echo "---- $PASS passed, $FAILC failed ----"
[ "$FAILC" -eq 0 ]
