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

# 5) infra-error classifier: a clean finish is never an infra error (real verdict)
is_infra_error finished; check "infra: finished not infra" 1 "$?"
is_infra_error cancelled; check "infra: cancelled not infra" 1 "$?"

# 6) infra-error classifier: error/timeout/failed (and empty) => infra error (retry)
is_infra_error error;   check "infra: error is infra" 0 "$?"
is_infra_error timeout; check "infra: timeout is infra" 0 "$?"
is_infra_error failed;  check "infra: failed is infra" 0 "$?"
is_infra_error "";      check "infra: empty is infra" 0 "$?"

# 8) multi-DUT: two devices share a build_target; the first is down → pick the available one
_two='{"targets":[{"target":"qtpy","device_id":"d-down","available":false,"status":"unavailable","kind":"temporary","reason":"maintenance","retry_after":null},{"target":"qtpy","device_id":"d-up","available":true,"status":"available","kind":null,"reason":null,"retry_after":null}]}'
NOW=1000; echo 0 > "$MOCK_IFILE"; MOCK_BODIES=("$_two")
out=$(wait_for_target_available qtpy 2>/dev/null); rc=$?
check "multi-dut: return code" 0 "$rc"
check "multi-dut: picks the AVAILABLE device" "available d-up" "$out"

# 9) proof_window: quotes a window around the LAST match (up to + just after), not a tail
printf 'l1\nl2\nl3 boot\nl4\nl5 connected (io-x)\nl6 after\nl7\nl8\n' > "$TMP/log"
out=$(HIL_PROOF_BEFORE=1 HIL_PROOF_AFTER=1 proof_window "$TMP/log" 'connected \(io-'); rc=$?
check "proof_window: matched return code" 0 "$rc"
check "proof_window: window is [match-1 .. match+1]" "l4
l5 connected (io-x)
l6 after" "$out"

# 10) proof_window: no match → tail fallback (rc 1)
out=$(proof_window "$TMP/log" 'NOPE_NO_SUCH'); rc=$?
check "proof_window: unmatched return code (tail fallback)" 1 "$rc"

# 11) proof_window: HIL_PROOF_BEFORE=-1 → from boot (line 1) up to match+after
out=$(HIL_PROOF_BEFORE=-1 HIL_PROOF_AFTER=1 proof_window "$TMP/log" 'connected \(io-')
check "proof_window: -1 starts at boot" "l1
l2
l3 boot
l4
l5 connected (io-x)
l6 after" "$out"

# 12) proof_window: records the window's time span in PW_TS_START/PW_TS_END
printf '%s a\n%s PHRASE\n%s b\n' \
  '2026-01-01T00:00:01.000+00:00' '2026-01-01T00:00:02.000+00:00' '2026-01-01T00:00:03.000+00:00' > "$TMP/tslog"
HIL_PROOF_BEFORE=1 HIL_PROOF_AFTER=1 proof_window "$TMP/tslog" 'PHRASE' >/dev/null
check "proof_window: PW_TS_START is the window's first line ts" "2026-01-01T00:00:01.000+00:00" "$PW_TS_START"
check "proof_window: PW_TS_END is the window's last line ts"   "2026-01-01T00:00:03.000+00:00" "$PW_TS_END"

# 13) time_window: keep only lines whose leading UTC-ms timestamp is within [a,b]
#     (this is what aligns the protomq quote to the serial window — no line-count drift)
printf '%s early\n%s mid\n%s late\n' \
  '2026-01-01T00:00:01.000+00:00' '2026-01-01T00:00:05.000+00:00' '2026-01-01T00:00:09.000+00:00' > "$TMP/tw"
out=$(time_window "$TMP/tw" '2026-01-01T00:00:02.000+00:00' '2026-01-01T00:00:06.000+00:00')
check "time_window: keeps only the in-range line" "2026-01-01T00:00:05.000+00:00 mid" "$out"

echo "---- $PASS passed, $FAILC failed ----"
[ "$FAILC" -eq 0 ]
