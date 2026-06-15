#!/usr/bin/env bash
# A/B pixelWrite HIL comparison driver, called by hil-test-suite.yml.
#
# For each requested build target the controller reports AVAILABLE: upload both
# firmware images, run firmware-bench (flash -> secrets -> power-cycle ->
# inject_pixelwrite) on each, and assert the verdict diverges — release crashes
# (rebooted=true), the PR fix survives (rebooted=false). Skipped/unavailable
# targets (temporary or permanent) are listed, never failing the run. Writes
# hil-out/comment.md for the PR and pulls serial.log/protomq.log/flash.log.
set -uo pipefail

API="${HIL_API_BASE:?}"; TOK="${HIL_API_TOKEN:?}"
AUTH=(-H "Authorization: Bearer ${TOK}")
# Shared host-reboot-tolerance helpers (wait_for_target_available, is_host_offline_failure).
source "${BASH_SOURCE[0]%/*}/hil-lib.sh"
mkdir -p hil-out   # append our section to the shared summary (workflow owns the marker/header)
fail=0; ran=0

jobreq() {  # side, target, device_id, fw_path -> stdout job json
  jq -n --arg dev "$3" --arg path "$4" \
    --arg u "${HIL_IO_USERNAME:-hil}" --arg k "${HIL_IO_KEY:-hil}" \
    --arg ss "${HIL_WIFI_SSID:-free4all}" --arg pw "${HIL_WIFI_PASSWORD:-password}" '{
    target: { device: { id: $dev }, pool: "public" },
    script: "firmware-bench",
    params: {
      firmware: { path: $path, offset: "0x0" },
      window_minutes: 1,
      stages: [
        {type:"enter_bootloader"},
        {type:"erase",  before:"no_reset", after:"no_reset"},
        {type:"flash",  offset:"0x0", before:"no_reset", after:"no_reset"},
        {type:"power_cycle"},
        {type:"write_secrets_msc"},
        {type:"power_cycle"},
        {type:"inject_pixelwrite", pin:"D0", color:200}
      ]
    },
    secrets: { IO_USERNAME:$u, IO_KEY:$k, WIFI_SSID:$ss, WIFI_PASSWORD:$pw },
    timeouts: { total_s: 1200 }
  }'
}

run_side() {  # side, target, device_id, fw_path -> sets RS_VERDICT (true|false|unknown) + RS_STATE
  local side="$1" t="$2" dev="$3" fw="$4"
  local fid path jid
  RS_VERDICT="unknown"; RS_STATE=""
  : > "hil-out/${t}-${side}.events.log"   # fresh per attempt (so the retry classifier sees only this run)
  path=$(curl -fsS "${AUTH[@]}" -X POST --data-binary "@${fw}" \
      "${API}/v1/firmware?filename=$(basename "$fw")" | jq -r '.path') || return 1
  jid=$(jobreq "$side" "$t" "$dev" "$path" | curl -fsS "${AUTH[@]}" -X POST \
      -H 'Content-Type: application/json' --data @- "${API}/v1/jobs" | jq -r '.id') || return 1
  echo "::group::[$t/$side] job $jid" >&2
  local since=0 state="" verdict="unknown" out
  # Time-bounded poll (not an iteration count): firmware-bench floods serial
  # events so each /wait returns instantly, exhausting a fixed loop. Poll until
  # the job is TERMINAL (not just the verdict): the serial/protomq/flash log
  # assets are registered at teardown, so downloading earlier misses them.
  local deadline=$(( $(date +%s) + 900 ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    out=$(curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/wait?since=${since}&timeout=10") || break
    echo "$out" | jq -r '.events[]?|.payload.msg // empty' 2>/dev/null | tee -a "hil-out/${t}-${side}.events.log" >&2
    if echo "$out" | grep -q 'PIXELWRITE_VERDICT rebooted=true'; then verdict=true; fi
    if echo "$out" | grep -q 'PIXELWRITE_VERDICT rebooted=false'; then verdict=false; fi
    since=$(echo "$out" | jq -r '.next_since // .since // 0')
    state=$(echo "$out" | jq -r '.state // ""')
    case "$state" in finished|failed|cancelled|error|timeout) break;; esac
  done
  # Drain trailing events: the error reason (e.g. "No route to host") often lands
  # AFTER the state→terminal event, so the loop above breaks before capturing it.
  out=$(curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/wait?since=${since}&timeout=2" 2>/dev/null) || out=""
  if [ -n "$out" ]; then
    echo "$out" | jq -r '.events[]?|.payload.msg // empty' 2>/dev/null | tee -a "hil-out/${t}-${side}.events.log" >&2
    echo "$out" | grep -q 'PIXELWRITE_VERDICT rebooted=true'  && verdict=true
    echo "$out" | grep -q 'PIXELWRITE_VERDICT rebooted=false' && verdict=false
  fi
  echo "[$t/$side] terminal state: ${state:-unknown}" >&2
  # Pull ONLY the captured log assets (serial/protomq/flash) — skip the firmware bin.
  curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/assets" \
    | jq -r '.assets[]? | select(.kind=="log") | "\(.id) \(.filename)"' \
    | while read -r aid fn; do
        [ -n "$aid" ] && curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/assets/${aid}/download" \
          -o "hil-out/${t}-${side}-${fn}" || true
      done
  echo "::endgroup::" >&2
  RS_VERDICT="$verdict"; RS_STATE="$state"
}

# Run one side with DUT-host-reboot tolerance: wait for the target (sleep out a
# reboot, bounded), run it, and if the job errored with a host-offline signature
# rather than a real verdict, wait the host out + re-run ONCE. Sets RSR_VERDICT
# (true|false|unknown, or "skip:<reason>" when not run) and RSR_NOTE. Returns 0
# when the side ran, 2 on a permanent skip, 3 if the host never came back.
run_side_resilient() {  # side, target, fw_path
  local side="$1" t="$2" fw="$3" status dev wrc
  RSR_VERDICT="unknown"; RSR_NOTE=""
  status=$(wait_for_target_available "$t"); wrc=$?
  if [ "$wrc" -ne 0 ]; then RSR_VERDICT="skip:$(echo "$status" | cut -d' ' -f2-)"; return "$wrc"; fi
  dev=$(echo "$status" | awk '{print $2}')
  run_side "$side" "$t" "$dev" "$fw"; RSR_VERDICT="$RS_VERDICT"
  if [ "$RSR_VERDICT" != "true" ] && [ "$RSR_VERDICT" != "false" ] && is_infra_error "$RS_STATE"; then
    echo "::warning::[$t/$side] infra error (state=${RS_STATE:-none}) — waiting for host + retrying once" >&2
    status=$(wait_for_target_available "$t"); wrc=$?
    if [ "$wrc" -eq 0 ]; then
      dev=$(echo "$status" | awk '{print $2}')
      run_side "$side" "$t" "$dev" "$fw"; RSR_VERDICT="$RS_VERDICT"; RSR_NOTE="host rebooted — retried"
    fi
  fi
  return 0
}

# Append an inline serial.log excerpt (proof) for a target/side to the comment.
append_proof() {  # target, side
  local t="$1" side="$2" sl
  sl=$(ls "hil-out/${t}-${side}-serial.log" 2>/dev/null | head -1)
  [ -z "$sl" ] && sl=$(ls "hil-out/${t}-${side}-flash.log" 2>/dev/null | head -1)
  [ -z "$sl" ] && return 0
  {
    echo
    echo "<details><summary>📜 \`$t\` ${side} — $(basename "$sl") (tail)</summary>"
    echo
    echo '```'
    tail -n 30 "$sl"
    echo '```'
    echo "</details>"
  } >> hil-out/comment.md
}

{
  echo
  echo "### 🧪 pixelWrite uninitialised-strand regression ([#926](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/issues/926))"
  echo
  echo "v1 \`ws.signal.pixelWrite\` to an uninitialised strand (pin **D0**, colour **200**) — release should **crash+reboot**, the fix should **log + continue** (#926 / fixed by #927)."
  echo
  echo "| target | release \`${LOW_REF}\` | this PR | verdict |"
  echo "|---|---|---|---|"
} >> hil-out/comment.md

for T in $TARGETS; do
  lowbin=$(find "fw/low-$T"  -name '*combined.bin' | head -1)
  highbin=$(find "fw/high-$T" -name '*combined.bin' | head -1)
  if [ -z "$lowbin" ] || [ -z "$highbin" ]; then
    lo=$([ -n "$lowbin" ] && echo ok || echo missing); hi=$([ -n "$highbin" ] && echo ok || echo missing)
    echo "| \`$T\` | $lo | $hi | ⚠️ firmware missing |" >> hil-out/comment.md; continue
  fi
  # LOW side (release) — wait out a host reboot before submitting + retry once on
  # a host-offline failure. A permanent outage / past-budget down skips the target.
  run_side_resilient low "$T" "$lowbin"; rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "| \`$T\` | — | — | ⏭️ skipped (${RSR_VERDICT#skip:}) |" >> hil-out/comment.md
    [ "$rc" -eq 3 ] && fail=1; continue
  fi
  ran=1; lv="$RSR_VERDICT"; lnote="$RSR_NOTE"
  # HIGH side (this PR) — same tolerance.
  run_side_resilient high "$T" "$highbin"; rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "| \`$T\` | rebooted=${lv} | — | ⏭️ HIGH skipped (${RSR_VERDICT#skip:}) |" >> hil-out/comment.md
    [ "$rc" -eq 3 ] && fail=1; append_proof "$T" low; continue
  fi
  hv="$RSR_VERDICT"; hnote="$RSR_NOTE"
  note=""; [ -n "$lnote" ] && note="${note} (low: $lnote)"; [ -n "$hnote" ] && note="${note} (high: $hnote)"
  pass="❌"; if [ "$lv" = "true" ] && [ "$hv" = "false" ]; then pass="✅"; else fail=1; fi
  echo "| \`$T\` | rebooted=${lv} | rebooted=${hv} | ${pass}${note} |" >> hil-out/comment.md
  append_proof "$T" low
  append_proof "$T" high
done

{
  echo
  echo "_Expected: release \`rebooted=true\` (crash), PR \`rebooted=false\` (graceful)._"
} >> hil-out/comment.md

[ "$ran" = 1 ] || echo "No available targets ran (all skipped)."
exit $fail
