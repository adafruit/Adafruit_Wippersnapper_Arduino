#!/usr/bin/env bash
# HIL check-in smoke test, called by hil-test-suite.yml.
#
# The lightweight default gate while the pixelWrite PR regression is parked:
# for each AVAILABLE target, flash THIS PR's build, write secrets, power-cycle,
# and assert the device checks in to the broker (CHECKIN_VERDICT ok=true). No
# signal injection, no A/B — just proof the end-to-end path (flash → secrets →
# WiFi → broker checkin) works on real hardware. Writes hil-out/comment.md and
# pulls serial.log/protomq.log/flash.log as proof.
set -uo pipefail

API="${HIL_API_BASE:?}"; TOK="${HIL_API_TOKEN:?}"
AUTH=(-H "Authorization: Bearer ${TOK}")
# Shared host-reboot-tolerance helpers (wait_for_target_available, is_host_offline_failure).
source "${BASH_SOURCE[0]%/*}/hil-lib.sh"
mkdir -p hil-out   # append our section to the shared summary (workflow owns the marker/header)
fail=0; ran=0

jobreq() {  # target, device_id, fw_path -> stdout job json
  jq -n --arg dev "$2" --arg path "$3" \
    --arg u "${HIL_IO_USERNAME:-hil}" --arg k "${HIL_IO_KEY:-hil}" \
    --arg ss "${HIL_WIFI_SSID:-free4all}" --arg pw "${HIL_WIFI_PASSWORD:-password}" '{
    target: { device: { id: $dev }, pool: "public" },
    script: "firmware-bench",
    params: {
      firmware: { path: $path, offset: "0x0" },
      window_minutes: 3,
      stages: [
        {type:"enter_bootloader"},
        {type:"erase",  before:"no_reset", after:"no_reset"},
        {type:"flash",  offset:"0x0", before:"no_reset", after:"no_reset"},
        {type:"power_cycle"},
        {type:"write_secrets_msc"},
        {type:"power_cycle"},
        {type:"verify_checkin"}
      ]
    },
    secrets: { IO_USERNAME:$u, IO_KEY:$k, WIFI_SSID:$ss, WIFI_PASSWORD:$pw },
    timeouts: { total_s: 1200 }
  }'
}

run_target() {  # target, device_id, fw_path -> sets RT_VERDICT (true|false|unknown) + RT_STATE
  local t="$1" dev="$2" fw="$3"
  local path jid since=0 state="" checkin="unknown" out
  RT_VERDICT="unknown"; RT_STATE=""
  : > "hil-out/${t}-checkin.events.log"   # fresh per attempt (so the retry classifier sees only this run)
  path=$(curl -fsS "${AUTH[@]}" -X POST --data-binary "@${fw}" \
      "${API}/v1/firmware?filename=$(basename "$fw")" | jq -r '.path') || return 1
  jid=$(jobreq "$t" "$dev" "$path" | curl -fsS "${AUTH[@]}" -X POST \
      -H 'Content-Type: application/json' --data @- "${API}/v1/jobs" | jq -r '.id') || return 1
  echo "::group::[$t/checkin] job $jid" >&2
  # Poll on a TIME budget, not an iteration count: firmware-bench floods serial
  # events so each /wait returns instantly — a fixed loop count burns out long
  # before the ~6-8min flash→secrets→checkin completes. Poll until the job is
  # TERMINAL (not just until the verdict): firmware-bench registers the
  # serial/protomq/flash log assets at teardown, so downloading earlier would
  # miss them and grab only the firmware bin.
  local deadline=$(( $(date +%s) + 900 ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    out=$(curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/wait?since=${since}&timeout=10") || break
    echo "$out" | jq -r '.events[]?|.payload.msg // empty' 2>/dev/null | tee -a "hil-out/${t}-checkin.events.log" >&2
    if echo "$out" | grep -q 'CHECKIN_VERDICT ok=true';  then checkin=true; fi
    if echo "$out" | grep -q 'CHECKIN_VERDICT ok=false'; then checkin=false; fi
    since=$(echo "$out" | jq -r '.next_since // .since // 0')
    state=$(echo "$out" | jq -r '.state // ""')
    case "$state" in finished|failed|cancelled|error|timeout) break;; esac
  done
  echo "[$t/checkin] terminal state: ${state:-unknown}" >&2
  # Pull ONLY the captured log assets (serial/protomq/flash) — skip the firmware bin.
  curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/assets" \
    | jq -r '.assets[]? | select(.kind=="log") | "\(.id) \(.filename)"' \
    | while read -r aid fn; do
        [ -n "$aid" ] && curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/assets/${aid}/download" \
          -o "hil-out/${t}-checkin-${fn}" || true
      done
  echo "::endgroup::" >&2
  RT_VERDICT="$checkin"; RT_STATE="$state"
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
  echo "### ✅ Check-in smoke test"
  echo
  echo "Flash this PR's build → write secrets → power-cycle → assert the device checks in to the broker."
  echo
  echo "| target | this PR | check-in |"
  echo "|---|---|---|"
} >> hil-out/comment.md

for T in $TARGETS; do
  highbin=$(find "fw/high-$T" -name '*combined.bin' | head -1)
  if [ -z "$highbin" ]; then
    echo "| \`$T\` | missing | ⚠️ firmware missing |" >> hil-out/comment.md; continue
  fi
  # Wait out a DUT-host reboot BEFORE submitting (the controller advertises
  # retry_after on a wedge/auto-reboot). Skip only on a permanent outage; a host
  # still down past the wait budget fails the run for that target.
  status=$(wait_for_target_available "$T"); wrc=$?
  dev=$(echo "$status" | awk '{print $2}')
  if [ "$wrc" -ne 0 ]; then
    why=$(echo "$status" | cut -d' ' -f2-)
    echo "| \`$T\` | — | ⏭️ skipped (${why}) |" >> hil-out/comment.md
    [ "$wrc" -eq 3 ] && fail=1   # host never came back within budget — not a clean skip
    continue
  fi
  ran=1; note=""
  run_target "$T" "$dev" "$highbin"; cv="$RT_VERDICT"
  # Reactive retry: a job that errored with a host-offline signature (host wedged
  # mid-job) — not a real ok=true/false — gets the host waited out + ONE re-run,
  # so even the test that triggered the wedge runs to a real verdict.
  if [ "$cv" != "true" ] && [ "$cv" != "false" ] \
     && is_host_offline_failure "hil-out/${T}-checkin.events.log" "$RT_STATE"; then
    echo "::warning::[$T/checkin] host-offline signature (state=$RT_STATE) — waiting for host + retrying once" >&2
    status=$(wait_for_target_available "$T"); wrc=$?
    if [ "$wrc" -eq 0 ]; then
      dev=$(echo "$status" | awk '{print $2}')
      run_target "$T" "$dev" "$highbin"; cv="$RT_VERDICT"; note=" (host rebooted — retried)"
    fi
  fi
  pass="❌"; if [ "$cv" = "true" ]; then pass="✅"; else fail=1; fi
  echo "| \`$T\` | flashed | ok=${cv} ${pass}${note} |" >> hil-out/comment.md
  append_proof "$T" checkin
done

{
  echo
  echo "_Expected: \`ok=true\` (device flashed + configured + checked in)._"
} >> hil-out/comment.md

[ "$ran" = 1 ] || echo "No available targets ran (all skipped)."
exit $fail
