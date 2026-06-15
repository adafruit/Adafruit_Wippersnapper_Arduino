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

run_side() {  # side, target, device_id, fw_path -> echoes verdict (true|false|unknown)
  local side="$1" t="$2" dev="$3" fw="$4"
  local fid path jid
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
  echo "[$t/$side] terminal state: ${state:-unknown}" >&2
  # Pull ONLY the captured log assets (serial/protomq/flash) — skip the firmware bin.
  curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/assets" \
    | jq -r '.assets[]? | select(.kind=="log") | "\(.id) \(.filename)"' \
    | while read -r aid fn; do
        [ -n "$aid" ] && curl -fsS "${AUTH[@]}" "${API}/v1/jobs/${jid}/assets/${aid}/download" \
          -o "hil-out/${t}-${side}-${fn}" || true
      done
  echo "::endgroup::" >&2
  echo "$verdict"
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
  rec=$(jq -c --arg t "$T" '.targets[]|select(.target==$t)' targets.json | head -1)
  if [ -z "$rec" ]; then echo "| \`$T\` | — | — | ⚠️ no controller entry |" >> hil-out/comment.md; continue; fi
  avail=$(echo "$rec" | jq -r '.available'); dev=$(echo "$rec" | jq -r '.device_id')
  kind=$(echo "$rec" | jq -r '.kind // ""'); reason=$(echo "$rec" | jq -r '.reason // ""')
  if [ "$avail" != "true" ]; then
    echo "| \`$T\` | — | — | ⏭️ skipped (${kind}: ${reason}) |" >> hil-out/comment.md; continue
  fi
  lowbin=$(find "fw/low-$T"  -name '*combined.bin' | head -1)
  highbin=$(find "fw/high-$T" -name '*combined.bin' | head -1)
  if [ -z "$lowbin" ] || [ -z "$highbin" ]; then
    lo=$([ -n "$lowbin" ] && echo ok || echo missing); hi=$([ -n "$highbin" ] && echo ok || echo missing)
    echo "| \`$T\` | $lo | $hi | ⚠️ firmware missing |" >> hil-out/comment.md; continue
  fi
  ran=1
  lv=$(run_side low "$T" "$dev" "$lowbin"); hv=$(run_side high "$T" "$dev" "$highbin")
  pass="❌"; if [ "$lv" = "true" ] && [ "$hv" = "false" ]; then pass="✅"; else fail=1; fi
  echo "| \`$T\` | rebooted=${lv} | rebooted=${hv} | ${pass} |" >> hil-out/comment.md
  append_proof "$T" low
  append_proof "$T" high
done

{
  echo
  echo "_Expected: release \`rebooted=true\` (crash), PR \`rebooted=false\` (graceful)._"
} >> hil-out/comment.md

[ "$ran" = 1 ] || echo "No available targets ran (all skipped)."
exit $fail
