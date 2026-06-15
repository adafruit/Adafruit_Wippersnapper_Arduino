#!/usr/bin/env bash
# Shared helpers for the HIL driver scripts (hil-checkin-run.sh,
# hil-pixelwrite-run.sh): ride through a DUT-host reboot between/within test runs.
#
# A CI HIL job runs several firmware-bench jobs in sequence. If the DUT host's USB
# stack wedges (dwc_otg) the controller flags its devices unavailable/temporary and
# advertises retry_after (= now + HIL_HOST_REBOOT_ETA_S) on GET /v1/targets, then
# auto-reboots the host (~3–5 min). Without these helpers the remaining tests
# submitted regardless and errored instantly with an SSH "no route to host" /
# "device unavailable" signature, reporting `unknown` — a transient-outage failure,
# not a logic bug (run #8). These helpers:
#   * wait_for_target_available <target> — re-poll /v1/targets before each job,
#     sleeping until retry_after (bounded) when the host is rebooting;
#   * is_host_offline_failure — classify a job that errored with a host-offline
#     signature (vs a real verdict) so the caller can re-submit that test once.
#
# Requires the caller to have set: API (controller base URL) and AUTH (curl -H
# auth header array). Reads/writes ./targets.json (the availability matrix).

# Bounded total wait for a host to come back. Default 360s covers a 3–5 min
# dwc_otg auto-reboot with margin. Override via HIL_WAIT_BUDGET_S.
HIL_WAIT_BUDGET_S="${HIL_WAIT_BUDGET_S:-360}"
# Extra slack added on top of retry_after before re-polling (the ETA is an
# estimate; the host may need a few more seconds). Override via HIL_WAIT_MARGIN_S.
HIL_WAIT_MARGIN_S="${HIL_WAIT_MARGIN_S:-15}"
# Fallback poll interval when no retry_after is advertised. Override via HIL_WAIT_POLL_S.
HIL_WAIT_POLL_S="${HIL_WAIT_POLL_S:-15}"

# Refresh targets.json from the controller and echo the record for one target
# (empty if the controller has no entry for it). Returns non-zero on a fetch error.
_hil_fetch_target_rec() {  # target -> stdout record json (or empty)
  curl -fsS "${AUTH[@]}" "${API}/v1/targets" > targets.json 2>/dev/null || return 1
  jq -c --arg t "$1" '.targets[]|select(.target==$t)' targets.json | head -1
}

# Wait until <target> is available, or decide to skip it. Re-polls /v1/targets;
# on a temporary outage sleeps until retry_after (+margin) and re-polls, bounded
# by HIL_WAIT_BUDGET_S. Echoes one of:
#   "available <device_id>"   (return 0) — proceed; device_id is the fresh value
#   "skip <kind> <reason>"    (return 2) — permanent outage / no controller entry
#   "timeout <reason>"        (return 3) — still down past the wait budget
wait_for_target_available() {  # target -> status line; return code per above
  local t="$1" rec avail dev kind reason ra now ra_epoch sleep_s
  local deadline=$(( $(date +%s) + HIL_WAIT_BUDGET_S ))
  while :; do
    rec=$(_hil_fetch_target_rec "$t") || rec=""
    if [ -z "$rec" ]; then
      # No record: either the fetch failed (controller briefly unreachable) or
      # the target genuinely isn't configured. Distinguish by whether we got a
      # valid targets.json with other entries.
      if [ -s targets.json ] && jq -e '.targets|length>0' targets.json >/dev/null 2>&1; then
        echo "skip none no-controller-entry"; return 2
      fi
      now=$(date +%s)
      if [ "$now" -ge "$deadline" ]; then echo "timeout controller-unreachable"; return 3; fi
      echo "[$t] controller unreachable; retrying in ${HIL_WAIT_POLL_S}s" >&2
      sleep "$HIL_WAIT_POLL_S"; continue
    fi
    avail=$(echo "$rec" | jq -r '.available')
    dev=$(echo "$rec"  | jq -r '.device_id')
    kind=$(echo "$rec" | jq -r '.kind // ""')
    reason=$(echo "$rec" | jq -r '.reason // ""')
    if [ "$avail" = "true" ]; then echo "available $dev"; return 0; fi
    if [ "$kind" = "permanent" ]; then echo "skip permanent ${reason}"; return 2; fi

    # Temporary (or unspecified-kind) outage — wait until retry_after, bounded.
    now=$(date +%s)
    if [ "$now" -ge "$deadline" ]; then echo "timeout ${reason}"; return 3; fi
    ra=$(echo "$rec" | jq -r '.retry_after // ""')
    sleep_s="$HIL_WAIT_POLL_S"
    if [ -n "$ra" ]; then
      ra_epoch=$(date -d "$ra" +%s 2>/dev/null || echo 0)
      if [ "$ra_epoch" -gt "$now" ]; then sleep_s=$(( ra_epoch - now + HIL_WAIT_MARGIN_S )); fi
    fi
    # Never sleep past the budget; always make some progress.
    if [ $(( now + sleep_s )) -gt "$deadline" ]; then sleep_s=$(( deadline - now )); fi
    [ "$sleep_s" -lt 1 ] && sleep_s=1
    echo "[$t] unavailable (${kind:-temporary}: ${reason}); waiting ${sleep_s}s for host (retry_after=${ra:-none})" >&2
    sleep "$sleep_s"
  done
}

# Classify a finished job as a transient host-offline failure (vs a real verdict
# or a genuine logic failure). True (return 0) when the job did NOT run to a clean
# finish AND its captured events carry a host-offline / device-unavailable / wedge
# signature — i.e. the host went away mid-job (or was rejected by the get_adapter
# gate) rather than producing a verdict. The caller treats this as "wait + retry".
is_host_offline_failure() {  # events_file, terminal_state -> 0 if transient
  local ef="$1" state="$2"
  # A clean finish produced a real result — never a transient retry.
  case "$state" in finished|cancelled) return 1;; esac
  [ -f "$ef" ] || return 1
  grep -qiE 'no route to host|timed out|timeout|connection refused|connection (closed|reset)|ssh.*(timeout|closed|reset|unreachable)|device .* unavailable|reboot required|host (usb )?(stack )?wedged|host rebooting|blocked by lease' "$ef"
}
