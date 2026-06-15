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
  # Prefer an AVAILABLE device when several DUTs share this build_target (e.g. the
  # same chip on two hosts) — pick the first available, else the first of any (so a
  # genuine outage/skip is still reported). Lets a 2nd DUT on another host serve the
  # target while the first is down for maintenance.
  jq -c --arg t "$1" '[.targets[]|select(.target==$t)] | (map(select(.available)) + .)[0] // empty' targets.json
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

# True (return 0) when a job ended in an INFRA/harness error state — error,
# timeout, or failed — i.e. it never produced a real verdict (a real
# firmware-behaviour verdict comes out as state=finished). The driver retries on
# this regardless of whether a host-offline signature made it into the events log,
# because the error reason often lands AFTER the state→terminal event (and is
# sometimes empty), so signature-matching alone misses it (run #10: pixelWrite
# LOW errored with an empty reason, HIGH with "No route to host" that the loop
# broke before capturing). A wait_for_target_available precedes the re-submit.
is_infra_error() {  # terminal_state -> 0 if an infra/harness error (no real verdict)
  case "$1" in error|timeout|failed|"") return 0;; *) return 1;; esac
}

# Lines of context shown before/after the matched test phrase in a proof window.
HIL_PROOF_BEFORE="${HIL_PROOF_BEFORE:-24}"
HIL_PROOF_AFTER="${HIL_PROOF_AFTER:-6}"

# Print the evidence WINDOW from a log: the lines leading up to and just after the
# LAST line matching <regex> (the detection point) — so the quote actually shows the
# expected data, not a blind tail. Returns 0 if matched, 1 if it fell back to tail.
proof_window() {  # file, regex
  local f="$1" re="$2" ln start
  ln=$(grep -nE "$re" "$f" 2>/dev/null | tail -1 | cut -d: -f1)
  if [ -n "$ln" ]; then
    start=$((ln - HIL_PROOF_BEFORE)); [ "$start" -lt 1 ] && start=1
    sed -n "${start},$((ln + HIL_PROOF_AFTER))p" "$f"
    return 0
  fi
  tail -n 25 "$f"
  return 1
}

# Append per-(target,test) proof to the comment: a SEPARATE collapsible section for
# serial.log AND protomq.log, each windowed around <evidence_regex> (the test
# phrase), plus a one-line index of the downloadable per-log artifacts. A log that
# wasn't captured is called out explicitly (not silently dropped).
#   append_proof <target> <test-label> <evidence_regex>
append_proof() {
  local t="$1" label="$2" re="$3" type f win note
  local run_url="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}"
  for type in serial protomq; do
    f="hil-out/${t}-${label}-${type}.log"
    if [ ! -s "$f" ]; then
      printf '\n> ⚠️ `%s` %s — `%s.log` not captured\n' "$t" "$label" "$type" >> hil-out/comment.md
      continue
    fi
    if win=$(proof_window "$f" "$re"); then note="✓ around the detected test phrase"; else note="⚠️ test phrase not found — tail shown"; fi
    {
      printf '\n<details><summary>📜 `%s` %s · %s.log (%s)</summary>\n\n' "$t" "$label" "$type" "$note"
      echo '```'; printf '%s\n' "$win"; echo '```'
      echo "</details>"
    } >> hil-out/comment.md
  done
  # Per-log artifact index (each log is uploaded as its own artifact — see below).
  printf '\n<sub>logs for `%s/%s`: serial / protomq / flash — in the per-log [Artifacts](%s#artifacts)</sub>\n' \
    "$t" "$label" "$run_url" >> hil-out/comment.md
}
