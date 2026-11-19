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

# Context shown around the matched test phrase in a proof window.
#   HIL_PROOF_BEFORE: lines BEFORE the phrase. -1 (the default) = from boot — show the
#     whole device/broker story up to the phrase, not just a fixed slice. N>=0 = N lines.
#   HIL_PROOF_AFTER:  lines after the phrase ("just after" the detection point).
HIL_PROOF_BEFORE="${HIL_PROOF_BEFORE:--1}"
HIL_PROOF_AFTER="${HIL_PROOF_AFTER:-6}"

# Print the evidence WINDOW from a log: the lines leading up to and just after the
# LAST line matching <regex> (the detection point) — so the quote actually shows the
# expected data, not a blind tail. HIL_PROOF_BEFORE<0 → from boot (line 1). Records the
# window's time span in PW_TS_START/PW_TS_END (leading UTC-ms timestamps of its first/
# last lines) so the OTHER log can be aligned to the SAME wall-clock span. Returns 0 if
# matched, 1 if it fell back to a tail.
proof_window() {  # file, regex
  local f="$1" re="$2" ln start end
  PW_TS_START=""; PW_TS_END=""
  ln=$(grep -nE "$re" "$f" 2>/dev/null | tail -1 | cut -d: -f1)
  if [ -n "$ln" ]; then
    if [ "${HIL_PROOF_BEFORE}" -lt 0 ]; then start=1
    else start=$((ln - HIL_PROOF_BEFORE)); [ "$start" -lt 1 ] && start=1; fi
    end=$((ln + HIL_PROOF_AFTER))
    PW_TS_START=$(sed -n "${start}p" "$f" | awk '{print $1}')
    PW_TS_END=$(sed -n "${end}p" "$f" | awk '{print $1}')
    [ -z "$PW_TS_END" ] && PW_TS_END=$(awk 'END{print $1}' "$f")   # window ran past EOF
    sed -n "${start},${end}p" "$f"
    return 0
  fi
  tail -n 25 "$f"
  return 1
}

# Print only the lines of <file> whose leading UTC-ms timestamp falls in [ts_a, ts_b].
# serial/protomq/flash logs share ONE clock (record() stamps every line), and fixed-
# width ISO-8601 UTC timestamps sort lexicographically = chronologically, so a plain
# string compare on field 1 selects the wall-clock window. This is what makes the
# protomq quote LINE UP with the serial quote instead of drifting (a fixed line count
# spans wildly different durations in a chatty broker log vs a sparse serial log).
time_window() {  # file, ts_a, ts_b
  local f="$1" a="$2" b="$3"
  [ -z "$a" ] || [ -z "$b" ] && { tail -n 25 "$f"; return 1; }
  awk -v a="$a" -v b="$b" '$1 >= a && $1 <= b' "$f"
}

_proof_section() {  # target, label, type, note, window
  {
    printf '\n<details><summary>📜 `%s` %s · %s.log (%s)</summary>\n\n' "$1" "$2" "$3" "$4"
    echo '```'; printf '%s\n' "$5"; echo '```'
    echo "</details>"
  } >> hil-out/comment.md
}

# Append per-(target,test) proof to the comment: a SEPARATE collapsible section for
# serial.log AND protomq.log + a one-line index of the per-log artifacts. The SERIAL
# section is the device-side reference (from boot by default); the PROTOMQ section is
# windowed to the SAME wall-clock span as the serial quote (shared UTC-ms clock) so the
# broker handshake lines up with the serial events instead of drifting to a different
# minute. A log that wasn't captured is called out explicitly (not silently dropped).
#   append_proof <target> <test-label> <evidence_regex>
append_proof() {
  local t="$1" label="$2" re="$3" win note
  local run_url="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}"
  local sfile="hil-out/${t}-${label}-serial.log" pfile="hil-out/${t}-${label}-protomq.log"
  local ts_a="" ts_b=""

  # SERIAL — the device side, and the time reference the protomq quote aligns to.
  if [ -s "$sfile" ]; then
    if win=$(proof_window "$sfile" "$re"); then
      if [ "${HIL_PROOF_BEFORE}" -lt 0 ]; then note="✓ from boot to just after the detected test phrase"
      else note="✓ around the detected test phrase"; fi
    else note="⚠️ test phrase not found — tail shown"; fi
    # Derive the window's time span from the CAPTURED text — proof_window ran in a
    # subshell (command substitution), so its PW_TS_* globals don't reach us here.
    # First/last stamped line's leading UTC-ms timestamp = the span protomq aligns to.
    ts_a=$(printf '%s\n' "$win" | awk 'NF{print $1; exit}')
    ts_b=$(printf '%s\n' "$win" | awk 'NF{ts=$1} END{print ts}')
    _proof_section "$t" "$label" serial "$note" "$win"
  else
    printf '\n> ⚠️ `%s` %s — `serial.log` not captured\n' "$t" "$label" >> hil-out/comment.md
  fi

  # PROTOMQ — aligned to the serial window's wall-clock span. Falls back to its own
  # evidence window only if serial gave us no anchor (serial absent / phrase not found).
  if [ -s "$pfile" ]; then
    if [ -n "$ts_a" ] && [ -n "$ts_b" ]; then
      win=$(time_window "$pfile" "$ts_a" "$ts_b")
      if [ -n "$win" ]; then note="⏱ aligned to the serial window (${ts_a} … ${ts_b})"
      else win=$(proof_window "$pfile" "$re"); note="⚠️ no broker traffic in the serial window — evidence window shown"; fi
    else
      if win=$(proof_window "$pfile" "$re"); then note="✓ around the detected test phrase"; else note="⚠️ test phrase not found — tail shown"; fi
    fi
    _proof_section "$t" "$label" protomq "$note" "$win"
  else
    printf '\n> ⚠️ `%s` %s — `protomq.log` not captured\n' "$t" "$label" >> hil-out/comment.md
  fi

  printf '\n<sub>logs for `%s/%s`: serial / protomq / flash — in the per-log [Artifacts](%s#artifacts)</sub>\n' \
    "$t" "$label" "$run_url" >> hil-out/comment.md
}
