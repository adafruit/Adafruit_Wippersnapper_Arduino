#!/usr/bin/env python3
"""Generic WipperSnapper HIL test driver (firmware-bench, camera proof).

Drives the usbip-hil-controller through the proven firmware-bench pipeline —
flash a combined image, confirm a v2 broker check-in, then run an arbitrary
**sequence** of injects/captures — and harvests a uniform proof bundle (process
invocation, the controller's command transcript, serial.log, protomq.log, and a
visual proof image), plus a Markdown report a CI job can post to the PR.

It is NOT LilyGo- or display-specific: the test is described by a JSON **spec**
(see ``--spec`` / ``specs/*.json``) whose ``sequence`` is a list of steps. Each
step maps to one controller stage, so future test writers can drive any board,
any display driver, or non-display flows (add an I2C sensor component, write to a
display, add another, capture, …) without touching this file.

Step kinds (all optional ``settle_s`` seconds held after the stage):
  - ``display_add``   i8080/SPI display Add. Fields: name, driver, interface
                      (i8080|spi), data_pins[], cs, dc, rst, width, height,
                      rotation, text_size, status_bar. (interface=i8080 uses the
                      controller's ``display_add_i8080`` builder; otherwise pass
                      an explicit ``kind`` or ``payload_hex``.)
  - ``display_write`` Write text to a named display. Fields: name, message.
  - ``component_add`` / ``inject``  Generic ``ws.signal.BrokerToDevice`` inject —
                      give a controller builder ``kind`` + ``params`` (e.g. an
                      I2C sensor add, once a builder exists) OR raw ``payload_hex``.
  - ``capture``       Camera proof. Fields: exposure_us, gain, roi
                      [x,y,w,h,frame_w,frame_h], focus_lock, white_balance, out.

Top-level spec fields: device, uid, camera_url, window_minutes, skip_flash,
secrets (names only — values come from env), sequence[]. CLI flags override the
spec's device/uid/camera_url/window_minutes and supply the firmware path.

Env: HIL_API_BASE, HIL_API_TOKEN (required); IO_USERNAME/IO_KEY/WIFI_SSID/
WIFI_PASSWORD (secret values, defaulted to bench values if unset).

Exit 0 iff CHECKIN ok AND every inject published AND every requested capture
produced an asset.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request


# --- protobuf wire helpers (varint / length-delimited) ----------------------
def _varint(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _lenfield(field: int, payload: bytes) -> bytes:
    return _varint((field << 3) | 2) + _varint(len(payload)) + payload


def display_write_signal(name: str, message: str) -> bytes:
    """ws.signal.BrokerToDevice(display=36) -> ws.display.B2D(write=3)
    -> ws.display.Write{name=1, message=3}."""
    write = _lenfield(1, name.encode()) + _lenfield(3, message.encode())
    return _lenfield(36, _lenfield(3, write))


# --- HTTP --------------------------------------------------------------------
def _req(method: str, url: str, token: str, body: bytes | None = None,
         ctype: str = "application/json", timeout: float = 60.0):
    req = urllib.request.Request(url, data=body, method=method)
    req.add_header("Authorization", f"Bearer {token}")
    if body is not None:
        req.add_header("Content-Type", ctype)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status, r.read()


# --- spec -> stages ----------------------------------------------------------
def _display_add_stage(step: dict, uid: str) -> dict:
    interface = step.get("interface", "i8080")
    stage = {"type": "inject_protobuf", "settle_s": step.get("settle_s", 5), "uid": uid}
    if "payload_hex" in step:
        stage["payload_hex"] = step["payload_hex"]
        return stage
    kind = step.get("kind")
    if not kind:
        if interface == "i8080":
            kind = "display_add_i8080"
        else:
            raise ValueError(
                f"display_add interface={interface!r} has no built-in builder; "
                "supply a controller 'kind' or 'payload_hex'")
    params = {k: step[k] for k in (
        "name", "driver", "data_pins", "cs", "dc", "rst", "sck", "mosi",
        "wr", "rd", "backlight", "power",
        "width", "height", "rotation", "text_size", "status_bar") if k in step}
    stage["kind"] = kind
    stage["params"] = params
    return stage


def build_stages(spec: dict, uid: str, camera_url: str, captures: list[str]) -> list[dict]:
    """Translate spec.sequence into controller firmware-bench stages, prefixed
    with the standard flash/secrets/check-in prologue (unless skip_flash)."""
    stages: list[dict] = []
    if not spec.get("skip_flash"):
        stages += [
            {"type": "enter_bootloader"},
            {"type": "erase", "before": "no_reset", "after": "no_reset"},
            {"type": "flash", "offset": "0x0", "before": "no_reset", "after": "no_reset"},
            {"type": "power_cycle"},
            {"type": "write_secrets_msc"},
            {"type": "power_cycle"},
        ]
    stages.append({"type": "verify_checkin", "checkin_timeout_s": 150,
                   "proto": "auto", "soft": True})

    for i, step in enumerate(spec.get("sequence", [])):
        kind = step.get("step")
        if kind == "display_add":
            stages.append(_display_add_stage(step, uid))
        elif kind == "display_write":
            stages.append({"type": "inject_protobuf", "uid": uid,
                           "settle_s": step.get("settle_s", 5),
                           "payload_hex": display_write_signal(
                               step["name"], step["message"]).hex()})
        elif kind in ("inject", "component_add"):
            stage = {"type": "inject_protobuf", "uid": uid,
                     "settle_s": step.get("settle_s", 5)}
            if "payload_hex" in step:
                stage["payload_hex"] = step["payload_hex"]
            else:
                stage["kind"] = step["kind"]
                stage["params"] = step.get("params", {})
            stages.append(stage)
        elif kind == "capture":
            out = step.get("out", f"/tmp/hil-capture-{i}.jpg")
            captures.append(out)
            cap = {"type": "capture_display", "camera_url": camera_url, "out": out}
            for k in ("exposure_us", "gain", "roi", "focus_lock",
                      "focus_position", "white_balance", "autofocus", "settle_s"):
                if k in step:
                    cap[k] = step[k]
            stages.append(cap)
        else:
            raise ValueError(f"sequence[{i}]: unknown step {kind!r}")
    return stages


# --- reporting (conforms to PR #930 hil-lib.sh append_proof) -----------------
# Logs share one UTC-ms clock (the controller's record() stamps every line), and
# fixed-width ISO-8601 timestamps sort lexicographically = chronologically, so a
# plain string compare on field 1 selects a wall-clock window. We anchor on the
# serial proof window, then quote protomq for the SAME span so the broker
# handshake lines up with the serial events instead of drifting.
HIL_PROOF_BEFORE = int(os.environ.get("HIL_PROOF_BEFORE", "-1"))  # <0 = from boot
HIL_PROOF_AFTER = int(os.environ.get("HIL_PROOF_AFTER", "6"))


def _proof_window(lines: list[str], regex: str):
    """Lines up to + just after the LAST line matching regex. Returns
    (window, ts_a, ts_b, matched); falls back to the last 25 lines if no match."""
    import re as _re
    hits = [i for i, ln in enumerate(lines) if _re.search(regex, ln)]
    if not hits:
        return lines[-25:], None, None, False
    ln = hits[-1]
    start = 0 if HIL_PROOF_BEFORE < 0 else max(0, ln - HIL_PROOF_BEFORE)
    end = min(len(lines), ln + HIL_PROOF_AFTER + 1)
    win = lines[start:end]
    ts_a = win[0].split(" ", 1)[0] if win and win[0].split() else None
    ts_b = None
    for w in win:
        p = w.split(" ", 1)
        if p and p[0]:
            ts_b = p[0]
    return win, ts_a, ts_b, True


def _time_window(lines: list[str], ts_a: str | None, ts_b: str | None):
    if not ts_a or not ts_b:
        return lines[-25:]
    out = [ln for ln in lines if (p := ln.split(" ", 1)) and p[0] and ts_a <= p[0] <= ts_b]
    return out or lines[-25:]


def _proof_section(target: str, label: str, logtype: str, note: str, body: list[str]) -> str:
    text = "\n".join(body).strip() or "(empty)"
    if len(text) > 14000:  # keep the PR comment under GitHub's 65536-char limit
        text = "…(truncated — full log in the Artifacts)…\n" + text[-14000:]
    return (f"\n<details><summary>📜 `{target}` {label} · {logtype}.log ({note})"
            f"</summary>\n\n```\n{text}\n```\n</details>\n")


def append_comment(path: str, *, ok: bool, argv: list[str], job_id: str, target: str,
                   label: str, spec_name: str, stages: list[dict], verdicts: dict[str, str],
                   serial_lines: list[str], protomq_lines: list[str], evidence_re: str,
                   proof_imgs: list[str], img_url_base: str | None, run_url: str) -> None:
    """Append this test's section to hil-out/comment.md (the suite's shared
    summary). Mirrors hil-lib.sh: a result row, time-aligned serial+protomq proof
    windows, a per-log artifacts index — plus the display-only visual proof and
    the invocation/command-plan (so the report carries process+args too)."""
    new = not os.path.exists(path)
    out: list[str] = []
    if new:  # standalone header (the suite writes its own and we just append)
        out += [f"## 🔌 HIL display proof", "",
                f"full logs (serial.log / protomq.log / flash.log) in the run **Artifacts**", ""]
    cv = "true" if "ok=true" in verdicts.get("CHECKIN_VERDICT", "") else \
         ("false" if "ok=false" in verdicts.get("CHECKIN_VERDICT", "") else "unknown")
    cap = "✅" if proof_imgs else "⚠️"
    out += ["", f"### {'✅' if ok else '❌'} Display proof — `{spec_name}`", "",
            "| target | check-in | injects | capture |", "|---|---|---|---|",
            f"| `{target}` | ok={cv} | {verdicts.get('INJECT_VERDICT','').count('published=true')} published | {cap} |", ""]
    if proof_imgs:
        out.append("**Visual proof:**")
        for img in proof_imgs:
            name = os.path.basename(img)
            out.append(f"![{target} {label}]({img_url_base.rstrip('/')}/{name})"
                       if img_url_base else f"- `{name}` (in Artifacts)")
        out.append("")
    # serial — device side, the time anchor; protomq — windowed to the same span
    swin, ts_a, ts_b, matched = _proof_window(serial_lines, evidence_re)
    snote = ("✓ from boot to just after the detected test phrase" if matched and HIL_PROOF_BEFORE < 0
             else "✓ around the detected test phrase" if matched
             else "⚠️ test phrase not found — tail shown")
    if serial_lines:
        out.append(_proof_section(target, label, "serial", snote, swin))
    else:
        out.append(f"\n> ⚠️ `{target}` {label} — `serial.log` not captured\n")
    if protomq_lines:
        if ts_a and ts_b:
            pwin = _time_window(protomq_lines, ts_a, ts_b)
            pnote = f"⏱ aligned to the serial window ({ts_a} … {ts_b})"
        else:
            pwin, _, _, m = _proof_window(protomq_lines, evidence_re)
            pnote = "✓ around the detected test phrase" if m else "⚠️ test phrase not found — tail shown"
        out.append(_proof_section(target, label, "protomq", pnote, pwin))
    else:
        out.append(f"\n> ⚠️ `{target}` {label} — `protomq.log` not captured\n")
    out.append(f"\n<sub>logs for `{target}/{label}`: serial / protomq / flash — "
               f"in the per-log [Artifacts]({run_url}#artifacts)</sub>\n")
    # invocation + the controller command plan (so the report carries process+args)
    out.append(f"\n<details><summary>🧾 invocation + submitted stages</summary>\n\n"
               f"```\n$ {' '.join(argv)}\n```\n\n```json\n{json.dumps(stages, indent=2)}\n```\n</details>\n")
    with open(path, "a", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--combined", required=True, help="flashable combined.bin")
    ap.add_argument("--spec", required=True, help="path to a JSON test spec")
    ap.add_argument("--device", default="")
    ap.add_argument("--uid", default="")
    ap.add_argument("--camera-url", default="")
    ap.add_argument("--target", default="",
                    help="build-target label for log/report naming (default: spec stem)")
    ap.add_argument("--label", default="display",
                    help="test label for log/report naming (<target>-<label>-<type>.log)")
    ap.add_argument("--evidence-re",
                    default=("DISPLAY_CAPTURE_VERDICT|Display added successfully|"
                             "i8080 ST7789 initialized|ST7789 .*initialized|Adding display|"
                             "CHECKIN_VERDICT|RegistrationComplete|totalGpioPins"),
                    help="regex anchoring the serial proof window (protomq aligns to it)")
    ap.add_argument("--out-dir", default="hil-out")
    ap.add_argument("--window-minutes", type=int, default=0)
    ap.add_argument("--deadline-s", type=int, default=900)
    ap.add_argument("--img-url-base", default="",
                    help="base URL where proof images will be hosted (for the report)")
    args = ap.parse_args()

    base = (os.environ.get("HIL_API_BASE") or "").rstrip("/")
    token = os.environ.get("HIL_API_TOKEN") or ""
    if not base or not token:
        print("ERROR: set HIL_API_BASE and HIL_API_TOKEN", file=sys.stderr)
        return 2
    os.makedirs(args.out_dir, exist_ok=True)

    with open(args.spec, encoding="utf-8") as f:
        spec = json.load(f)
    spec_name = os.path.basename(args.spec)
    device = args.device or spec.get("device") or "mcu-lilygo-tdisplay-s3-hil006"
    uid = args.uid or spec.get("uid") or ""
    camera_url = args.camera_url or spec.get("camera_url") or "http://rpi-hil006:8080/"
    window = args.window_minutes or spec.get("window_minutes", 12)
    target = args.target or spec.get("target") or os.path.splitext(spec_name)[0]
    label = args.label

    print(f"$ {' '.join(sys.argv)}")
    print(f"spec={spec_name} device={device} uid={uid or '(from checkin)'} "
          f"camera={camera_url} window={window}m")

    # 1) upload firmware
    blob = open(args.combined, "rb").read()
    st, body = _req("POST",
                    f"{base}/v1/firmware?filename={os.path.basename(args.combined)}",
                    token, blob, ctype="application/octet-stream", timeout=180)
    fw_path = json.loads(body)["path"]
    print(f"uploaded firmware ({len(blob)} bytes) -> {fw_path}")

    # 2) build stages from the spec
    captures: list[str] = []
    stages = build_stages(spec, uid, camera_url, captures)
    secrets = {k: os.environ.get(k, "hil") for k in ("IO_USERNAME", "IO_KEY")}
    secrets["WIFI_SSID"] = os.environ.get("WIFI_SSID", "free4all")
    secrets["WIFI_PASSWORD"] = os.environ.get("WIFI_PASSWORD", "password")
    job = {
        "target": {"device": {"id": device}, "pool": "public"},
        "script": "firmware-bench",
        "params": {
            "firmware": {"path": fw_path, "offset": "0x0"},
            "window_minutes": window,
            "collect_artifacts": captures,
            "stages": stages,
        },
        "secrets": secrets,
    }
    st, body = _req("POST", f"{base}/v1/jobs", token, json.dumps(job).encode())
    job_id = json.loads(body)["id"]
    print(f"submitted job {job_id} ({len(stages)} stages, {len(captures)} captures)")

    # 3) stream events -> transcript + verdicts (serial captured as fallback).
    # When every expected verdict is in, cancel the job to end the window hold
    # early: the worker harvests collect_artifacts in its finally block (incl. on
    # cancel), so this is how the proof image/logs become downloadable assets
    # without waiting out window_minutes.
    n_inject = len([s for s in stages if s["type"] == "inject_protobuf"])
    n_capture = len(captures)
    verdicts: dict[str, str] = {}
    transcript_lines: list[str] = []
    serial_lines: list[str] = []
    since, deadline, state = 0, time.time() + args.deadline_s, "pending"
    saw_capture = saw_inject = 0
    ended_hold = False
    while time.time() < deadline:
        st, body = _req("GET", f"{base}/v1/jobs/{job_id}/wait?since={since}&timeout=20",
                        token, timeout=40)
        d = json.loads(body)
        since = d.get("next_since", since)
        for e in d.get("events", []):
            p = e.get("payload", {})
            m = str(p.get("msg", "") or "")
            if p.get("stream") == "serial":
                if m:
                    serial_lines.append(m.rstrip("\n"))
                continue
            if m:
                transcript_lines.append(m.rstrip("\n"))
            for key in ("CHECKIN_VERDICT", "INJECT_VERDICT", "DISPLAY_CAPTURE_VERDICT"):
                if key in m:
                    verdicts[key] = (verdicts.get(key, "") + m.strip() + "\n")
                    if key == "DISPLAY_CAPTURE_VERDICT":
                        saw_capture += 1
                    if key == "INJECT_VERDICT" and "published=true" in m:
                        saw_inject += 1
                    print(f"  {m.strip()[:160]}")
        state = d.get("state", state)
        if state in ("finished", "failed", "cancelled", "error", "timeout"):
            break
        # all expected verdicts seen -> end the hold so artifacts harvest now
        if (not ended_hold and "CHECKIN_VERDICT" in verdicts
                and saw_inject >= n_inject and saw_capture >= n_capture):
            try:
                _req("POST", f"{base}/v1/jobs/{job_id}/cancel", token, timeout=15)
                print("  all verdicts in; ending hold to harvest artifacts")
            except Exception as exc:  # noqa: BLE001
                print(f"  end-hold failed: {exc}")
            ended_hold = True
    print(f"job state={state}")

    # 4) download harvested assets. firmware-bench registers the serial/protomq/
    # flash logs (kind=log) + the capture (kind=file) at teardown — which the
    # cancel above triggered — so retry the listing until they appear. Files are
    # named <target>-<label>-<filename> to match the suite's per-log artifacts.
    proof_imgs: list[str] = []
    serial_asset = protomq_asset = ""
    for attempt in range(8):
        proof_imgs, serial_asset, protomq_asset = [], "", ""
        try:
            _, body = _req("GET", f"{base}/v1/jobs/{job_id}/assets", token)
            for a in json.loads(body).get("assets", []):
                name = a.get("name") or a.get("filename") or a.get("id")
                aid = a.get("id") or a.get("asset_id")
                if not aid or a.get("kind") == "firmware":
                    continue
                try:  # the controller serves bytes at .../assets/<id>/download
                    _, content = _req("GET",
                                      f"{base}/v1/jobs/{job_id}/assets/{aid}/download",
                                      token, timeout=120)
                except Exception as exc:  # noqa: BLE001
                    print(f"  asset {name}: {exc}")
                    continue
                dest = os.path.join(args.out_dir, f"{target}-{label}-{name}")
                open(dest, "wb").write(content)
                low = name.lower()
                if low.endswith((".jpg", ".jpeg", ".png")):
                    proof_imgs.append(dest)
                elif "serial" in low:
                    serial_asset = content.decode("utf-8", "replace")
                elif "protomq" in low or "broker" in low:
                    protomq_asset = content.decode("utf-8", "replace")
        except Exception as exc:  # noqa: BLE001
            print(f"asset listing failed: {exc}")
        if (proof_imgs and serial_asset) or attempt == 7:
            break
        time.sleep(3)  # assets register at teardown, just after the cancel
    for img in proof_imgs:
        print(f"  pulled proof {os.path.basename(img)}")

    # 5) write the events transcript + assemble the conformant comment.md section.
    transcript = "\n".join(transcript_lines)
    open(os.path.join(args.out_dir, f"{target}-{label}.events.log"),
         "w", encoding="utf-8").write(transcript)
    serial_text = serial_asset or "\n".join(serial_lines)
    protomq_text = protomq_asset
    serial_log_lines = serial_text.splitlines()
    protomq_log_lines = protomq_text.splitlines()

    checkin_ok = "ok=true" in verdicts.get("CHECKIN_VERDICT", "")
    injects = [s for s in stages if s["type"] == "inject_protobuf"]
    inject_ok = verdicts.get("INJECT_VERDICT", "").count("published=true") >= len(injects)
    captured_ok = (saw_capture >= len(captures))
    ok = checkin_ok and inject_ok and captured_ok

    run_url = (f"{os.environ.get('GITHUB_SERVER_URL','https://github.com')}/"
               f"{os.environ.get('GITHUB_REPOSITORY','')}/actions/runs/"
               f"{os.environ.get('GITHUB_RUN_ID','')}")
    append_comment(os.path.join(args.out_dir, "comment.md"), ok=ok, argv=sys.argv,
                   job_id=job_id, target=target, label=label, spec_name=spec_name,
                   stages=stages, verdicts=verdicts, serial_lines=serial_log_lines,
                   protomq_lines=protomq_log_lines, evidence_re=args.evidence_re,
                   proof_imgs=proof_imgs, img_url_base=(args.img_url_base or None),
                   run_url=run_url)

    print(f"RESULT checkin_ok={checkin_ok} injects_ok={inject_ok} "
          f"captured={saw_capture}/{len(captures)} -> {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
