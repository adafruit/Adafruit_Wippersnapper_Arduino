#!/usr/bin/env python3
"""Drive the LilyGo T-Display-S3 i8080 display HIL test on the usbip-hil-controller.

Codifies the pipeline proven by hand: flash the (tinyuf2 + WipperSnapper) combined
image, confirm a **v2** broker check-in, inject a v2 ``ws.display.Add`` (i8080
ST7789) over the broker's ``/api/echo``, then capture a camera proof of the lit
panel (autofocus + manual exposure + ROI crop) and download it as a job asset.

It talks only to the controller's HTTP API (``HIL_API_BASE`` + bearer
``HIL_API_TOKEN``); the controller owns the bench (flashing, broker, camera).

Env / args:
  HIL_API_BASE   controller base URL (e.g. http://192.168.1.169:8080)
  HIL_API_TOKEN  bearer token
  --combined     path to the flashable combined.bin (tinyuf2 bootloader +
                 boot_app0 + WS app @0x10000 + tinyuf2 @uf2); see build-combined.
  --device       device id (default mcu-lilygo-tdisplay-s3-hil006)
  --uid          WS device uid for the display Add topic
                 (default lilygo-t-display-s3<macUID>; pass the board's)
  --camera-url   CSI camera-server base (default http://rpi-hil006:8080/)
  --roi          "x,y,w,h,frame_w,frame_h" ROI for the panel crop
  --out-dir      where to save downloaded proof assets (default ./hil-out)

Exit 0 iff CHECKIN_VERDICT ok=true AND a display capture asset was produced.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request


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


def _req(method: str, url: str, token: str, body: bytes | None = None,
         ctype: str = "application/json", timeout: float = 60.0):
    req = urllib.request.Request(url, data=body, method=method)
    req.add_header("Authorization", f"Bearer {token}")
    if body is not None:
        req.add_header("Content-Type", ctype)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status, r.read()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--combined", required=True)
    ap.add_argument("--device", default="mcu-lilygo-tdisplay-s3-hil006")
    ap.add_argument("--uid", default="")
    ap.add_argument("--camera-url", default="http://rpi-hil006:8080/")
    ap.add_argument("--roi", default="1270,770,235,135,2304,1296")
    ap.add_argument("--exposure-us", type=int, default=32000)
    ap.add_argument("--gain", type=float, default=3.0)
    ap.add_argument("--message",
                    default="LilyGo T-Display-S3\nWipperSnapper v2\ni8080 ST7789\nHIL camera proof OK")
    ap.add_argument("--out-dir", default="hil-out")
    ap.add_argument("--window-minutes", type=int, default=10)
    ap.add_argument("--deadline-s", type=int, default=900)
    args = ap.parse_args()

    base = (os.environ.get("HIL_API_BASE") or "").rstrip("/")
    token = os.environ.get("HIL_API_TOKEN") or ""
    if not base or not token:
        print("ERROR: set HIL_API_BASE and HIL_API_TOKEN", file=sys.stderr)
        return 2
    os.makedirs(args.out_dir, exist_ok=True)

    # 1) upload the combined firmware
    blob = open(args.combined, "rb").read()
    st, body = _req("POST", f"{base}/v1/firmware?filename=lilygo_t_display_s3.combined.bin",
                    token, blob, ctype="application/octet-stream", timeout=180)
    fw_path = json.loads(body)["path"]
    print(f"uploaded firmware ({len(blob)} bytes) -> {fw_path}")

    # 2) submit the firmware-bench job (the proven recipe)
    roi = [int(x) for x in args.roi.split(",")]
    secrets = {k: os.environ.get(k, "hil") for k in ("IO_USERNAME", "IO_KEY")}
    secrets["WIFI_SSID"] = os.environ.get("WIFI_SSID", "free4all")
    secrets["WIFI_PASSWORD"] = os.environ.get("WIFI_PASSWORD", "password")
    display_params = {
        "name": "tft-19-i8080", "driver": "ST7789",
        "data_pins": ["D39", "D40", "D41", "D42", "D45", "D46", "D47", "D48"],
        "cs": "D6", "dc": "D7", "rst": "D5",
        "width": 320, "height": 170, "rotation": 1, "text_size": 2, "status_bar": True,
    }
    cap = {"type": "capture_display", "camera_url": args.camera_url,
           "exposure_us": args.exposure_us, "gain": args.gain,
           "roi": roi, "out": "/tmp/hil-display-capture.jpg"}
    inject = {"type": "inject_protobuf", "kind": "display_add_i8080", "settle_s": 5,
              "params": display_params}
    write = {"type": "inject_protobuf", "settle_s": 5,
             "payload_hex": display_write_signal("tft-19-i8080", args.message).hex()}
    if args.uid:
        inject["uid"] = args.uid
        write["uid"] = args.uid
    job = {
        "target": {"device": {"id": args.device}, "pool": "public"},
        "script": "firmware-bench",
        "params": {
            "firmware": {"path": fw_path, "offset": "0x0"},
            "window_minutes": args.window_minutes,
            "collect_artifacts": ["/tmp/hil-display-capture.jpg"],
            "stages": [
                {"type": "enter_bootloader"},
                {"type": "erase", "before": "no_reset", "after": "no_reset"},
                {"type": "flash", "offset": "0x0", "before": "no_reset", "after": "no_reset"},
                {"type": "power_cycle"},
                {"type": "write_secrets_msc"},
                {"type": "power_cycle"},
                {"type": "verify_checkin", "checkin_timeout_s": 150, "proto": "auto", "soft": True},
                inject,
                write,
                cap,
            ],
        },
        "secrets": secrets,
    }
    st, body = _req("POST", f"{base}/v1/jobs", token, json.dumps(job).encode())
    job_id = json.loads(body)["id"]
    print(f"submitted job {job_id}")

    # 3) poll verdicts to terminal
    verdicts: dict[str, str] = {}
    since, deadline = 0, time.time() + args.deadline_s
    state = "pending"
    while time.time() < deadline:
        st, body = _req("GET", f"{base}/v1/jobs/{job_id}/wait?since={since}&timeout=20", token, timeout=40)
        d = json.loads(body)
        since = d.get("next_since", since)
        for e in d.get("events", []):
            p = e.get("payload", {})
            if p.get("stream") == "serial":
                continue
            m = str(p.get("msg", "") or "")
            for key in ("CHECKIN_VERDICT", "INJECT_VERDICT", "DISPLAY_CAPTURE_VERDICT"):
                if key in m:
                    verdicts[key] = m.strip()
                    print(f"  {m.strip()[:160]}")
        state = d.get("state", state)
        if state in ("finished", "failed", "cancelled", "error", "timeout"):
            break
    print(f"job state={state}")

    # 4) download proof assets (camera image + logs)
    st, body = _req("GET", f"{base}/v1/jobs/{job_id}/assets", token)
    for a in json.loads(body).get("assets", []):
        name = a.get("name") or a.get("filename") or a.get("id")
        aid = a.get("id") or a.get("asset_id")
        if not aid or (a.get("kind") == "firmware"):
            continue
        try:
            _, content = _req("GET", f"{base}/v1/jobs/{job_id}/assets/{aid}", token, timeout=120)
            dest = os.path.join(args.out_dir, name)
            open(dest, "wb").write(content)
            print(f"  pulled asset {name} ({len(content)} B)")
        except Exception as exc:  # noqa: BLE001
            print(f"  asset {name}: {exc}")

    # 5) verdict
    checkin_ok = "ok=true" in verdicts.get("CHECKIN_VERDICT", "")
    captured = "DISPLAY_CAPTURE_VERDICT" in verdicts
    print(f"RESULT checkin_ok={checkin_ok} display_captured={captured}")
    return 0 if (checkin_ok and captured) else 1


if __name__ == "__main__":
    sys.exit(main())
