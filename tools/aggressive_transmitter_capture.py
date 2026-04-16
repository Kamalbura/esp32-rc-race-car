#!/usr/bin/env python3
import argparse
import csv
import json
import os
import statistics
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import serial


ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / "tests" / "transmitter_aggressive_logger_test"
LOG_DIR = ROOT / "calibration" / "logs"


STEPS = [
    ("center_rest", "Release both joysticks. Do not touch encoder."),
    ("both_slow_up_center", "Move BOTH joysticks slowly full UP, then return center inside the capture window."),
    ("both_normal_up_center", "Move BOTH joysticks normally full UP, then return center."),
    ("both_fast_up_center", "Flick BOTH joysticks fast full UP, then return center."),
    ("both_slow_down_center", "Move BOTH joysticks slowly full DOWN, then return center."),
    ("both_normal_down_center", "Move BOTH joysticks normally full DOWN, then return center."),
    ("both_fast_down_center", "Flick BOTH joysticks fast full DOWN, then return center."),
    ("both_slow_left_center", "Move BOTH joysticks slowly full LEFT, then return center."),
    ("both_normal_left_center", "Move BOTH joysticks normally full LEFT, then return center."),
    ("both_fast_left_center", "Flick BOTH joysticks fast full LEFT, then return center."),
    ("both_slow_right_center", "Move BOTH joysticks slowly full RIGHT, then return center."),
    ("both_normal_right_center", "Move BOTH joysticks normally full RIGHT, then return center."),
    ("both_fast_right_center", "Flick BOTH joysticks fast full RIGHT, then return center."),
    ("right_slow_up_center", "RIGHT joystick slowly full UP, then center. Leave left joystick centered."),
    ("right_normal_up_center", "RIGHT joystick normally full UP, then center."),
    ("right_fast_up_center", "RIGHT joystick fast full UP, then center."),
    ("right_slow_down_center", "RIGHT joystick slowly full DOWN, then center."),
    ("right_normal_down_center", "RIGHT joystick normally full DOWN, then center."),
    ("right_fast_down_center", "RIGHT joystick fast full DOWN, then center."),
    ("right_slow_left_center", "RIGHT joystick slowly full LEFT, then center."),
    ("right_normal_left_center", "RIGHT joystick normally full LEFT, then center."),
    ("right_fast_left_center", "RIGHT joystick fast full LEFT, then center."),
    ("right_slow_right_center", "RIGHT joystick slowly full RIGHT, then center."),
    ("right_normal_right_center", "RIGHT joystick normally full RIGHT, then center."),
    ("right_fast_right_center", "RIGHT joystick fast full RIGHT, then center."),
    ("right_45_up_left_center", "RIGHT joystick diagonal 45 degrees UP-LEFT, then center."),
    ("right_45_up_right_center", "RIGHT joystick diagonal 45 degrees UP-RIGHT, then center."),
    ("right_45_down_left_center", "RIGHT joystick diagonal 45 degrees DOWN-LEFT, then center."),
    ("right_45_down_right_center", "RIGHT joystick diagonal 45 degrees DOWN-RIGHT, then center."),
    ("right_circle_clockwise", "RIGHT joystick one smooth clockwise circle, then center."),
    ("right_circle_counterclockwise", "RIGHT joystick one smooth counterclockwise circle, then center."),
    ("left_slow_up_center", "LEFT joystick slowly full UP, then center. Leave right joystick centered."),
    ("left_normal_up_center", "LEFT joystick normally full UP, then center."),
    ("left_fast_up_center", "LEFT joystick fast full UP, then center."),
    ("left_slow_down_center", "LEFT joystick slowly full DOWN, then center."),
    ("left_normal_down_center", "LEFT joystick normally full DOWN, then center."),
    ("left_fast_down_center", "LEFT joystick fast full DOWN, then center."),
    ("left_slow_left_center", "LEFT joystick slowly full LEFT, then center."),
    ("left_normal_left_center", "LEFT joystick normally full LEFT, then center."),
    ("left_fast_left_center", "LEFT joystick fast full LEFT, then center."),
    ("left_slow_right_center", "LEFT joystick slowly full RIGHT, then center."),
    ("left_normal_right_center", "LEFT joystick normally full RIGHT, then center."),
    ("left_fast_right_center", "LEFT joystick fast full RIGHT, then center."),
    ("left_45_up_left_center", "LEFT joystick diagonal 45 degrees UP-LEFT, then center."),
    ("left_45_up_right_center", "LEFT joystick diagonal 45 degrees UP-RIGHT, then center."),
    ("left_45_down_left_center", "LEFT joystick diagonal 45 degrees DOWN-LEFT, then center."),
    ("left_45_down_right_center", "LEFT joystick diagonal 45 degrees DOWN-RIGHT, then center."),
    ("left_circle_clockwise", "LEFT joystick one smooth clockwise circle, then center."),
    ("left_circle_counterclockwise", "LEFT joystick one smooth counterclockwise circle, then center."),
    ("encoder_idle", "Do not touch joysticks or encoder."),
    ("encoder_one_right", "Turn encoder exactly 1 click RIGHT."),
    ("encoder_two_right", "Turn encoder exactly 2 clicks RIGHT."),
    ("encoder_many_right", "Turn encoder MANY clicks RIGHT across the 2 seconds."),
    ("encoder_one_left", "Turn encoder exactly 1 click LEFT."),
    ("encoder_two_left", "Turn encoder exactly 2 clicks LEFT."),
    ("encoder_many_left", "Turn encoder MANY clicks LEFT across the 2 seconds."),
    ("encoder_button_hold_release", "Press and hold encoder button, then release before capture ends."),
    ("aggressive_all_controls", "Aggressive final: mix both sticks, diagonals, encoder turns, then release center."),
]


FIELDNAMES = [
    "step_index",
    "step_name",
    "host_time_ms",
    "device_ms",
    "seq",
    "A0",
    "A1",
    "A2",
    "A3",
    "enc_bits",
    "enc_delta",
    "sent",
    "errors",
    "ack",
]


def run(cmd):
    print("+", " ".join(str(part) for part in cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)


def parse_data(line):
    if not line.startswith("DATA,"):
        return None
    parts = line.split(",")
    if len(parts) != 12:
        return None
    try:
        return {
            "device_ms": int(parts[1]),
            "seq": int(parts[2]),
            "A0": int(parts[3]),
            "A1": int(parts[4]),
            "A2": int(parts[5]),
            "A3": int(parts[6]),
            "enc_bits": int(parts[7]),
            "enc_delta": int(parts[8]),
            "sent": int(parts[9]),
            "errors": int(parts[10]),
            "ack": int(parts[11]),
        }
    except ValueError:
        return None


def drain_until_ready(port, timeout_s):
    deadline = time.time() + timeout_s
    seen = []
    while time.time() < deadline:
        raw = port.readline().decode("utf-8", errors="replace").strip()
        if raw:
            seen.append(raw)
            print(raw)
            if raw == "AGG_TX_READY":
                return True
    print("Did not see AGG_TX_READY. Last serial lines:")
    for line in seen[-10:]:
        print("  " + line)
    return False


def collect_samples(port, duration_s, step_index, step_name, writer, start_monotonic):
    port.reset_input_buffer()
    end_time = time.monotonic() + duration_s
    samples = []
    while time.monotonic() < end_time:
        raw = port.readline().decode("utf-8", errors="replace").strip()
        data = parse_data(raw)
        if data is None:
            continue
        data["step_index"] = step_index
        data["step_name"] = step_name
        data["host_time_ms"] = int((time.monotonic() - start_monotonic) * 1000)
        writer.writerow({name: data[name] for name in FIELDNAMES})
        samples.append(data)
    return samples


def summarize_step(step_index, step_name, instruction, samples):
    summary = {
        "step_index": step_index,
        "step_name": step_name,
        "instruction": instruction,
        "samples": len(samples),
        "channels": {},
        "encoder": {},
        "espnow": {},
    }
    if not samples:
        return summary

    for channel in ["A0", "A1", "A2", "A3"]:
        values = [s[channel] for s in samples]
        rates = []
        for prev, cur in zip(samples, samples[1:]):
            dt_ms = cur["device_ms"] - prev["device_ms"]
            if dt_ms <= 0:
                continue
            rates.append((cur[channel] - prev[channel]) * 1000.0 / dt_ms)
        summary["channels"][channel] = {
            "min": min(values),
            "max": max(values),
            "avg": round(statistics.fmean(values), 2),
            "span": max(values) - min(values),
            "start": values[0],
            "end": values[-1],
            "delta": values[-1] - values[0],
            "max_abs_rate_per_sec": round(max((abs(r) for r in rates), default=0.0), 2),
            "avg_abs_rate_per_sec": round(statistics.fmean(abs(r) for r in rates), 2) if rates else 0.0,
        }

    enc_values = [s["enc_delta"] for s in samples]
    enc_bits_seen = sorted(set(s["enc_bits"] for s in samples))
    summary["encoder"] = {
        "start_delta": enc_values[0],
        "end_delta": enc_values[-1],
        "movement": enc_values[-1] - enc_values[0],
        "bits_seen": enc_bits_seen,
    }
    summary["espnow"] = {
        "sent_start": samples[0]["sent"],
        "sent_end": samples[-1]["sent"],
        "errors_start": samples[0]["errors"],
        "errors_end": samples[-1]["errors"],
        "ack_start": samples[0]["ack"],
        "ack_end": samples[-1]["ack"],
    }
    return summary


def print_quick_summary(summary):
    print(f"Captured {summary['samples']} samples")
    for channel in ["A0", "A1", "A2", "A3"]:
        ch = summary["channels"].get(channel)
        if not ch:
            continue
        print(
            f"  {channel}: min={ch['min']} max={ch['max']} avg={ch['avg']} "
            f"span={ch['span']} maxRate={ch['max_abs_rate_per_sec']}/s"
        )
    enc = summary.get("encoder", {})
    if enc:
        print(f"  encoder movement={enc['movement']} bits={enc['bits_seen']}")


def main():
    parser = argparse.ArgumentParser(description="Aggressive transmitter motion capture for ESP32-S3 RC transmitter.")
    parser.add_argument("--port", default="COM3", help="Transmitter serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=2.0, help="Capture seconds per step")
    parser.add_argument("--prepare", type=float, default=1.5, help="Countdown seconds before each capture")
    parser.add_argument("--upload", action="store_true", help="Compile and upload the aggressive logger sketch first")
    parser.add_argument("--fqbn", default="esp32:esp32:esp32s3")
    args = parser.parse_args()

    if args.upload:
        run(["arduino-cli", "compile", "--fqbn", args.fqbn, str(SKETCH)])
        run(["arduino-cli", "upload", "--fqbn", args.fqbn, "--port", args.port, str(SKETCH)])

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = LOG_DIR / f"transmitter_aggressive_{stamp}.csv"
    json_path = LOG_DIR / f"transmitter_aggressive_{stamp}.json"

    print()
    print("Aggressive transmitter capture")
    print(f"Port: {args.port} @ {args.baud}")
    print(f"CSV:  {csv_path}")
    print(f"JSON: {json_path}")
    print()
    print("Controls while running:")
    print("  Enter = start the shown step")
    print("  s     = skip a step")
    print("  q     = quit and save what we have")
    print()

    all_summaries = []
    start_monotonic = time.monotonic()

    with serial.Serial(args.port, args.baud, timeout=0.25, write_timeout=0.25) as port:
        port.dtr = False
        port.rts = False
        time.sleep(1.0)
        if not drain_until_ready(port, 3.0):
            print("Continuing anyway because DATA lines may already be streaming.")

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
            writer.writeheader()

            for idx, (name, instruction) in enumerate(STEPS, start=1):
                print()
                print("=" * 72)
                print(f"STEP {idx}/{len(STEPS)}: {name}")
                print(instruction)
                print("Get ready, then press Enter. Do the movement only after CAPTURING appears.")
                print("Type s+Enter to skip, q+Enter to quit.")
                answer = input("> ").strip().lower()
                if answer == "q":
                    break
                if answer == "s":
                    continue

                port.reset_input_buffer()
                for remaining in range(int(args.prepare), 0, -1):
                    print(f"Starting in {remaining}...")
                    time.sleep(1)
                fractional = args.prepare - int(args.prepare)
                if fractional > 0:
                    time.sleep(fractional)

                port.reset_input_buffer()
                print(f"CAPTURING NOW for {args.duration:.1f}s: {instruction}")
                samples = collect_samples(port, args.duration, idx, name, writer, start_monotonic)
                csv_file.flush()

                summary = summarize_step(idx, name, instruction, samples)
                all_summaries.append(summary)
                print_quick_summary(summary)

    result = {
        "source": "tools/aggressive_transmitter_capture.py",
        "sketch": "tests/transmitter_aggressive_logger_test",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "port": args.port,
        "baud": args.baud,
        "duration_s": args.duration,
        "steps": all_summaries,
    }
    json_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print()
    print("Saved:")
    print(csv_path)
    print(json_path)
    print("Done. Send the JSON path or say 'done' and Codex can inspect it.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
