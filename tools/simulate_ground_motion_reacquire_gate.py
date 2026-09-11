#!/usr/bin/env python3
"""Симуляция безопасного gate повторного захвата ExternalNav по существующему CSV.

Ничего не меняет в FC или production estimator. Использует raw valid из
Ground Motion и проверяет, как повёл бы себя publisher с правилами:
  - обычные короткие invalid-пропуски не меняют состояние;
  - после LOSS_TIMEOUT без valid ExternalNav считается LOST;
  - после LOST публикация не возобновляется по одному valid кадру;
  - для REACQUIRE нужны N последовательных valid и минимальная длительность серии.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

NS = 1_000_000_000


def read_events(path: Path):
    out = {}
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            out[r["event"]] = int(r["mono_ns"])
    return out


def read_rows(path: Path):
    out = []
    with path.open(newline="") as f:
        rd = csv.DictReader(f)
        need = {"mono_ns", "valid", "mav_sent", "quality", "inliers", "scatter_m"}
        miss = need.difference(rd.fieldnames or [])
        if miss:
            raise RuntimeError(f"{path}: нет колонок: {', '.join(sorted(miss))}")
        for r in rd:
            out.append({
                "t": int(r["mono_ns"]),
                "valid": int(r["valid"]) != 0,
                "sent": int(r["mav_sent"]) != 0,
                "quality": float(r["quality"]),
                "inliers": int(r["inliers"]),
                "scatter": float(r["scatter_m"]),
            })
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Симуляция safe reacquire gate Ground Motion")
    ap.add_argument("csv", type=Path)
    ap.add_argument("events", type=Path)
    ap.add_argument("--loss-ms", type=float, default=500.0)
    ap.add_argument("--reacquire-ms", type=float, default=200.0)
    ap.add_argument("--reacquire-frames", type=int, default=8)
    args = ap.parse_args()

    rows = read_rows(args.csv)
    ev = read_events(args.events)
    for e in ("STATIC_AFTER_MOVE_END", "BLOCK_START", "BLOCK_10S_REACHED", "BLOCK_END", "RECOVERY_END"):
        if e not in ev:
            raise RuntimeError(f"events.csv: нет {e}")

    loss_ns = int(args.loss_ms * 1e6)
    reacq_ns = int(args.reacquire_ms * 1e6)

    active = True
    last_valid_t = None
    streak_start = None
    streak_count = 0
    loss_events = []
    reacq_events = []
    gated = []

    for r in rows:
        t = r["t"]
        gsend = False

        if active:
            if r["valid"]:
                last_valid_t = t
                gsend = True
            else:
                if last_valid_t is not None and t - last_valid_t > loss_ns:
                    active = False
                    streak_start = None
                    streak_count = 0
                    loss_events.append(t)
        else:
            if r["valid"]:
                if streak_count == 0:
                    streak_start = t
                streak_count += 1
                span = t - streak_start if streak_start is not None else 0
                if streak_count >= args.reacquire_frames and span >= reacq_ns:
                    active = True
                    last_valid_t = t
                    reacq_events.append(t)
                    gsend = True
            else:
                streak_start = None
                streak_count = 0

        gated.append((t, gsend))

    strict_a = ev["BLOCK_START"]
    strict_b = ev["BLOCK_10S_REACHED"]
    strict_core_a = strict_a + NS
    strict_rows = [r for r in rows if strict_core_a <= r["t"] <= strict_b]
    strict_raw = [r for r in strict_rows if r["valid"]]
    strict_gate = [t for t, s in gated if strict_core_a <= t <= strict_b and s]

    print("===== SAFE REACQUIRE GATE SIM =====")
    print(f"loss_timeout={args.loss_ms:.0f} ms; reacquire={args.reacquire_ms:.0f} ms + {args.reacquire_frames} consecutive valid frames")
    print(f"strict blocked core: raw valid={len(strict_raw)}, gated sends={len(strict_gate)}")
    if strict_raw:
        for n, r in enumerate(strict_raw, 1):
            print(
                f"  raw false-valid #{n}: t={(r['t']-strict_a)/1e9:.3f}s "
                f"q={r['quality']:.3f} inliers={r['inliers']} scatter={r['scatter']*1000:.3f}mm"
            )

    print("\nLOSS events:")
    for n, t in enumerate(loss_events, 1):
        print(f"  #{n}: t={(t-ev['STATIC_AFTER_MOVE_END'])/1e9:.3f}s from STATIC_AFTER_MOVE_END")

    print("\nREACQUIRE events:")
    for n, t in enumerate(reacq_events, 1):
        rel_block_end = (t - ev["BLOCK_END"]) / 1e6
        print(
            f"  #{n}: t={(t-ev['STATIC_AFTER_MOVE_END'])/1e9:.3f}s; "
            f"relative BLOCK_END={rel_block_end:+.1f} ms"
        )

    after_open = [(t, s) for t, s in gated if t >= ev["BLOCK_END"] and s]
    if after_open:
        print(f"\nfirst gated send after BLOCK_END: {(after_open[0][0]-ev['BLOCK_END'])/1e6:.1f} ms")
    else:
        print("\nfirst gated send after BLOCK_END: НЕТ")

    if len(strict_gate) == 0:
        print("PASS: одиночные false-valid в закрытом интервале не дошли бы до ExternalNav.")
    else:
        print("FAIL: выбранный gate всё ещё пропускает данные при закрытой камере.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
