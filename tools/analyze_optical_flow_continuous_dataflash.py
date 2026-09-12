#!/usr/bin/env python3
# JT-Zero — forensic непрерывной OpticalFlow серии по CSV + SESSION JSON + DataFlash BIN.
# Не требует pymavlink: разбирает только необходимые DataFlash FMT/XKF/OF/RFND сообщения.

from __future__ import annotations
import argparse, csv, json, math, statistics, struct
from collections import defaultdict
from pathlib import Path

FMT_SPEC = {
    "b": ("b", 1, 1.0), "B": ("B", 1, 1.0),
    "h": ("h", 2, 1.0), "H": ("H", 2, 1.0),
    "i": ("i", 4, 1.0), "I": ("I", 4, 1.0),
    "f": ("f", 4, 1.0), "d": ("d", 8, 1.0),
    "q": ("q", 8, 1.0), "Q": ("Q", 8, 1.0),
    "c": ("h", 2, 0.01), "C": ("H", 2, 0.01),
    "e": ("i", 4, 0.01), "E": ("I", 4, 0.01),
    "L": ("i", 4, 1e-7), "M": ("B", 1, 1.0),
    "n": ("4s", 4, None), "N": ("16s", 16, None), "Z": ("64s", 64, None),
}

def decode(payload: bytes, fmt: str):
    vals, off = [], 0
    for c in fmt:
        spec, size, scale = FMT_SPEC[c]
        raw = struct.unpack_from("<" + spec, payload, off)[0]
        off += size
        if scale is None:
            raw = raw.split(b"\0", 1)[0].decode("ascii", "replace")
        elif scale != 1.0:
            raw *= scale
        vals.append(raw)
    return vals

def parse_dataflash(path: Path):
    data = path.read_bytes()
    fmts, out = {}, defaultdict(list)
    i, n = 0, len(data)
    while i + 3 <= n:
        if data[i:i+2] != b"\xA3\x95":
            i += 1
            continue
        typ = data[i+2]
        if typ == 0x80:  # FMT
            if i + 89 > n:
                break
            r = data[i+3:i+89]
            msg_type, msg_len = r[0], r[1]
            name = r[2:6].split(b"\0", 1)[0].decode("ascii", "replace")
            fmt = r[6:22].split(b"\0", 1)[0].decode("ascii", "replace")
            cols_raw = r[22:86].split(b"\0", 1)[0]
            cols = cols_raw.decode("ascii", "replace").split(",") if cols_raw else []
            fmts[msg_type] = (msg_len, name, fmt, cols)
            i += 89
            continue
        info = fmts.get(typ)
        if not info:
            i += 1
            continue
        msg_len, name, fmt, cols = info
        if msg_len < 3 or i + msg_len > n:
            i += 1
            continue
        try:
            vals = decode(data[i+3:i+msg_len], fmt)
            if len(vals) == len(cols):
                out[name].append(dict(zip(cols, vals)))
        except Exception:
            pass
        i += msg_len
    return out

def f(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default

def median(values):
    return statistics.median(values) if values else float("nan")

def mean(values):
    return statistics.mean(values) if values else float("nan")

def maxabs(values):
    return max((abs(v) for v in values), default=float("nan"))

def fit_clock(csv_rows, of_rows):
    by_flow = defaultdict(list)
    for r in of_rows:
        by_flow[(round(float(r["flowX"]), 7), round(float(r["flowY"]), 7))].append(float(r["TimeUS"]))

    pairs = []
    for r in csv_rows:
        if int(f(r, "flow_sent")) != 1:
            continue
        key = (round(f(r, "flow_send_x"), 7), round(f(r, "flow_send_y"), 7))
        if len(by_flow[key]) == 1:
            pairs.append((f(r, "mono_ns") / 1000.0, by_flow[key][0]))

    if len(pairs) < 20:
        raise RuntimeError(f"недостаточно OF совпадений для синхронизации часов: {len(pairs)}")

    xs = [p[0] for p in pairs]
    ys = [p[1] for p in pairs]
    x0, y0 = mean(xs), mean(ys)
    mask = [True] * len(pairs)

    for _ in range(5):
        num = sum((x-x0)*(y-y0) for (x,y), keep in zip(pairs, mask) if keep)
        den = sum((x-x0)*(x-x0) for (x,y), keep in zip(pairs, mask) if keep)
        a = num / den
        b = y0 - a*x0
        residuals = [y - (a*x+b) for x,y in pairs]
        kept = [r for r,k in zip(residuals, mask) if k]
        med = median(kept)
        mad = median([abs(r-med) for r in kept]) or 1.0
        lim = 5.0 * 1.4826 * mad
        mask = [abs(r-med) < lim for r in residuals]

    residuals = [y - (a*x+b) for x,y in pairs]
    kept_abs = sorted(abs(r) for r,k in zip(residuals, mask) if k)
    p95 = kept_abs[min(len(kept_abs)-1, int(0.95*(len(kept_abs)-1)))]
    return a, b, sum(mask), p95

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("session_json", type=Path)
    ap.add_argument("dataflash_bin", type=Path)
    args = ap.parse_args()

    with args.csv.open(newline="") as fh:
        csv_rows = list(csv.DictReader(fh))
    session = json.loads(args.session_json.read_text(encoding="utf-8"))
    log = parse_dataflash(args.dataflash_bin)

    a, b, matches, p95_us = fit_clock(csv_rows, log["OF"])
    print("===== CLOCK ALIGNMENT =====")
    print(f"OF unique matches : {matches}")
    print(f"FC_us = {a:.9f} * RPi_us + {b:.3f}")
    print(f"alignment residual p95 = {p95_us/1000.0:.3f} ms")
    print()

    print("===== LEG FORENSIC =====")
    hdr = (
        "LEG DIR CORE PHYS   RAW/P  EKF/P EKF/RAW  DURs "
        "HAGL50 HAGLmean  RFND50   NImean NImax  |FIX|max |FIY|max"
    )
    print(hdr)

    for rec in session:
        leg = int(rec["leg"])
        leg_rows = [r for r in csv_rows if int(f(r, "guide_leg")) == leg]
        i0, i1 = int(rec["movement_row_start"]), int(rec["movement_row_end"])
        win = leg_rows[i0:i1+1]
        if not win:
            continue

        rpi0 = f(win[0], "mono_ns") / 1000.0
        rpi1 = f(win[-1], "mono_ns") / 1000.0
        fc0, fc1 = a*rpi0+b, a*rpi1+b
        # Select the EKF3 core whose XKF1 horizontal displacement best matches
        # the LOCAL_POSITION_NED displacement stored by the bench for this leg.
        # This avoids mixing simultaneous EKF3 cores in XKF1/XKF5.
        x1_all = [r for r in log["XKF1"] if fc0 <= float(r["TimeUS"]) <= fc1]
        cores = sorted({int(round(float(r.get("C", 0)))) for r in x1_all})
        target_mm = float(rec["ekf_mm"])
        best_core, best_err = None, float("inf")
        for core in cores:
            rr = [r for r in x1_all if int(round(float(r.get("C", 0)))) == core]
            if len(rr) < 2:
                continue
            a0, a1 = rr[0], rr[-1]
            move_mm = 1000.0 * math.hypot(float(a1["PN"])-float(a0["PN"]),
                                          float(a1["PE"])-float(a0["PE"]))
            err = abs(move_mm-target_mm)
            if err < best_err:
                best_err, best_core = err, core
        if best_core is None:
            best_core = 0

        x5 = [r for r in log["XKF5"]
              if fc0 <= float(r["TimeUS"]) <= fc1
              and int(round(float(r.get("C", 0)))) == best_core]
        rf = [r for r in log["RFND"] if fc0 <= float(r["TimeUS"]) <= fc1]

        h = [float(r["HAGL"]) for r in x5]
        ni = [float(r["NI"]) for r in x5]
        fix = [float(r["FIX"]) for r in x5]
        fiy = [float(r["FIY"]) for r in x5]
        rfd = [float(r["Dist"]) for r in rf]

        phys = float(rec["physical_measured_mm"])
        raw_ratio = float(rec["raw_ratio"])
        ekf_ratio = float(rec["ekf_ratio"])
        ekf_raw = ekf_ratio / raw_ratio
        dur = (f(win[-1], "mono_ns") - f(win[0], "mono_ns")) / 1e9

        print(
            f"{leg:>3} {rec['direction']:<4} {best_core:>4} {phys:>5.0f} "
            f"{raw_ratio:>7.4f} {ekf_ratio:>6.4f} {ekf_raw:>7.4f} "
            f"{dur:>5.2f} {median(h):>6.3f} {mean(h):>8.3f} "
            f"{median(rfd):>7.3f} {mean(ni):>8.3f} {max(ni, default=float('nan')):>5.1f} "
            f"{maxabs(fix):>9.1f} {maxabs(fiy):>9.1f}"
        )

    print()
    print("Примечание: XKF5.HAGL — настоящий EKF3 HAGL из DataFlash.")
    print("CORE выбирается по совпадению XKF1 displacement с LOCAL_POSITION_NED bench-результатом.")
    print("RFND50 — DataFlash RFND.Dist. NI/FIX/FIY взяты только из выбранного XKF5 core.")

if __name__ == "__main__":
    main()
