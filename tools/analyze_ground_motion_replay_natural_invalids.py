#!/usr/bin/env python3
"""Разбор естественных invalid-шагов BASELINE_CURRENT vs BASELINE_ANCHOR.

Использует уже готовый deterministic replay dataset. Ничего не пересчитывает через OpenCV.
Показывает, где именно baseline anchor разошёлся с current и связано ли это
с естественными visual-invalid шагами.
"""

from __future__ import annotations

import csv
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

VISUAL_INVALID = {
    "DT", "FEATURES", "LK", "HOMOGRAPHY", "HOM_INLIERS", "V3",
    "STEP_CAP", "INLIERS", "SCATTER", "QUALITY",
}


def read_events(path: Path):
    out = {}
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            out[row["event"]] = int(row["mono_ns"])
    return out


def read_frame_times(path: Path):
    out = {}
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            out[int(row["frame"])] = int(row["mono_ns"])
    return out


def _normalise_detail_row(row):
    """Нормализует replay_detail.csv старого диагностического формата.

    В раннем deterministic replay строки DECODE/SENSORS/NO_PREV содержали на одно
    числовое поле меньше заголовка: отсутствовал нулевой dy_m перед x_m/y_m.
    csv.DictReader поэтому сдвигал x/y влево и оставлял y_m=None.
    Это дефект только диагностического CSV; сам replay result от него не менялся.
    """
    if row.get("y_m") is None:
        # Старый короткий формат:
        #   ... dx_m, <x фактически в dy_m>, <y фактически в x_m>, y_m=None
        row["y_m"] = row.get("x_m")
        row["x_m"] = row.get("dy_m")
        row["dy_m"] = "0"
    return row


def read_detail(path: Path):
    by_scenario = {"BASELINE_CURRENT": {}, "BASELINE_ANCHOR": {}}
    short_rows = 0
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        for raw in r:
            if raw.get("y_m") is None:
                short_rows += 1
            row = _normalise_detail_row(raw)
            sc = row["scenario"]
            if sc not in by_scenario:
                continue
            fr = int(row["frame"])
            by_scenario[sc][fr] = {
                "reason": row["reason"],
                "accepted": int(row["accepted"]) != 0,
                "dt_s": float(row["dt_s"]),
                "dx": float(row["dx_m"]),
                "dy": float(row["dy_m"]),
                "x": float(row["x_m"]),
                "y": float(row["y_m"]),
            }
    return by_scenario, short_rows


def median_state(rows, frame_times, t0, t1):
    xs, ys = [], []
    for fr, row in rows.items():
        t = frame_times.get(fr)
        if t is not None and t0 <= t <= t1:
            xs.append(row["x"])
            ys.append(row["y"])
    if not xs:
        return (math.nan, math.nan)
    return (statistics.median(xs), statistics.median(ys))


def vec(a, b):
    return (a[0] - b[0], a[1] - b[1])


def norm(v):
    return math.hypot(v[0], v[1])


def main():
    if len(sys.argv) != 2:
        print(f"Использование: {sys.argv[0]} <dataset_dir>", file=sys.stderr)
        return 2

    d = Path(sys.argv[1])
    events = read_events(d / "events.csv")
    frame_times = read_frame_times(d / "frames.csv")
    detail, short_rows = read_detail(d / "replay_detail.csv")
    cur = detail["BASELINE_CURRENT"]
    anc = detail["BASELINE_ANCHOR"]

    move_a = events["MOVE_START"]
    move_b = events["MOVE_END"]
    move_frames = [fr for fr in sorted(cur) if move_a <= frame_times.get(fr, -1) <= move_b and fr in anc]
    if not move_frames:
        raise RuntimeError("в MOVE не найдены общие baseline кадры")

    reasons = Counter(cur[fr]["reason"] for fr in move_frames)
    visual_bad = [fr for fr in move_frames if cur[fr]["reason"] in VISUAL_INVALID]

    W = 500_000_000
    cur_pre = median_state(cur, frame_times, move_a - W, move_a)
    cur_post = median_state(cur, frame_times, move_b, move_b + W)
    anc_pre = median_state(anc, frame_times, move_a - W, move_a)
    anc_post = median_state(anc, frame_times, move_b, move_b + W)
    cur_move = vec(cur_post, cur_pre)
    anc_move = vec(anc_post, anc_pre)
    endpoint_delta = vec(anc_move, cur_move)

    print("===== NATURAL INVALIDS: BASELINE CURRENT vs ANCHOR =====")
    if short_rows:
        print(f"Примечание: нормализовано коротких legacy detail-строк: {short_rows}")
    print(f"MOVE frames={len(move_frames)}")
    print(f"CURRENT move=({cur_move[0]*1000:+.2f},{cur_move[1]*1000:+.2f}) mm |d|={norm(cur_move)*1000:.2f} mm")
    print(f"ANCHOR  move=({anc_move[0]*1000:+.2f},{anc_move[1]*1000:+.2f}) mm |d|={norm(anc_move)*1000:.2f} mm")
    print(f"ANCHOR-CURRENT vector=({endpoint_delta[0]*1000:+.2f},{endpoint_delta[1]*1000:+.2f}) mm |D|={norm(endpoint_delta)*1000:.2f} mm")
    print()

    print("CURRENT reasons внутри MOVE:")
    for reason, n in reasons.most_common():
        print(f"  {reason:16s} {n:5d}")
    print(f"visual-invalid frames={len(visual_bad)}")

    # Серии естественных visual-invalid именно текущей production-политики.
    runs = []
    i = 0
    while i < len(move_frames):
        fr = move_frames[i]
        if cur[fr]["reason"] not in VISUAL_INVALID:
            i += 1
            continue
        j = i
        while j + 1 < len(move_frames):
            fr2 = move_frames[j + 1]
            if fr2 != move_frames[j] + 1 or cur[fr2]["reason"] not in VISUAL_INVALID:
                break
            j += 1
        runs.append((i, j))
        i = j + 1

    episodes = []
    for i0, i1 in runs:
        first = move_frames[i0]
        last = move_frames[i1]
        before_idx = max(0, i0 - 1)
        before_fr = move_frames[before_idx]

        # Ищем первый следующий кадр, который anchor реально принял после серии.
        after_idx = i1 + 1
        while after_idx < len(move_frames) and not anc[move_frames[after_idx]]["accepted"]:
            after_idx += 1
        if after_idx >= len(move_frames):
            continue
        after_fr = move_frames[after_idx]

        d_before = (anc[before_fr]["x"] - cur[before_fr]["x"], anc[before_fr]["y"] - cur[before_fr]["y"])
        d_after = (anc[after_fr]["x"] - cur[after_fr]["x"], anc[after_fr]["y"] - cur[after_fr]["y"])
        change = (d_after[0] - d_before[0], d_after[1] - d_before[1])
        reasons_run = "+".join(cur[move_frames[k]]["reason"] for k in range(i0, i1 + 1))
        episodes.append({
            "first": first,
            "last": last,
            "n": i1 - i0 + 1,
            "after": after_fr,
            "reasons": reasons_run,
            "anchor_after_reason": anc[after_fr]["reason"],
            "change": change,
            "mag": norm(change),
            "dt_ms": (frame_times[after_fr] - frame_times[before_fr]) / 1e6,
        })

    print()
    print(f"Естественных visual-invalid серий внутри MOVE: {len(episodes)}")
    if episodes:
        total_mag = sum(e["mag"] for e in episodes)
        print(f"Сумма модулей изменений ANCHOR-CURRENT после этих серий: {total_mag*1000:.2f} mm")
        print("Крупнейшие серии по изменению ANCHOR-CURRENT:")
        for e in sorted(episodes, key=lambda x: x["mag"], reverse=True)[:20]:
            ch = e["change"]
            print(
                f"  frames {e['first']}..{e['last']} n={e['n']} reasons={e['reasons']} "
                f"bridge_to={e['after']} ({e['anchor_after_reason']}) span={e['dt_ms']:.1f}ms "
                f"dDiff=({ch[0]*1000:+.2f},{ch[1]*1000:+.2f}) |d|={e['mag']*1000:.2f}mm"
            )

    print()
    if not episodes:
        print("VERDICT: внутри MOVE не найдено естественных visual-invalid серий; 3.2 мм надо искать вне этого механизма.")
    elif norm(endpoint_delta) < 0.0005:
        print("VERDICT: baseline CURRENT и ANCHOR практически совпадают; естественные invalid не дали заметного endpoint эффекта.")
    else:
        print("VERDICT: baseline ANCHOR и CURRENT расходятся, и теперь видно, на каких естественных invalid-сериях возникает разница.")
        print("Это ещё не доказывает, что ANCHOR физически точнее: для этого нужно сопоставить знак восстановленного вектора с направлением реального движения.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
