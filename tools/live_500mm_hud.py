#!/usr/bin/env python3
import os
import re
import subprocess
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

RUNNER_DEFAULT = "/tmp/live_mono_imu_500mm"
PARAMS_DEFAULT = "params/JTZeroMono"
WINDOW = "JT-ZERO 500mm alignment HUD"

vio_re = re.compile(
    r"\[VIO\]\s+kf=(\d+)\s+P=\[([^,]+),([^,]+),([^\]]+)\]"
    r"\s+V=\[([^,]+),([^,]+),([^\]]+)\]"
    r"\s+RPYdeg=\[([^,]+),([^,]+),([^\]]+)\]"
)

lock = threading.Lock()
latest = None
history = deque(maxlen=100)
phase = "WAIT"
runner_done = False
runner_rc = None


def wrap_deg(v):
    while v > 180.0:
        v -= 360.0
    while v < -180.0:
        v += 360.0
    return v


def reader(proc):
    global latest, phase, runner_done, runner_rc
    for raw in proc.stdout:
        line = raw.rstrip("\n")
        print(line, flush=True)
        if ">>>>>>>> MOVE NOW" in line:
            with lock:
                phase = "MOVE"
        elif ">>>>>>>> STOP." in line:
            with lock:
                phase = "END"
        elif line.startswith("[START]"):
            with lock:
                phase = "START"
        m = vio_re.search(line)
        if m:
            s = {
                "kf": int(m.group(1)),
                "p": np.array([float(m.group(2)), float(m.group(3)), float(m.group(4))]),
                "v": np.array([float(m.group(5)), float(m.group(6)), float(m.group(7))]),
                "rpy": np.array([float(m.group(8)), float(m.group(9)), float(m.group(10))]),
            }
            with lock:
                latest = s
                history.append(s)
                if phase == "WAIT":
                    phase = "START"
    runner_rc = proc.wait()
    runner_done = True


def put(img, text, x, y, scale=0.65, thick=1):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (235, 235, 235), thick, cv2.LINE_AA)


def gauge(img, label, value, limit, y, unit):
    x0, x1 = 245, 735
    center = (x0 + x1) // 2
    cv2.line(img, (x0, y), (x1, y), (130, 130, 130), 2)
    cv2.line(img, (center, y - 12), (center, y + 12), (230, 230, 230), 2)
    q = max(-1.0, min(1.0, value / limit))
    xpos = int(center + q * (x1 - x0) * 0.5)
    absq = abs(q)
    color = (80, 210, 80) if absq < 0.35 else ((0, 210, 255) if absq < 0.7 else (60, 60, 240))
    cv2.circle(img, (xpos, y), 8, color, -1, cv2.LINE_AA)
    put(img, f"{label:<7} {value:+7.2f} {unit}", 20, y + 7, 0.62, 1)


def snapshot():
    with lock:
        return latest, list(history), phase


def main():
    runner = sys.argv[1] if len(sys.argv) > 1 else RUNNER_DEFAULT
    params = sys.argv[2] if len(sys.argv) > 2 else PARAMS_DEFAULT
    if not os.path.exists(runner):
        print(f"[HUD] runner not found: {runner}", file=sys.stderr)
        return 2

    env = os.environ.copy()
    old = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = "/home/vio/Kimera-VIO/build:/usr/local/lib" + ((":" + old) if old else "")
    cmd = ["stdbuf", "-oL", "-eL", runner, params]
    print("[HUD] launching:", " ".join(cmd), flush=True)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, env=env)
    threading.Thread(target=reader, args=(proc,), daemon=True).start()

    baseline = None
    direction = None
    jump_until = 0.0
    prev = None
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 800, 620)

    while True:
        s, hist, ph = snapshot()
        img = np.zeros((620, 800, 3), dtype=np.uint8)
        put(img, "JT-ZERO 500 mm ALIGNMENT", 20, 35, 0.85, 2)
        put(img, f"PHASE: {ph}", 610, 35, 0.62, 2)

        if s is None:
            put(img, "Waiting for first Kimera backend state...", 20, 90, 0.7, 1)
        else:
            if baseline is None:
                baseline = {"p": s["p"].copy(), "rpy": s["rpy"].copy()}
            dp = s["p"] - baseline["p"]
            dr = np.array([wrap_deg(s["rpy"][i] - baseline["rpy"][i]) for i in range(3)])

            horizontal = dp[:2]
            hnorm = float(np.linalg.norm(horizontal))
            if direction is None and ph == "MOVE" and hnorm >= 0.050:
                direction = horizontal / hnorm
            travel = 0.0
            cross = 0.0
            if direction is not None:
                travel = float(np.dot(horizontal, direction))
                cross = float(direction[0] * horizontal[1] - direction[1] * horizontal[0])

            if prev is not None:
                step_mm = float(np.linalg.norm(s["p"] - prev["p"])) * 1000.0
                dkf = max(1, s["kf"] - prev["kf"])
                if step_mm > 80.0 and dkf <= 20:
                    jump_until = time.time() + 2.5
                    print(f"[VIO-JUMP] kf={s['kf']} dP={step_mm:.1f} mm dkf={dkf}", flush=True)
            prev = s

            gauge(img, "ROLL", dr[0], 5.0, 100, "deg")
            gauge(img, "PITCH", dr[1], 5.0, 155, "deg")
            gauge(img, "YAW", dr[2], 8.0, 210, "deg")
            gauge(img, "Z", dp[2] * 1000.0, 50.0, 285, "mm")
            if direction is not None:
                gauge(img, "CROSS", cross * 1000.0, 50.0, 340, "mm")
            else:
                put(img, "CROSS: waiting for >=50 mm movement to learn direction", 20, 347, 0.55, 1)

            put(img, f"TRAVEL: {travel*1000.0:7.1f} / 500.0 mm", 20, 415, 0.78, 2)
            put(img, f"dP XYZ: [{dp[0]*1000:+.1f}, {dp[1]*1000:+.1f}, {dp[2]*1000:+.1f}] mm", 20, 455, 0.62, 1)
            put(img, f"|V|: {np.linalg.norm(s['v'])*1000.0:.1f} mm/s    KF: {s['kf']}", 20, 490, 0.62, 1)
            put(img, "Keep ROLL/PITCH/YAW, Z and CROSS near zero.", 20, 535, 0.62, 1)
            if time.time() < jump_until:
                cv2.rectangle(img, (8, 555), (792, 610), (50, 50, 230), -1)
                put(img, "VIO JUMP DETECTED - measurement may be invalid", 25, 590, 0.68, 2)
            else:
                put(img, "ESC/Q: abort test", 20, 590, 0.55, 1)

        cv2.imshow(WINDOW, img)
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
            break
        if runner_done:
            time.sleep(1.0)
            break

    cv2.destroyAllWindows()
    return runner_rc if runner_rc is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
