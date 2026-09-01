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
CAMERA = "/dev/video0"

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


def put(img, text, x, y, scale=0.55, thick=1, color=(235, 235, 235)):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thick, cv2.LINE_AA)


def gauge(img, label, value, limit, y, unit):
    x0, x1 = 150, 625
    center = (x0 + x1) // 2
    cv2.line(img, (x0, y), (x1, y), (110, 110, 110), 2)
    cv2.line(img, (center, y - 8), (center, y + 8), (235, 235, 235), 2)
    q = max(-1.0, min(1.0, value / limit))
    xpos = int(center + q * (x1 - x0) * 0.5)
    absq = abs(q)
    color = (80, 210, 80) if absq < 0.35 else ((0, 210, 255) if absq < 0.7 else (60, 60, 240))
    cv2.circle(img, (xpos, y), 6, color, -1, cv2.LINE_AA)
    put(img, f"{label:<6} {value:+6.2f} {unit}", 8, y + 5, 0.48, 1)


def snapshot():
    with lock:
        return latest, list(history), phase


def open_preview_camera():
    # The C++ runner already owns the same UVC device. Some V4L2/UVC stacks
    # allow a second read handle, others reject STREAMON with EBUSY. Try it
    # explicitly and report the result; the HUD remains usable if unavailable.
    cap = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    return cap


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

    # Give the runner time to configure/start its camera first, then try a
    # secondary preview handle. This never replaces the runner's camera feed.
    time.sleep(1.0)
    preview = open_preview_camera()
    if preview is None:
        print("[HUD] WARNING: live camera preview could not open /dev/video0 while runner owns it.", flush=True)
        print("[HUD] HUD will continue, but camera image is unavailable in this build.", flush=True)
    else:
        print("[HUD] live camera preview: OK", flush=True)

    baseline = None
    direction = None
    jump_until = 0.0
    prev = None
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 800, 760)

    while True:
        s, hist, ph = snapshot()

        camera_ok = False
        camera_frame = None
        if preview is not None:
            ok, frame = preview.read()
            if ok and frame is not None:
                camera_ok = True
                camera_frame = cv2.resize(frame, (640, 480))

        img = np.zeros((760, 800, 3), dtype=np.uint8)
        if camera_ok:
            img[0:480, 80:720] = camera_frame
            cv2.line(img, (400, 0), (400, 480), (0, 255, 255), 1)
            cv2.line(img, (80, 240), (720, 240), (0, 255, 255), 1)
            put(img, f"PHASE: {ph}", 90, 28, 0.62, 2, (0, 255, 255))
        else:
            put(img, "NO CAMERA PREVIEW", 255, 210, 0.9, 2, (60, 60, 240))
            put(img, "V4L2 device is busy or preview read failed", 205, 250, 0.55, 1)
            put(img, f"PHASE: {ph}", 330, 290, 0.62, 2)

        if s is None:
            put(img, "Waiting for first Kimera backend state...", 210, 440, 0.6, 1)
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

            if prev is not None and s["kf"] != prev["kf"]:
                step_mm = float(np.linalg.norm(s["p"] - prev["p"])) * 1000.0
                dkf = max(1, s["kf"] - prev["kf"])
                if step_mm > 80.0 and dkf <= 20:
                    jump_until = time.time() + 2.5
                    print(f"[VIO-JUMP] kf={s['kf']} dP={step_mm:.1f} mm dkf={dkf}", flush=True)
            prev = s

            gauge(img, "ROLL", dr[0], 5.0, 505, "deg")
            gauge(img, "PITCH", dr[1], 5.0, 535, "deg")
            gauge(img, "YAW", dr[2], 8.0, 565, "deg")
            gauge(img, "Z", dp[2] * 1000.0, 50.0, 595, "mm")
            if direction is not None:
                gauge(img, "CROSS", cross * 1000.0, 50.0, 625, "mm")
            else:
                put(img, "CROSS: learn direction after 50 mm", 8, 630, 0.45, 1)

            put(img, f"TRAVEL {travel*1000.0:6.1f}/500 mm", 8, 665, 0.58, 2)
            put(img, f"XYZ [{dp[0]*1000:+.0f},{dp[1]*1000:+.0f},{dp[2]*1000:+.0f}] mm", 285, 665, 0.50, 1)
            put(img, f"|V| {np.linalg.norm(s['v'])*1000.0:.1f} mm/s  KF {s['kf']}", 570, 665, 0.48, 1)
            put(img, "Keep camera view centered; keep R/P/Y, Z and CROSS near zero.", 8, 705, 0.48, 1)
            if time.time() < jump_until:
                cv2.rectangle(img, (8, 715), (792, 752), (50, 50, 230), -1)
                put(img, "VIO JUMP DETECTED - measurement may be invalid", 110, 741, 0.62, 2)
            else:
                put(img, "ESC/Q: abort test", 8, 742, 0.45, 1)

        cv2.imshow(WINDOW, img)
        key = cv2.waitKey(1) & 0xFF
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

    if preview is not None:
        preview.release()
    cv2.destroyAllWindows()
    return runner_rc if runner_rc is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
