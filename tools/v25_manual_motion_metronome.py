#!/usr/bin/env python3
"""Manual-motion metronome for reproducible JT-Zero 500 mm tests.

Does not touch Kimera or VIO data. It only provides a repeatable audible/text
schedule for the operator so CONTROL and TEST runs use the same motion timing.
"""

import argparse
import sys
import time


def beep():
    # Terminal bell; harmless if the terminal/audio path ignores it.
    print("\a", end="", flush=True)


def countdown(seconds: int, label: str):
    print(f"\n[{label}] {seconds} s")
    for left in range(seconds, 0, -1):
        print(f"  {left:2d}", flush=True)
        beep()
        time.sleep(1)


def motion_phase(direction: str, seconds: float):
    print(f"\n>>> {direction}: ДВИЖЕНИЕ 500 мм за {seconds:.1f} s <<<", flush=True)
    beep()
    start = time.monotonic()
    tick = 0
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= seconds:
            break
        target = min(seconds, tick + 1)
        sleep_for = target - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)
        tick += 1
        if tick < seconds:
            print(f"  {tick:2d} s", flush=True)
            beep()
    print(">>> СТОП <<<", flush=True)
    beep()
    beep()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--move-sec", type=float, default=8.0,
                   help="время перемещения на 500 мм")
    p.add_argument("--settle-sec", type=int, default=5,
                   help="покой перед каждым участком")
    p.add_argument("--start-sec", type=int, default=5,
                   help="обратный отсчёт перед началом")
    p.add_argument("--cycles", type=int, default=2,
                   help="число циклов A->B->A")
    args = p.parse_args()

    if args.move_sec <= 0 or args.settle_sec < 0 or args.cycles < 1:
        sys.exit("Некорректные параметры")

    print("JT-Zero manual 500 mm protocol")
    print(f"500 мм / {args.move_sec:.1f} s = {500.0/args.move_sec:.1f} мм/с nominal")
    print("Правило: двигать плавно и непрерывно; не пытаться подгонять VIO HUD.")

    countdown(args.start_sec, "ГОТОВНОСТЬ")

    for cycle in range(1, args.cycles + 1):
        countdown(args.settle_sec, f"ЦИКЛ {cycle}: ПОКОЙ В A")
        motion_phase("A -> B", args.move_sec)
        countdown(args.settle_sec, f"ЦИКЛ {cycle}: ПОКОЙ В B")
        motion_phase("B -> A", args.move_sec)

    countdown(args.settle_sec, "ФИНАЛЬНЫЙ ПОКОЙ В A")
    print("\nPROTOCOL COMPLETE", flush=True)
    beep()


if __name__ == "__main__":
    main()
