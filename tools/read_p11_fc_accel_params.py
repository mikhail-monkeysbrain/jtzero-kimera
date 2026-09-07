#!/usr/bin/env python3
import argparse
import time
try:\n    from pymavlink import mavutil\nexcept ModuleNotFoundError as exc:\n    raise SystemExit(\n        "ОШИБКА: модуль pymavlink не установлен для этого Python.\\n"\n        "Этот диагностический скрипт не должен добавлять новую Python-зависимость в рабочее окружение.\\n"\n        "Используйте существующий C++/MAVLink путь проекта; установка pymavlink не требуется."\n    ) from exc

WANTED_PREFIXES = (
    "INS_ACC",
    "INS_USE",
    "INS_POS",
    "INS_ACCEL_FILTER",
    "AHRS_ORIENTATION",
    "AHRS_TRIM",
)

def decode_param_id(value):
    if isinstance(value, bytes):
        return value.split(b"\x00",1)[0].decode("ascii","replace")
    return str(value).split("\x00",1)[0]

def main():
    ap=argparse.ArgumentParser(description="Read-only dump of accelerometer-related ArduPilot parameters for P11")
    ap.add_argument("--serial", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--timeout", type=float, default=12.0)
    args=ap.parse_args()

    print(f"[MAV] подключение {args.serial} @ {args.baud}")
    mav=mavutil.mavlink_connection(args.serial, baud=args.baud, autoreconnect=False)
    hb=mav.wait_heartbeat(timeout=5)
    if hb is None:
        raise SystemExit("HEARTBEAT timeout")
    print(f"[MAV] HEARTBEAT SYS={mav.target_system} COMP={mav.target_component}")
    print("[MAV] запрос списка параметров (только чтение)...")
    mav.mav.param_request_list_send(mav.target_system, mav.target_component)

    params={}
    expected=None
    last=time.monotonic()
    deadline=time.monotonic()+args.timeout
    while time.monotonic()<deadline:
        msg=mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            if params and time.monotonic()-last>1.5:
                break
            continue
        name=decode_param_id(msg.param_id)
        params[name]=float(msg.param_value)
        expected=int(msg.param_count)
        last=time.monotonic()
        if expected and len(params)>=expected:
            break

    mav.close()
    print(f"[MAV] получено параметров: {len(params)}" + (f" / ожидается {expected}" if expected else ""))

    selected={k:v for k,v in params.items() if any(k.startswith(p) for p in WANTED_PREFIXES)}
    print("\n================ P11 FC ACCEL PARAMS ================")
    for k in sorted(selected):
        print(f"{k:20s} = {selected[k]:+.9f}")

    print("\n================ KEY GROUPS ================")
    groups=[
        ("IMU1 scale",("INS_ACCSCAL_X","INS_ACCSCAL_Y","INS_ACCSCAL_Z")),
        ("IMU1 offset",("INS_ACCOFFS_X","INS_ACCOFFS_Y","INS_ACCOFFS_Z")),
        ("IMU2 scale",("INS_ACC2SCAL_X","INS_ACC2SCAL_Y","INS_ACC2SCAL_Z")),
        ("IMU2 offset",("INS_ACC2OFFS_X","INS_ACC2OFFS_Y","INS_ACC2OFFS_Z")),
        ("IMU IDs",("INS_ACC_ID","INS_ACC2_ID","INS_ACC3_ID")),
        ("IMU use",("INS_USE","INS_USE2","INS_USE3")),
        ("AHRS trim",("AHRS_TRIM_X","AHRS_TRIM_Y","AHRS_TRIM_Z")),
        ("Board orientation",("AHRS_ORIENTATION",)),
    ]
    for title,names in groups:
        print(title+":")
        for n in names:
            if n in params:
                print(f"  {n}={params[n]:+.9f}")
            else:
                print(f"  {n}=<нет в полученном списке>")

    print("\nINTERPRETATION:")
    print("- Этот скрипт ничего не записывает в FC и не меняет параметры.")
    print("- Разные scale/offset IMU1 и IMU2 сами по себе нормальны после калибровки.")
    print("- Нас интересует, может ли ориентационно-зависимый norm shift быть совместим с существующей per-axis calibration, а не требуется ли новый физический тест.")

if __name__=="__main__":
    main()
