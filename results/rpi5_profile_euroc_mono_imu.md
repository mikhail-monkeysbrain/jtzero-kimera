# Raspberry Pi 5 profiling — Kimera-VIO Mono + IMU

## Test setup

- Platform: Raspberry Pi 5 Model B Rev 1.0, 4 GiB RAM
- OS: Debian GNU/Linux 13 (trixie), ARM64
- Kimera-VIO: upstream commit `ce8c59b7b273ab5ac29db7e5572e1623760e19c7`
- Dataset: EuRoC `V1_01_easy`
- Mode: Mono + IMU, `params/EurocMono`
- Input sequence: 2912 camera frames
- Headless run: visualization, LCD and frontend image output disabled

## Main profiling run

`/usr/bin/time -v`:

- Wall time: 34.02 s
- User time: 92.21 s
- System time: 8.57 s
- Process CPU: 296%
- Maximum RSS: 580336 KiB (~567 MiB)
- Swaps: 0
- Exit status: 0

Derived input throughput:

- 2912 / 34.02 = ~85.6 input frames/s
- ~4.28x EuRoC real-time at 20 FPS
- ~2.85x a 30 FPS input rate by pure offline throughput comparison

## CPU per core

Separate full EuRoC run, 64 samples:

- CPU0: mean 68.6%, min 8.0%, max 100.0%
- CPU1: mean 66.3%, min 3.8%, max 92.6%
- CPU2: mean 92.8%, min 2.0%, max 100.0%
- CPU3: mean 75.4%, min 40.0%, max 100.0%
- `/usr/bin/time`: process CPU 295%
- Wall time: 33.54 s
- Throughput: ~86.8 input frames/s
- Maximum RSS: 882944 KiB
- Swaps: 0
- Exit status: 0

CPU2 is the most heavily loaded core, but the complete pipeline still finishes substantially faster than real-time.

## Temperature, frequency and throttling

64-sample system profile during a full run:

- Mean CPU temperature: 63.3 °C
- Minimum: 52.9 °C
- Maximum: 65.5 °C
- Throttling: `0x0` for all 64 samples

CPU frequency:

- CPU0 mean 2386 MHz, min 1500 MHz, max 2400 MHz
- CPU1 mean 2386 MHz, min 1500 MHz, max 2400 MHz
- CPU2 mean 2389 MHz, min 1700 MHz, max 2400 MHz
- CPU3 mean 2389 MHz, min 1700 MHz, max 2400 MHz

The Pi therefore sustains approximately 2.4 GHz under the Kimera workload without thermal throttling or undervoltage indication.

## RAM and swap

System memory profile during the full run:

- RAM used: 459.6–941.7 MiB
- Swap already occupied by the system: 163.6 MiB throughout the profile
- Kimera `/usr/bin/time` reported `Swaps: 0`

The constant 163.6 MiB swap occupancy was not caused by active swapping by the profiled Kimera process during the run.

## Sustained 30 FPS qualification

Three complete EuRoC runs were executed consecutively without cooldown pauses.

| Run | Wall time | Throughput | CPU | Max RSS | Temperature start/end | Throttling | Exit |
|---|---:|---:|---:|---:|---:|---|---:|
| 1 | 33.010 s | 88.22 FPS | 297% | 1102368 KiB | 54.0 → 64.5 °C | 0x0 → 0x0 | 0 |
| 2 | 33.520 s | 86.87 FPS | 295% | 1037248 KiB | 64.5 → 65.5 °C | 0x0 → 0x0 | 0 |
| 3 | 32.990 s | 88.27 FPS | 297% | 1150560 KiB | 65.5 → 67.8 °C | 0x0 → 0x0 | 0 |

Sustained summary:

- Average throughput: 87.79 FPS
- Minimum throughput: 86.87 FPS
- Required qualification threshold: 30.00 FPS
- Mean sampled temperature: 67.1 °C
- Minimum sampled temperature: 54.0 °C
- Maximum sampled temperature: 70.0 °C
- Throttling samples: 97 × `0x0`
- Result: **PASS**

This is an offline compute-throughput qualification using EuRoC data. It demonstrates substantial compute margin relative to a 30 FPS target, but it is not equivalent to a live 30 FPS camera/IMU pipeline; live acquisition, timestamp synchronization, USB/CSI latency and RAW IMU ingestion still need to be measured in later stages.

## Working RPi5 profile decision

The current `EurocMono` frontend/backend configuration is retained unchanged. A lighter frontend profile is not required at this stage because:

- sustained throughput remains at least 86.87 FPS across three consecutive full runs;
- no thermal throttling occurs up to 70.0 °C;
- process CPU is about 295–297%, not a four-core saturation;
- RAM remains within the 4 GiB platform budget;
- no Kimera process swaps were reported.

The next performance qualification should be repeated with live OV9281 + RAW IMU after camera/IMU synchronization is implemented.
