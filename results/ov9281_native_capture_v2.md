# OV9281 native capture v2

Дата проверки: 2026-08-31.

## Конфигурация

- Raspberry Pi 5, ARM64
- OV9281 через USB UVC (`uvcvideo`)
- устройство захвата: `/dev/video0`
- вход: 640x480 MJPEG @ 120 FPS
- V4L2 mmap buffers: 6
- manual exposure: 50
- gain: 0
- dynamic framerate: off
- выходной поток: grayscale `CV_8UC1`, target 30 FPS
- temporal selection: ближайший реальный кадр к сетке 33.333333 ms
- timestamp выбранного кадра не синтезируется: сохраняется исходный V4L2 timestamp
- MJPEG декодируется только для выбранных кадров; отброшенные source frames не копируются и не декодируются

## Проверенный прогон

Команда:

```bash
cd ~/Kimera-VIO
./tools/ov9281_native_capture 30 30
```

Во время 30-секундного теста камера перемещалась, сцена и яркость существенно менялись.

Результат:

```text
source_frames     = 3610
source_drops      = 0
selected_frames   = 891
decode_errors     = 0
skipped_targets   = 0
timestamp_span    = 29.669408 s
output_fps        = 29.997
dt_min_ms         = 24.003
dt_mean_ms        = 33.336
dt_p95_ms         = 40.016
dt_max_ms         = 40.030
target_abs_mean_ms= 2.114
target_abs_p95_ms = 3.967
target_abs_max_ms = 5.955
```

CSV: `/home/vio/ov9281_native_capture_v2.csv`

Сохранённые grayscale-кадры: `/home/vio/ov9281_native_frames_v2`

## Вывод

Native V4L2 capture v2 прошёл проверку. За 3610 исходных кадров не обнаружено V4L2 sequence gaps, ошибок MJPEG decode или пропущенных 30-Hz target slots. Средняя выходная частота по реальным timestamps составляет 29.997 FPS, средний межкадровый интервал 33.336 ms.

`dt_min_ms` и `dt_max_ms` не должны искусственно приводиться к 33.333 ms: live VIO должен получать реальные timestamps выбранных кадров. P95 абсолютной ошибки выбора относительно целевой 30-Hz сетки составляет 3.967 ms, максимум 5.955 ms.

V4L2 маркирует timestamps как monotonic/SOE. Это пока не считается доказательством физического sensor exposure timestamp через USB UVC bridge; связь с camera metadata и RAW IMU clock должна быть проверена отдельно на этапе временной синхронизации.

## Связанные настройки frontend

Для OV9281 выбран GFTT baseline:

```yaml
feature_detector_type: 3
maxFeaturesPerFrame: 300
quality_level: 0.005
min_distance: 20
block_size: 3
```

Порог `quality_level: 0.005` подтверждён сравнительным live-тестом и применён в локальном `params/EurocMono/FrontendParams.yaml`.
