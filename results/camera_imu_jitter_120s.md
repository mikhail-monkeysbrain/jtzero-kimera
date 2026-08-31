# Camera + IMU jitter validation — 120 s

## Цель

Измерить jitter камеры OV9281 USB и HIGHRES_IMU/MAVLink/serial одним длительным прогоном после введения общей временной шкалы и компенсации camera-to-IMU offset `+10.5 ms`.

## Конфигурация

- Raspberry Pi 5
- OV9281 USB UVC, 640x480 MJPEG @ 120 FPS
- Matek H743, `HIGHRES_IMU` @ 200 Hz через `/dev/ttyAMA0` @ 460800
- MAVLink TIMESYNC @ 10 Hz
- `CLOCK_MONOTONIC`
- camera timestamp correction: `+10.500 ms`
- длительность каждого контрольного прогона: 120 s

## Прогон 1 — MJPEG записывается на обычный накопитель

Итог logger:

- clock mapping `A = 1.002042066591`, drift `2042.067 ppm`
- TIMESYNC good `1164/1164`
- TIMESYNC RTT median `2.419 ms`, P95 `2.919 ms`
- camera frames `14149`
- camera source drops `330`
- camera delivery median `8.029 ms`, P95 `8.062 ms`, P99 `8.083 ms`, max `1599.617 ms`
- camera timestamp dt median `8.004 ms`, P95 `11.961 ms`
- IMU samples `23951`
- IMU transport median `0.686 ms`, P95 `1.200 ms`, P99 `590.347 ms`, max `1596.297 ms`

Расширенный анализ:

- camera frame dt: median `8.004 ms`, P95 `11.961 ms`, P99 `11.976 ms`, max `1568.062 ms`
- camera absolute dt deviation from median: P95 `3.957 ms`, P99 `3.972 ms`, max `1560.058 ms`
- IMU mapped dt: median `5.011 ms`, P95 `5.288 ms`, P99 `5.401 ms`, max `5.832 ms`
- IMU absolute dt deviation from median: P95 `0.328 ms`, P99 `0.593 ms`, max `0.821 ms`
- TIMESYNC RTT: median `2.419 ms`, P95 `2.919 ms`, P99 `4.057 ms`, max `5.688 ms`

Обнаружены четыре больших camera gap:

1. `224.010 ms`, lost `26` frames
2. `200.002 ms`, lost `23` frames
3. `752.054 ms`, lost `90` frames
4. `1568.062 ms`, lost `188` frames

IMU transport latency >10 ms наблюдалась у `567` samples; >500 ms у `276`; >1000 ms у `120` samples. При этом собственный mapped IMU dt оставался в пределах `4.207..5.832 ms`, то есть FC продолжал формировать поток примерно 200 Hz, а задержка возникала при обслуживании потока на RPi.

## Прогон 2 — MJPEG перенаправлен в `/dev/shm`

Для отделения sensor/transport jitter от блокировок файлового I/O `/home/vio/camera_imu_yaw.mjpg` был временно заменён symlink на `/dev/shm/camera_imu_yaw.mjpg`.

Итог logger:

- clock mapping `A = 1.002041129625`, drift `2041.130 ppm`
- TIMESYNC good `1195/1195`
- TIMESYNC RTT median `2.379 ms`, P95 `2.917 ms`
- camera frames `14475`
- camera source drops `4`
- camera delivery median `8.029 ms`, P95 `8.063 ms`, P99 `8.084 ms`, max `12.304 ms`
- camera timestamp dt median `8.004 ms`, P95 `11.961 ms`
- IMU samples `23951`
- IMU transport median `0.718 ms`, P95 `1.206 ms`, P99 `1.276 ms`, min `0.636 ms`, max `4.501 ms`

## Вывод

Обычный jitter sensor timestamps и MAVLink transport мал. Большие задержки порядка `1.6 s` из первого прогона не являются jitter самого HIGHRES_IMU/TIMESYNC и не являются постоянной характеристикой камеры. Они практически исчезают при записи MJPEG в RAM.

Это подтверждает, что длинные stall в диагностическом logger связаны с синхронной записью большого MJPEG-потока на обычный накопитель в том же цикле, который обслуживает camera dequeue и UART/MAVLink.

Рабочие измеренные значения при исключённом дисковом stall:

- camera delivery: median `8.029 ms`, P95 `8.063 ms`, P99 `8.084 ms`, max `12.304 ms`
- camera source drops: `4` за 120 s при 120 FPS
- IMU transport: median `0.718 ms`, P95 `1.206 ms`, P99 `1.276 ms`, max `4.501 ms`
- TIMESYNC RTT: median `2.379 ms`, P95 `2.917 ms`
- IMU sample interval из первого расширенного анализа: median `5.011 ms`, absolute jitter P95 `0.328 ms`, P99 `0.593 ms`, max `0.821 ms`

Для live Kimera-VIO файловый I/O не должен блокировать sensor capture. Диагностическую запись MJPEG следует вынести в отдельный asynchronous writer либо полностью отключать в обычном live-режиме.