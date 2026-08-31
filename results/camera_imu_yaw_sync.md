# Camera ↔ IMU yaw timing validation

## Цель

Проверить физическую временную привязку OV9281 USB и `HIGHRES_IMU` Matek H743 после перевода обоих потоков в общую временную шкалу Raspberry Pi `CLOCK_MONOTONIC`.

Тест выполнен на жёстком узле камера + FC. Камера работала в режиме 640x480 MJPEG @ 120 FPS, IMU — `HIGHRES_IMU` @ 200 Hz. FC timestamp переводился в RPi clock affine mapping, полученным через MAVLink TIMESYNC.

## Предварительные yaw-прогоны

Три независимых обычных yaw-прогона дали одинаковую ось и знак корреляции:

| Прогон | Gyro axis | Sign | Global offset | Correlation |
|---|---|---|---:|---:|
| 1 | Z | - | -14.1 ms | -0.967 |
| 2 | Z | - | -16.8 ms | -0.774 |
| 3 | Z | - | -12.7 ms | -0.966 |

Медиана трёх глобальных оценок: `-14.1 ms`. Разброс показал, что плавные длинные повороты дают широкий максимум корреляции и не подходят для точного определения постоянного offset.

Для третьего прогона segment-level анализ дал 10 принятых сегментов, median `-12.6 ms`, MAD `6.95 ms`, P05..P95 `-24.82 .. -0.115 ms`, median `|corr|=0.959`. Эти сегментные оценки не были признаны достаточно стабильными.

Глобальная lag curve третьего прогона также показала очень плоский максимум: значения приблизительно от `-12.2` до `-15.2 ms` практически неразличимы по корреляции. Поэтому субмиллисекундную точность по обычному yaw-тесту считать физически обоснованной нельзя.

## Специальный yaw-тест со старт/стоп фронтами

Для повышения временной наблюдаемости выполнен отдельный 30-секундный тест с короткими быстрыми yaw-поворотами и выраженными остановками между ними.

Clock mapping и транспортные параметры:

- `A (RPi/FC) = 1.002045795829`
- drift = `2045.796 ppm`
- TIMESYNC good = `298/298`
- TIMESYNC RTT median = `2.771 ms`
- TIMESYNC RTT p95 = `3.011 ms`
- camera frames = `3619`
- camera source drops = `0`
- camera delivery median = `8.029 ms`
- camera delivery p95 = `8.061 ms`
- camera delivery p99 = `8.081 ms`
- camera timestamp dt median = `8.004 ms`
- camera timestamp dt p95 = `11.963 ms`
- IMU samples = `5989`
- IMU transport median = `0.505 ms`
- IMU transport p95 = `0.999 ms`
- IMU transport p99 = `1.060 ms`
- IMU transport max = `3.088 ms`

Обычный глобальный анализатор:

- gyro axis = `Z`
- sign = `-`
- camera-IMU offset = `-10.600 ms`
- correlation = `-0.996`
- samples used = `1805`

Segment-level analyzer v2:

- global offset = `-10.550 ms`
- global correlation = `-0.996`
- detected/accepted yaw segments = `15/15`
- segment offset median = `-10.150 ms`
- segment offset MAD = `0.800 ms`
- segment P16..P84 = `-12.430 .. -9.422 ms`
- segment P05..P95 = `-13.190 .. -9.110 ms`
- median `|correlation| = 0.992`
- global-vs-segment median delta = `-0.400 ms`

Все 15 сегментов выбрали ту же ось `Z` и отрицательную корреляцию. Специальный старт/стоп тест существенно уменьшил неопределённость по сравнению с плавными yaw-прогонами.

## Вывод

Физический camera-to-IMU time offset экспериментально обнаружен и yaw-синхронизация подтверждена. Для текущей конфигурации `OV9281 USB 120 FPS + HIGHRES_IMU 200 Hz + TIMESYNC affine mapping` рабочая оценка постоянного offset принимается приблизительно как `-10.5 ms` в соглашении текущего анализатора:

- positive offset = camera measurement occurs later than IMU;
- negative offset = camera measurement occurs earlier than IMU.

Перед внесением компенсации в live timestamp path необходимо отдельно проверить математический знак применения offset. Поэтому сама компенсация пока не считается реализованной.

Текущие диагностические файлы на RPi:

- `/home/vio/camera_imu_yaw.csv`
- `/home/vio/camera_imu_yaw_camera.csv`
- `/home/vio/camera_imu_yaw.mjpg`
- `/home/vio/camera_imu_yaw_segments.csv`
- `/home/vio/camera_imu_yaw_lag_curve.csv`
- `/home/vio/camera_imu_yaw_visual_v2.csv`
