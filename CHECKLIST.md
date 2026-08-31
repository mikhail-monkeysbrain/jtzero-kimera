# Чеклист jtzero-kimera

Этот файл отражает фактический прогресс реализации плана из `README.md`.

Правило ведения:
- `[ ]` — этап ещё не завершён;
- `[x]` — этап полностью реализован и проверен;
- пункт отмечается выполненным только после фактического завершения соответствующего этапа.

## Этап 1. Проверить текущее окружение Raspberry Pi 5

- [x] Подтвердить модель Raspberry Pi 5 — Raspberry Pi 5 Model B Rev 1.0
- [x] Подтвердить 64-bit ARM64 архитектуру — aarch64
- [x] Зафиксировать версию Debian / Raspberry Pi OS — Debian GNU/Linux 13 (trixie), 13.5
- [x] Зафиксировать версию ядра — 6.18.34+rpt-rpi-2712
- [x] Зафиксировать объём RAM и swap — 4.0 GiB RAM, 2.0 GiB swap
- [x] Проверить свободное место — 6.6 GiB свободно на корневом разделе
- [x] Проверить охлаждение и температуру CPU — активный кулер, температура 52.1°C, throttling=0x0
- [x] Зафиксировать версии GCC, CMake и Git — GCC/G++ 14.2.0, Git 2.47.3, CMake до этапа 2 не был установлен
- [x] **Этап 1 завершён**

## Этап 2. Подготовить системные зависимости

- [x] Обновить список пакетов
- [x] Установить базовые build-инструменты — build-essential, CMake 3.31.6, Ninja 1.12.1, pkg-config
- [x] Установить Eigen — 3.4.0-5
- [x] Установить OpenCV development packages — OpenCV 4.10.0
- [x] Установить glog и gflags — glog 0.6.0-2.1+b2, gflags 2.2.2-2+b1
- [x] Установить Boost — 1.83.0.2+b2
- [x] Установить SuiteSparse — 1:7.10.1+dfsg-1
- [x] Установить TBB — 2022.1.0-1+deb13u1
- [x] Зафиксировать фактически необходимые пакеты для текущей ОС — Debian 13 ARM64; после установки свободно 5.3 GiB
- [x] **Этап 2 завершён**

## Этап 3. Собрать зависимости Kimera

- [x] Собрать / установить GTSAM совместимой версии — GTSAM 4.2, tag 4.2 / commit 4f66a491ffc83cf092d0d818b11dc35135521612, Release, system Eigen 3.4.0, TBB, Pose3/Rot3 ExpMap ON, tangent preintegration OFF, GTSAM_BUILD_UNSTABLE=ON; libgtsam.so и libgtsam_unstable.so установлены в /usr/local, CMake packages GTSAM и GTSAM_UNSTABLE установлены
- [x] Собрать / установить OpenGV — commit 91f4b19c73450833a40e463ad3648aae80b3a7f3, Release, system Eigen /usr/include/eigen3, tests/python OFF, ARM64 -march=armv8-a, статическая libopengv.a установлена в /usr/local
- [x] Собрать / установить DBoW2 — commit 3924753db6145f12618e7de09b7e6b258db93c6e, Release, demo OFF, OpenCV 4.10.0, shared libDBoW2.so установлена в /usr/local
- [x] Собрать / установить Kimera-RPGO — commit d28b4df0570d642a2bb00e511344ce1110f87519, Release, GTSAM из /usr/local/lib/cmake/GTSAM, tests OFF, shared libKimeraRPGO.so установлена в /usr/local
- [x] Проверить обнаружение OpenCV, Eigen, glog и gflags через CMake — smoke-test успешно обнаружил OpenCV 4.10.0, Eigen 3.4.0, glog и gflags; также GTSAM, OpenGV, DBoW2 и Kimera-RPGO
- [x] Сохранить версии и параметры сборки зависимостей — итоговая конфигурация зафиксирована; GTSAM дополнен GTSAM_BUILD_UNSTABLE=ON, установлен и доступен через CMake как GTSAM_UNSTABLE
- [x] **Этап 3 завершён**

## Этап 4. Собрать Kimera-VIO без ROS

- [x] Клонировать Kimera-VIO — shallow clone текущего master в /home/vio/Kimera-VIO
- [x] Зафиксировать используемый commit/tag Kimera-VIO — master, commit ce8c59b7b273ab5ac29db7e5572e1623760e19c7
- [x] Создать отдельный build-каталог — /home/vio/Kimera-VIO/build
- [x] Успешно выполнить CMake configure — Release, Ninja, KIMERA_BUILD_TESTS=OFF; GTSAM/GTSAM_UNSTABLE/OpenGV/DBoW2/Kimera-RPGO обнаружены; Pangolin отсутствует и является optional
- [x] Успешно собрать Kimera-VIO на ARM64 — Debian 13 / GCC 14.2; после минимальных compatibility fixes успешно собраны libkimera_vio.so и stereoVIOEuroc
- [x] Сохранить build log — /home/vio/Kimera-VIO/build_kimera_vio.log
- [x] Проверить запуск основных исполняемых файлов — stereoVIOEuroc является ARM64 ELF, все shared libraries найдены через ldd, --help выполняется с exit code 0
- [x] **Этап 4 завершён**

## Этап 5. Первый запуск на EuRoC

- [x] Скачать / подготовить EuRoC V1_01_easy — распакованный датасет /home/vio/datasets/euroc/V1_01_easy, 2912 кадров cam0 + 2912 cam1, IMU и ground truth; все 7 sensor YAML подготовлены штатным yamelize.bash и имеют OpenCV-заголовок %YAML:1.0
- [x] Настроить режим Mono + IMU — params/EurocMono; Regular VIO Backend; loop closure, 3D visualizer и frontend image visualization отключены для headless RPi5; smoke-test frames 50..200 завершён с exit code 0 и `Pipeline successful? Yes!`
- [x] Запустить полный pipeline без критических ошибок — frames 0..2912, exit status 0, `Pipeline successful? Yes!`, FATAL/ERROR/segfault отсутствуют; Spin took 40057 ms
- [x] Проверить обработку всех кадров — вход cam0 содержит 2912 кадров за 145.550 s; pipeline запущен на диапазоне 0..2912 и завершён штатно; последний VIO/frontend timestamp 1403715418712143104 находится за 100 ms до последнего входного кадра, что соответствует keyframe/backend cadence, а не преждевременной остановке
- [x] Измерить средний FPS — offline throughput 2912 / 40.057 s = 72.7 input frames/s, примерно 3.64x от реального времени EuRoC 20 FPS; /usr/bin/time wall 40.96 s, CPU 338%, max RSS ~1.03 GiB, swaps 0
- [x] Проверить пропуски кадров — `ThreadsafeQueue::push()` не удаляет элементы при росте очереди; Mono EuRoC callback использует неограниченный `fillLeftFrameQueue`; в полном логе отсутствуют `Dropping frame`, ошибки IMU sync и IMU wait events; неожиданных пропусков кадров не выявлено
- [x] Проверить стабильность выходной траектории — 728 VIO states с ровным 0.200 s cadence и без irregular gaps; ATE RMSE 0.921396 m, final common error 1.714645 m, path length error -3.86%; absolute orientation error растёт с 0.2249 до 5.2351 deg; RPE 1 s translation RMSE 0.085976 m / rotation RMSE 0.4457 deg, RPE 10 s 0.466713 m / 3.1551 deg; pipeline не расходится катастрофически, но накопленный drift заметен
- [x] Сохранить результаты первого теста — `results/euroc_v1_01_easy_mono_imu_rpi5.md`
- [x] **Этап 5 завершён**

## Этап 6. Профилирование Raspberry Pi 5

- [x] Измерить загрузку CPU по ядрам — CPU0 68.6%, CPU1 66.3%, CPU2 92.8%, CPU3 75.4% в среднем; Kimera process CPU 295–297%
- [x] Измерить температуру CPU — основной профиль 52.9–65.5°C, sustained-тест 30 FPS: максимум 70.0°C
- [x] Проверить частоту CPU и throttling — средняя частота ядер 2386–2389 MHz, максимум 2400 MHz; throttling=0x0 во всех измерениях, включая 97 samples sustained-теста
- [x] Измерить использование RAM — системная RAM 459.6–941.7 MiB в основном профиле; максимальный RSS Kimera в профильных запусках ~567 MiB–1.10 GiB
- [x] Измерить использование swap — системный swap стабильно 163.6 MiB уже занят, `/usr/bin/time` для Kimera: Swaps=0
- [x] Измерить реальную скорость обработки Kimera-VIO — полные EuRoC прогоны ~85.6–88.3 input FPS; sustained 3-run average 87.79 FPS, minimum 86.87 FPS
- [x] Определить, достигается ли real-time — да; sustained qualification порог 30 FPS пройден с минимумом 86.87 FPS, все три прогона exit 0 и без throttling
- [x] При необходимости подобрать облегчённые настройки frontend — не требуется на текущем этапе: текущий EurocMono имеет значительный вычислительный запас; качество не ухудшаем преждевременной оптимизацией
- [x] Зафиксировать рабочий профиль RPi5 — результаты сохранены в `results/rpi5_profile_euroc_mono_imu.md`; live 30 FPS с OV9281 + RAW IMU необходимо повторно квалифицировать после реализации захвата и синхронизации
- [x] **Этап 6 завершён**

## Этап 7. Подключить OV9281

- [x] Подключить OV9281 к RPi5 — USB UVC, `/dev/video0`, драйвер `uvcvideo`
- [x] Подтвердить стабильный захват grayscale — MJPEG декодируется в 640x480 `CV_8UC1`; в проверенном v2-прогоне 891 выбранный кадр, `decode_errors=0`
- [x] Настроить 640x480 — вход 640x480 MJPEG
- [x] Настроить начальные 30 FPS — USB-камера работает 640x480 MJPEG @ 120 FPS; native temporal selector формирует ~30 FPS с сохранением реальных V4L2 timestamps; проверено `output_fps=29.997`
- [x] Зафиксировать короткую экспозицию — manual `exposure_time_absolute=50`; тест движения не выявил критичного directional blur
- [x] Зафиксировать gain — `gain=0`
- [x] Отключить нежелательные автоматические изменения изображения — `white_balance_automatic=0`, `power_line_frequency=0`, `backlight_compensation=0`; также manual exposure, `exposure_dynamic_framerate=0`, exposure=50 и gain=0 выставляются программой при каждом запуске; контрольный прогон подтверждён
- [x] Проверить стабильность timestamps кадров — native v2: `source_frames=3610`, `source_drops=0`, `skipped_targets=0`, `dt_mean=33.336 ms`, target error P95=3.967 ms; V4L2 flags monotonic/SOE; sensor-native semantics USB bridge будут отдельно проверены на этапе 9
- [x] Сохранить тестовую последовательность кадров — `/home/vio/ov9281_motion_10s.mjpg`, `/home/vio/ov9281_motion_10s_timestamps.txt`, `/home/vio/ov9281_native_capture_v2.csv`, `/home/vio/ov9281_native_frames_v2`; результаты v2 сохранены в `results/ov9281_native_capture_v2.md`
- [x] **Этап 7 завершён**

## Этап 8. Получить RAW IMU с Matek H743

- [x] Определить MAVLink-сообщение / источник raw gyro и accelerometer — `HIGHRES_IMU` (MAVLink message id 105) от Matek H743 через UART3 → `/dev/ttyAMA0`, MAVLink2
- [x] Получить angular velocity X/Y/Z — `xgyro/ygyro/zgyro` в rad/s
- [x] Получить linear acceleration X/Y/Z — `xacc/yacc/zacc` в m/s²
- [x] Получить timestamp для каждого IMU sample — `HIGHRES_IMU.time_usec`; контрольный лог не содержит non-positive timestamp deltas
- [x] Добиться стабильной частоты не менее 200 Hz — UART3 460800 baud, запрос 200 Hz; 30 s validation: RX 199.591 Hz, IMU timestamp rate 200.001 Hz
- [x] Проверить возможность работы в диапазоне 200–400 Hz — 200 Hz воспроизводится точно; запрос 300 Hz квантуется примерно в 333.4 Hz с большим jitter; 400 Hz отклоняется `MAV_RESULT_DENIED`; рабочая частота зафиксирована 200 Hz
- [x] Проверить пропуски и jitter потока IMU — 5988 samples за 30.002 s; dt mean 5.000 ms, P95 5.275 ms, P99 5.374 ms, max 5.769 ms; absolute jitter P95 0.325 ms, P99 0.581 ms, max 0.769 ms; large_gaps=0, estimated_missing=0
- [x] Сохранить сырой IMU-лог — `/home/vio/highres_imu_200hz_30s.csv`; результаты и рабочая конфигурация зафиксированы в `results/matek_h743_highres_imu_200hz.md`
- [x] **Этап 8 завершён**

## Этап 9. Временная синхронизация камеры и IMU

- [x] Привести camera и IMU timestamps к общей временной шкале — единый native C++ logger использует RPi `CLOCK_MONOTONIC`; camera V4L2 timestamp уже monotonic/SOE, FC `HIGHRES_IMU.time_usec` переводится affine mapping `t_rpi = RPi_ref + A*(t_fc-FC_ref)` по MAVLink TIMESYNC; 30 s native validation: A=1.002061378243, drift=2061.378 ppm, TIMESYNC 300/300 good; 300 s independent validation: A=1.002066836, drift=2066.836 ppm; результаты в `results/camera_imu_common_clock.md`
- [x] Измерить постоянный camera-to-IMU time offset — yaw correlation на общей временной шкале подтверждён; специальный start/stop yaw-тест дал global `-10.55 ms`, segment median `-10.15 ms`, MAD `0.80 ms`, P05..P95 `-13.19..-9.11 ms`, median |corr| `0.992`; рабочая оценка `≈ -10.5 ms`, результаты в `results/camera_imu_yaw_sync.md`
- [x] Измерить jitter камеры — 120 s validation; при исключённом блокирующем disk I/O: 14475 frames, 4 source drops, delivery median `8.029 ms`, P95 `8.063 ms`, P99 `8.084 ms`, max `12.304 ms`; отдельный disk-backed прогон выявил диагностические stalls до `1.6 s`, устраняющиеся при записи MJPEG в `/dev/shm`; результаты в `results/camera_imu_jitter_120s.md`
- [x] Измерить jitter IMU / MAVLink / serial — 120 s validation; 23951 samples, transport median `0.718 ms`, P95 `1.206 ms`, P99 `1.276 ms`, max `4.501 ms` при записи MJPEG в `/dev/shm`; mapped IMU dt median `5.011 ms`, absolute jitter P95 `0.328 ms`, P99 `0.593 ms`, max `0.821 ms`; disk-backed stalls диагностического logger не являются jitter FC/IMU; результаты в `results/camera_imu_jitter_120s.md`
- [x] Реализовать компенсацию постоянного offset при необходимости — camera timestamp сдвигается вперёд на `+10.5 ms`; компенсированный start/stop yaw validation дал residual global offset `-0.150 ms`, global corr `-0.964`, segment median `-0.150 ms`; знак компенсации подтверждён отдельным verifier; значение относится к текущему OV9281 USB UVC 640x480 MJPEG @ 120 FPS и подлежит повторной калибровке при смене интерфейса/режима
- [x] Проверить синхронизацию на yaw-движении — во всех независимых тестах лучшая ось `Z`, знак корреляции отрицательный; специальный start/stop тест: global corr `-0.996`, 15/15 segment accepted, median |corr| `0.992`
- [x] Исключить крупные скачки межкадрового угла из-за timestamp mismatch — source runtime policy до temporal decimation проверяет непрерывность `V4L2 sequence`, положительный corrected timestamp delta и верхнюю границу `20 ms` только для исходного 120 FPS UVC-потока; этот порог не применяется к выбранному ~30 FPS потоку Kimera. Проверено `camera_timestamp_policy_check`: 3620 frames, 1 потерянный source frame обнаружен по sequence при `dt=7.998 ms`, rejected pairs=1, corrected timestamp mismatches=0, `RESULT: PASS`
- [x] Зафиксировать итоговую схему timestamping — `results/camera_imu_timestamp_scheme.md`: единый `CLOCK_MONOTONIC`; camera=`V4L2 source timestamp + 10.5 ms` для текущей USB-конфигурации; selected ~30 FPS кадры сохраняют реальные corrected source timestamps без синтетического cadence; IMU=`HIGHRES_IMU.time_usec` через непрерывный affine TIMESYNC mapping; receive-time только diagnostic; при невалидном/stale TIMESYNC mapping синхронизированный IMU не подаётся и receive-time fallback запрещён
- [x] **Этап 9 завершён**

## Этап 10. Калибровка камеры

- [x] Зафиксировать финальную оптику OV9281 — текущая линза и фокус зафиксированы; после калибровки оптика больше не изменяется
- [x] Зафиксировать рабочее разрешение — OV9281 USB UVC, 640x480; calibration и independent validation выполнены в этом разрешении
- [x] Выполнить калибровку intrinsic parameters — ChArUco 7x5, DICT_4X4_50, squareLength=27.324 mm, markerLength=20.043 mm; 63 usable views
- [x] Получить fx и fy — fx=568.53170752165227, fy=569.68005562865858
- [x] Получить cx и cy — cx=315.98271077441063, cy=239.88148589100641
- [x] Получить distortion coefficients — radtan_5: k1=0.073569192194028493, k2=-0.095253893789117, p1=-0.010810530757187299, p2=-0.0022843373576970235, k3=0.082177400802757483
- [x] Проверить reprojection error — calibration RMS=0.3473217318 px; independent validation на 49 новых кадрах: aggregate view RMSE=0.407339 px, median=0.379265 px, max=0.686381 px, 0 views >0.75 px, RESULT=PASS; визуальный undistortion check также PASS
- [x] Сохранить calibration-файл в репозитории — `calibration/ov9281_intrinsics.yaml`; итоговый отчёт `results/ov9281_intrinsics_validation.md`
- [x] **Этап 10 завершён**

## Этап 11. Калибровка Camera ↔ IMU

- [ ] Зафиксировать системы координат камеры и IMU
- [ ] Определить Camera↔IMU rotation
- [ ] Определить Camera↔IMU translation
- [ ] Проверить направления и знаки всех осей
- [ ] Проверить extrinsics экспериментально
- [ ] Сохранить extrinsic calibration в репозитории
- [ ] **Этап 11 завершён**

## Этап 12. Live Mono + IMU

- [ ] Подать live OV9281 grayscale в Kimera-VIO
- [ ] Подать live RAW IMU в Kimera-VIO
- [ ] Использовать синхронизированные timestamps
- [ ] Получить устойчивый live pose
- [ ] Получить velocity
- [ ] Реализовать логирование входных данных и результата
- [ ] Провести тест в покое
- [ ] Провести тест известного линейного перемещения
- [ ] Провести тест возврата в исходную точку
- [ ] Провести отдельный yaw-тест
- [ ] Провести тест комбинированного движения
- [ ] **Этап 12 завершён**

## Этап 13. Сравнить с jtzero-optical-flow-mvp

- [ ] Сравнить drift в покое
- [ ] Сравнить ошибку на перемещении 500 мм
- [ ] Сравнить поведение при yaw
- [ ] Сравнить зависимость результата от скорости движения
- [ ] Сравнить восстановление после потери features
- [ ] Сравнить загрузку CPU
- [ ] Сравнить RAM
- [ ] Сравнить задержку
- [ ] Сравнить устойчивость к вибрации
- [ ] Определить предпочтительную архитектуру: OF, VIO или гибрид
- [ ] Зафиксировать итоговое решение
- [ ] **Этап 13 завершён**

## Контрольные точки

- [x] **CP1:** Kimera-VIO нативно собрана на текущем Raspberry Pi 5 — ARM64 build и запуск stereoVIOEuroc проверены, динамические зависимости разрешены
- [x] **CP2:** EuRoC V1_01_easy успешно проходит в Mono+IMU — полный pipeline frames 0..2912 завершён с exit 0, неожиданных пропусков кадров не выявлено, throughput 72.7 input fps, выходная траектория и её ошибки относительно GT измерены и сохранены
- [x] **CP3:** OV9281 + RAW IMU работают с общей временной шкалой — camera corrected V4L2 timestamps и affine-mapped `HIGHRES_IMU.time_usec` сведены в RPi `CLOCK_MONOTONIC`, offset/jitter/yaw validation и source continuity policy проверены; итоговый timestamp contract зафиксирован в `results/camera_imu_timestamp_scheme.md`
- [ ] **CP4:** Получена стабильная live VIO-одометрия на стенде
- [ ] **CP5:** Kimera-VIO сравнена с `jtzero-optical-flow-mvp` на одинаковых тестах