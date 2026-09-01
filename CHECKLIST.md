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
- [x] Проверить возможность работы в диапазоне 200–400 Hz — 200 Hz воспроизводится точно; запрос 300 Hz квантуется примерно в 333.4 Hz с большим jitter; 400 Hz отклоняется `MAV_RESULT_DENIED`; рабочая частота зафиксирована на 200 Hz
- [x] Проверить jitter IMU timestamps — 30 s validation: timestamp dt median=5.000 ms, p95=5.003 ms, p99=5.013 ms, min=4.928 ms, max=5.079 ms; receive dt p99=8.692 ms относится к UART transport, а не к sensor timestamp jitter
- [x] Проверить пропуски samples — 30 s validation: 5988 samples за 29.995 s, `timestamp gaps > 7.5 ms = 0`; пропусков sensor-timestamp samples не выявлено
- [x] Сохранить RAW IMU log — `/home/vio/matek_highres_imu_200hz.csv`; результаты и команды зафиксированы в `results/matek_highres_imu.md`
- [x] **Этап 8 завершён**

## Этап 9. Синхронизация Camera + IMU

- [x] Выбрать общую временную шкалу — Raspberry Pi `CLOCK_MONOTONIC`; camera V4L2 timestamp уже в monotonic domain, `HIGHRES_IMU.time_usec` переводится в него через MAVLink TIMESYNC affine mapping
- [x] Определить источник timestamp камеры — native V4L2 `v4l2_buffer.timestamp`; flags `MONOTONIC + SOE`; raw timestamp сохраняется без перезаписи receive-time
- [x] Определить источник timestamp IMU — `HIGHRES_IMU.time_usec`; UART receive timestamp используется только как диагностический transport timestamp и не подменяет sensor time
- [x] Реализовать преобразование `HIGHRES_IMU.time_usec -> RPi CLOCK_MONOTONIC` — непрерывно оцениваемый affine mapping `t_rpi = rpi_ref + A*(t_fc-fc_ref)` по MAVLink TIMESYNC; measured clock mismatch ~2000 ppm, поэтому `A` не должен быть захардкожен; mapping имеет явное состояние valid/stale и при invalid/stale синхронизация считается недоступной без fallback на UART receive time
- [x] Измерить camera↔IMU offset — sharp yaw, camera raw timestamp против mapped gyro: 15/15 независимых сегментов дали median lag `-10.15 ms`, MAD `0.8 ms`; global lag `-10.55 ms`, corr `-0.996`; sign-verifier подтвердил компенсацию сдвигом camera timestamp вперёд на `+10.5 ms` (residual `-0.05 ms`), противоположный знак дал `-21.05 ms`
- [x] Измерить jitter — clean `/dev/shm` 120 s run: TIMESYNC RTT median `2.379 ms`, p95 `2.917 ms`; camera delivery median `8.029 ms`, p95 `8.063 ms`, p99 `8.084 ms`, max `12.304 ms`; IMU transport median `0.718 ms`, p95 `1.206 ms`, p99 `1.276 ms`, max `4.501 ms`; simultaneous ~1.6 s camera/UART stalls наблюдались только при синхронной записи большого лога на диск и исчезли при `/dev/shm`
- [x] Проверить стабильность offset во времени — 5 min TIMESYNC drift `2066.836 ppm`; отдельные yaw-сегменты стабильны (15/15, MAD `0.8 ms`); compensated validation дала global residual `-0.150 ms`, corr `-0.964`
- [x] Проверить поведение при пропуске кадров — continuity проверяется на каждом RAW 120 FPS V4L2 frame до temporal selection: `sequence == prev+1`, corrected timestamp строго возрастает, `dt <= 20 ms`; при нарушении pair отвергается / tracker должен reset; clean test: 3620 frames, 1 реальный source sequence gap, missing=1, rejected pair=1, при этом dt был всего `7.998 ms`, что подтверждает необходимость sequence-check; выбранный ~30 FPS поток сохраняет реальные source timestamps и намеренно имеет raw sequence gaps, поэтому raw seq+1 policy к нему не применяется
- [x] Проверить yaw-движение и знак временной компенсации — camera `+10.5 ms` residual `-0.05 ms`; противоположный знак `-10.5 ms` residual `-21.05 ms`; global compensated validation residual `-0.150 ms`, corr `-0.964`
- [x] Зафиксировать итоговую схему timestamps — `results/camera_imu_timestamp_scheme.md`; для текущей конфигурации OV9281 USB UVC 640x480 MJPEG @120 FPS применяется camera correction `+10.5 ms`; компенсация должна быть перекалибрована после смены interface/mode/USB bridge/driver/timestamp semantics, а для CSI — обязательно; IMU подаётся в Kimera только при valid/stale-safe TIMESYNC affine mapping
- [x] **Этап 9 завершён**

## Этап 10. Калибровка камеры

- [x] Подготовить calibration target — ChArUco 7x5, `DICT_4X4_50`; фактически измеренный `squareLength=27.324 mm`, `markerLength≈20.043 mm`; использован финальный фокус/линза текущей камеры
- [x] Собрать набор кадров с разными углами и положениями — финальный calibration set: 63 принятых кадра, покрытие поля и наклоны проверены
- [x] Получить intrinsics — `fx=568.53170752165227`, `fy=569.68005562865858`, `cx=315.98271077441063`, `cy=239.88148589100641`
- [x] Получить distortion coefficients — radtan_5: k1=0.073569192194028493, k2=-0.095253893789117, p1=-0.010810530757187299, p2=-0.0022843373576970235, k3=0.082177400802757483
- [x] Проверить reprojection error — calibration RMS=0.3473217318 px; independent validation на 49 новых кадрах: aggregate view RMSE=0.407339 px, median=0.379265 px, max=0.686381 px, 0 views >0.75 px, RESULT=PASS; визуальный undistortion check также PASS
- [x] Сохранить calibration-файл в репозитории — `calibration/ov9281_intrinsics.yaml`; итоговый отчёт `results/ov9281_intrinsics_validation.md`
- [x] **Этап 10 завершён**

## Этап 11. Калибровка Camera ↔ IMU

- [x] Зафиксировать системы координат камеры и IMU — IMU/body frame экспериментально подтверждён как FRD (`+X` forward, `+Y` right, `+Z` down) по отдельным roll/pitch/yaw тестам и проверке знаков; camera frame зафиксирован как OpenCV (`+X` right, `+Y` down, `+Z` optical forward); результаты сохранены в `results/camera_imu_extrinsics.md`
- [x] Определить Camera↔IMU rotation — после перестройки стенда выполнены две независимые синхронизированные ChArUco + HIGHRES_IMU калибровки; geodesic disagreement `0.465 deg`; принята post-rebuild матрица `R_BC` из `results/camera_imu_extrinsics.md`
- [x] Определить Camera↔IMU translation — для текущей жёсткой сборки прямым механическим измерением зафиксировано `t_BC=[0.000, 0.000, 0.055] m`, консервативная неопределённость `X ±5 mm, Y ±5 mm, Z ±3 mm`; ручной visual pivot-test не используется как оценка lever arm из-за сопоставимой ошибки repositioning
- [x] Проверить направления и знаки всех осей — `tools/camera_imu_extrinsics_validator.cpp` подтвердил `R_BC*C_X≈-B_Y`, `R_BC*C_Y≈+B_X`, `R_BC*C_Z≈+B_Z`, `R_CB=R_BC^T`, `det(R)=+1`; `STATIC RESULT: PASS`; отчёт `results/camera_imu_extrinsics_static_validation.md`
- [ ] Проверить extrinsics экспериментально — rotation подтверждён независимо; первый live Mono+IMU smoke test с полным `T_BS` успешно прошёл без катастрофической ошибки, но количественный motion-test полного transform остаётся открытым
- [x] Сохранить extrinsic calibration в репозитории — финальный для текущей механической сборки файл `calibration/ov9281_extrinsics.yaml`; в нём явно зафиксировано, что translation получена механическим измерением, а не независимой visual/IMU оценкой
- [ ] **Этап 11 завершён**

## Этап 12. Live Mono + IMU

- [x] Подать live OV9281 grayscale в Kimera-VIO — первый native smoke run: 3618 raw frames, 847 выбранных/декодированных кадров, backend работал до kf=123; `results/live_mono_imu_smoke_20260901.md`
- [x] Подать live RAW IMU в Kimera-VIO — 6231 `HIGHRES_IMU` получено, 5606 samples подано в Kimera после валидизации mapping; receive-time fallback не используется
- [x] Использовать синхронизированные timestamps — camera corrected V4L2 timestamps + TIMESYNC affine-mapped `HIGHRES_IMU.time_usec`; 297 TIMESYNC samples, mapping valid, measured drift `2034.997 ppm`
- [x] Получить устойчивый live pose — 30 s smoke test PASS и отдельный ~118 s stand-still backend run PASS; в покое final |dP| `16.29 mm`, RMS `5.56 mm`, max `18.39 mm`
- [x] Получить velocity — backend выдаёт live `V=[vx,vy,vz]`; stand-still RMS скорости `1.50 mm/s`, max `10.63 mm/s`
- [ ] Реализовать логирование входных данных и результата
- [x] Провести тест в покое — 120 s requested / 117.90 s backend duration; final |dP| `16.29 mm`, position RMS `5.56 mm`, max `18.39 mm`, yaw drift `-1.043 deg`; `results/live_mono_imu_standstill_20260901.md`
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
- [ ] **CP4:** Получена стабильная live VIO-одометрия на стенде — 30 s live smoke PASS и 120 s stand-still PASS; CP4 будет закрыта после известного линейного перемещения и проверки масштаба
- [ ] **CP5:** Kimera-VIO сравнена с `jtzero-optical-flow-mvp` на одинаковых тестах
