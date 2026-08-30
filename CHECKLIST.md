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
- [ ] Проверить пропуски кадров
- [ ] Проверить стабильность выходной траектории
- [ ] Сохранить результаты первого теста
- [ ] **Этап 5 завершён**

## Этап 6. Профилирование Raspberry Pi 5

- [ ] Измерить загрузку CPU по ядрам
- [ ] Измерить температуру CPU
- [ ] Проверить частоту CPU и throttling
- [ ] Измерить использование RAM
- [ ] Измерить использование swap
- [ ] Измерить реальную скорость обработки Kimera-VIO
- [ ] Определить, достигается ли real-time
- [ ] При необходимости подобрать облегчённые настройки frontend
- [ ] Зафиксировать рабочий профиль RPi5
- [ ] **Этап 6 завершён**

## Этап 7. Подключить OV9281

- [ ] Подключить OV9281 к RPi5
- [ ] Подтвердить стабильный захват grayscale
- [ ] Настроить 640x480
- [ ] Настроить начальные 30 FPS
- [ ] Зафиксировать короткую экспозицию
- [ ] Зафиксировать gain
- [ ] Отключить нежелательные автоматические изменения изображения
- [ ] Проверить стабильность timestamps кадров
- [ ] Сохранить тестовую последовательность кадров
- [ ] **Этап 7 завершён**

## Этап 8. Получить RAW IMU с Matek H743

- [ ] Определить MAVLink-сообщение / источник raw gyro и accelerometer
- [ ] Получить angular velocity X/Y/Z
- [ ] Получить linear acceleration X/Y/Z
- [ ] Получить timestamp для каждого IMU sample
- [ ] Добиться стабильной частоты не менее 200 Hz
- [ ] Проверить возможность работы в диапазоне 200–400 Hz
- [ ] Проверить пропуски и jitter потока IMU
- [ ] Сохранить сырой IMU-лог
- [ ] **Этап 8 завершён**

## Этап 9. Временная синхронизация камеры и IMU

- [ ] Привести camera и IMU timestamps к общей временной шкале
- [ ] Измерить постоянный camera-to-IMU time offset
- [ ] Измерить jitter камеры
- [ ] Измерить jitter IMU / MAVLink / serial
- [ ] Реализовать компенсацию постоянного offset при необходимости
- [ ] Проверить синхронизацию на yaw-движении
- [ ] Исключить крупные скачки межкадрового угла из-за timestamp mismatch
- [ ] Зафиксировать итоговую схему timestamping
- [ ] **Этап 9 завершён**

## Этап 10. Калибровка камеры

- [ ] Зафиксировать финальную оптику OV9281
- [ ] Зафиксировать рабочее разрешение
- [ ] Выполнить калибровку intrinsic parameters
- [ ] Получить fx и fy
- [ ] Получить cx и cy
- [ ] Получить distortion coefficients
- [ ] Проверить reprojection error
- [ ] Сохранить calibration-файл в репозитории
- [ ] **Этап 10 завершён**

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
- [ ] **CP2:** EuRoC V1_01_easy успешно проходит в Mono+IMU
- [ ] **CP3:** OV9281 + RAW IMU работают с общей временной шкалой
- [ ] **CP4:** Получена стабильная live VIO-одометрия на стенде
- [ ] **CP5:** Kimera-VIO сравнена с `jtzero-optical-flow-mvp` на одинаковых тестах
