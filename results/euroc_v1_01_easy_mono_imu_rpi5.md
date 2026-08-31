# EuRoC V1_01_easy — первый полный Mono+IMU тест на Raspberry Pi 5

## Конфигурация

- Платформа: Raspberry Pi 5 Model B Rev 1.0, ARM64
- ОС: Debian GNU/Linux 13 (trixie), 13.5
- Ядро: 6.18.34+rpt-rpi-2712
- Kimera-VIO: master, commit `ce8c59b7b273ab5ac29db7e5572e1623760e19c7`
- Режим: Mono + IMU, `params/EurocMono`, Regular VIO Backend
- Loop closure: выключен
- 3D visualizer: выключен
- Frontend image visualization: выключена
- Dataset: `/home/vio/datasets/euroc/V1_01_easy`
- Диапазон: frames `0..2912`

## Результат полного запуска

- Pipeline завершился штатно: `Pipeline successful? Yes!`
- Exit status: `0`
- FATAL / ERROR / segfault: отсутствуют
- `Spin took`: 40057 ms
- Wall time `/usr/bin/time -v`: 40.96 s
- User CPU time: 130.85 s
- System CPU time: 7.76 s
- CPU usage: 338%
- Max RSS: 1,076,064 KiB (~1.03 GiB)
- Swap: 0
- Input cam0: 2912 кадров за 145.550 s
- Средний offline throughput: `2912 / 40.057 = 72.7 input frames/s`
- Относительно EuRoC 20 FPS: примерно `3.64x real-time`

## Кадры, keyframes и очереди

- Вход cam0 содержит 2912 кадров.
- Последний VIO/frontend timestamp находится за 100 ms до последнего входного кадра, что соответствует штатному keyframe cadence.
- `min_intra_keyframe_time: 0.2`, поэтому ожидаемая частота keyframes/backend states — 5 Hz.
- `traj_vio.csv`: 728 VIO states; интервалы между состояниями ровно 0.200000 s, irregular gaps = 0.
- `output_frontend_stats.csv`: frontend keyframes также идут ровно через 0.200000 s, irregular gaps = 0.
- В Kimera `ThreadsafeQueue::push()` не удаляет кадры при росте очереди; очередь неограниченная.
- Для Mono dataset callback в текущем upstream `examples/KimeraVIO.cpp` используется обычный `Pipeline::fillLeftFrameQueue`, то есть кадры не отбрасываются из-за переполнения очереди.
- Проверка `euroc_mono_full.log`:
  - `Dropping frame|dropping this frame|Skipping first frame`: 0 совпадений при текущем verbosity
  - ошибки IMU sync (`Asking for data before start...`, `TooFewMeasurements`, `does not contain measurements`, `IMU buffer was shutdown`, `No IMU measurements available`): 0
  - `Data Provider waiting for IMU data newer than`: 0
  - `data_provider_left_frame_queue.*getting full`: 0 при текущем verbosity
- Вывод: неожиданных пропусков кадров из-за очереди или IMU-синхронизации в этом прогоне не обнаружено. Первый кадр может штатно использоваться как временная опорная точка согласно реализации DataProvider.

## Оценка позиции относительно ground truth

Сравнение выполнялось напрямую в общей метрической системе координат с линейной интерполяцией GT на timestamps VIO.

- GT valid samples: 28664
- VIO states: 728
- Matched VIO states: 717
- Skipped before GT: 5
- Skipped after GT: 6
- Compared duration: 143.200 s
- ATE RMSE: 0.921396 m
- Mean position error: 0.696045 m
- Median position error: 0.360336 m
- P95 position error: 1.717681 m
- Maximum position error: 1.766943 m
- Initial error: 0.003658 m
- Final common error: 1.714645 m
- VIO path length: 56.128 m
- GT path length: 58.379 m
- Path length error: -3.86%

Worst point:

- time from start: 124.800 s
- error: 1.766943 m
- VIO xyz: `-1.249258 0.973810 1.155523`
- GT xyz: `-0.008331 -0.274814 1.307602`

## Оценка ориентации и RPE

Absolute orientation error:

- Matched states: 717
- Initial angle error: 0.2249 deg
- Final angle error: 5.2351 deg
- Mean angle error: 3.3925 deg
- Maximum angle error: 5.2351 deg

RPE 1 second:

- Pairs: 712
- Translation RMSE: 0.085976 m
- Translation mean: 0.057060 m
- Translation max: 0.530247 m
- Rotation RMSE: 0.4457 deg
- Rotation mean: 0.3818 deg
- Rotation max: 1.1359 deg

RPE 10 seconds:

- Pairs: 667
- Translation RMSE: 0.466713 m
- Translation mean: 0.326508 m
- Translation max: 1.781587 m
- Rotation RMSE: 3.1551 deg
- Rotation mean: 2.8827 deg
- Rotation max: 5.6934 deg

## Вывод

Kimera-VIO нативно и устойчиво выполняет полный EuRoC V1_01_easy Mono+IMU pipeline на текущем Raspberry Pi 5, без падений и без выявленных неожиданных пропусков входных кадров. Производительность offline выше real-time для EuRoC 20 FPS.

Выходная траектория непрерывная и имеет регулярный 5 Hz keyframe/backend cadence, однако без loop closure накопленная ошибка заметна: ATE RMSE ~0.92 m, конечная ошибка ~1.71 m, а абсолютная ошибка ориентации возрастает до ~5.24 deg. Это фиксируется как фактическая точность текущей Mono+IMU конфигурации, а не как критерий отказа CP2.
