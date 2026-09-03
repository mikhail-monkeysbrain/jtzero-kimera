# FC IMU A/B tests

Изолированная рабочая область для аппаратных тестов P11/B11 с альтернативными полётными контроллерами и IMU.

## Ветка

`test/fc-imu-ab`

## Цель

Проверить, воспроизводится ли обнаруженный после yaw сдвиг stationary gyro на других FC/IMU без изменения production-кода Kimera.

Исходный baseline: основной FC на ArduPilot с ICM42688. Наблюдавшийся сдвиг после yaw: |Δgyro XY| около 0.00202 rad/s, преимущественно ΔX около +0.00192 rad/s.

## Test A — KARMAF435V1G / LSM6DSV16X

Аппаратная цепочка:
- MCU: AT32F435G
- Betaflight 4.5.4
- IMU: LSM6DSV16X
- gyro bus: SPI1
- sensor alignment: CW0FLIP
- USB на Raspberry Pi: `/dev/ttyACM0`
- USB device: Artery AT32 Virtual Com Port, VID:PID `2e3c:5740`
- MSP API 1.46 подтверждён с Raspberry Pi.

Протокол:
- MSP_RAW_IMU over USB ACM
- 15 s STILL -> ~90 deg YAW -> 15 s STILL
- native Betaflight MSP coordinates
- no FRD->FLU
- no ZXY correction
- PRE=750 samples, POST=750 samples

Результат 2026-09-03:
- Δgyro X = -0.000139626 rad/s
- Δgyro Y = +0.000072140 rad/s
- Δgyro Z = +0.000025598 rad/s
- |Δgyro XY| = 0.000157161 rad/s
- |Δgyro XYZ| = 0.000159233 rad/s
- baseline ICM42688 |Δgyro XY| ~= 0.002020000 rad/s
- ratio test/baseline = 0.078

Интерпретация: на независимой цепочке AT32F435G + Betaflight + LSM6DSV16X наблюдаемый PRE->POST gyro shift примерно в 12.9 раза меньше baseline. Это сильное свидетельство против гипотезы, что ~0.002 rad/s shift является неизбежным следствием самого yaw-манёвра или общей методики PRE/POST. Пока нельзя приписывать эффект непосредственно ICM42688, потому что одновременно изменились MCU, firmware и IMU.

## Test B — следующий контролируемый A/B

HK4530V2.1 / HAKRCF405V2:
- MCU: STM32F405
- Betaflight 4.4.3
- IMU: ICM42688P
- gyro bus: SPI1
- sensor alignment: CW90

Цель Test B: вернуть ICM42688-family при сохранении независимой платы/прошивки и повторить тот же физический протокол. Если shift снова порядка baseline, подозрение смещается к ICM42688-family. Если shift остаётся порядка Test A, подозрение смещается к baseline FC / ArduPilot / конкретному экземпляру / FC-side preprocessing.

## Правило

Все экспериментальные логгеры, анализаторы и результаты этого аппаратного A/B держим в этой папке. Production-код Kimera в рамках первичного теста не меняем.
