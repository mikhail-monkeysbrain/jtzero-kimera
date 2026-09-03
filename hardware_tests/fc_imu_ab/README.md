# FC IMU A/B tests

Изолированная рабочая область для аппаратных тестов P11/B11 с альтернативными полётными контроллерами и IMU.

## Ветка

`test/fc-imu-ab`

## Цель

Проверить, воспроизводится ли обнаруженный после yaw сдвиг stationary gyro на других FC/IMU без изменения production-кода Kimera.

Исходный baseline: основной FC на ArduPilot с ICM42688. Наблюдавшийся сдвиг после yaw: |Δgyro XY| около 0.00202 rad/s, преимущественно ΔX около +0.00192 rad/s.

## Важное уточнение по MSP_RAW_IMU

В Betaflight 4.4/4.5 `MSP_RAW_IMU` сериализует значение `gyroRateDps(i)`, а `gyroRateDps()` вычисляется как `gyro.gyroADCf / rawSensorDev->scale`. Поэтому возвращаемое gyro-поле фактически находится в sensor-count-equivalent units, и для перевода в deg/s требуется масштаб конкретного драйвера IMU.

Используемые масштабы:
- LSM6DSV16X: 0.070000 deg/s/count
- ICM42688P: 2000/32768 = 0.06103515625 deg/s/count

Первая версия логгера ошибочно использовала универсальные 0.1 deg/s/count. Сырые CSV не повреждены, поэтому Test A пересчитан без повторного физического прогона.

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

Исходный raw-count результат сохранён в CSV. После правильного масштаба LSM6DSV16X 0.070 deg/s/count:
- Δgyro X ≈ -0.000097738 rad/s
- Δgyro Y ≈ +0.000050498 rad/s
- Δgyro Z ≈ +0.000017919 rad/s
- |Δgyro XY| ≈ 0.000110013 rad/s
- |Δgyro XYZ| ≈ 0.000111463 rad/s
- baseline ICM42688 |Δgyro XY| ~= 0.002020000 rad/s
- ratio test/baseline ≈ 0.0545

Интерпретация: на независимой цепочке AT32F435G + Betaflight + LSM6DSV16X PRE->POST gyro shift примерно в 18.4 раза меньше baseline. Это сильное свидетельство против гипотезы, что ~0.002 rad/s shift является неизбежным следствием самого yaw-манёвра или общей методики PRE/POST. Пока нельзя приписывать эффект непосредственно ICM42688, потому что одновременно изменились MCU, firmware и IMU.

## Test B — следующий контролируемый A/B

HK4530V2.1 / HAKRCF405V2:
- MCU: STM32F405
- Betaflight 4.4.3
- IMU: ICM42688P
- gyro bus: SPI1
- sensor alignment: CW90
- USB на Raspberry Pi: `/dev/ttyACM0`
- USB: STMicroelectronics Virtual COM Port, VID:PID `0483:5740`
- MSP API 1.45 подтверждён с Raspberry Pi.

Для Test B использовать профиль:

`python3 hardware_tests/fc_imu_ab/betaflight_imu_bias_test.py --profile hk4530-icm42688p`

Цель Test B: вернуть ICM42688-family при сохранении независимой платы/прошивки и повторить тот же физический протокол. Если shift снова порядка baseline, подозрение смещается к ICM42688-family. Если shift остаётся порядка Test A, подозрение смещается к baseline FC / ArduPilot / конкретному экземпляру / FC-side preprocessing.

## Правило

Все экспериментальные логгеры, анализаторы и результаты этого аппаратного A/B держим в этой папке. Production-код Kimera в рамках первичного теста не меняем.
