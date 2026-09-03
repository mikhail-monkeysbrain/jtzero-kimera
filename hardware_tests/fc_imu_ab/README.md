# FC IMU A/B tests

Изолированная рабочая область для аппаратных тестов P11/B11 с альтернативными полётными контроллерами и IMU.

## Ветка

`test/fc-imu-ab`

## Цель

Проверить, воспроизводится ли обнаруженный после yaw сдвиг stationary gyro на других FC/IMU без изменения production-кода Kimera.

Исходный baseline: основной FC на ArduPilot с ICM42688. Наблюдавшийся сдвиг после yaw: |Δgyro XY| около 0.00202 rad/s, преимущественно ΔX около +0.00192 rad/s.

## Первый тестовый FC

KARMAF435V1G:
- MCU: AT32F435G
- Betaflight 4.5.4
- IMU: LSM6DSV16X
- gyro bus: SPI1
- sensor alignment: CW0FLIP
- USB на Raspberry Pi: `/dev/ttyACM0`
- USB device: Artery AT32 Virtual Com Port, VID:PID `2e3c:5740`
- MSP API 1.46 подтверждён с Raspberry Pi.

## Правило

Все экспериментальные логгеры, анализаторы и результаты этого аппаратного A/B держим в этой папке. Production-код Kimera в рамках первичного теста не меняем.
