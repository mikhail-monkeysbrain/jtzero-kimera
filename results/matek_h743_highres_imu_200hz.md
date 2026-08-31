# Matek H743 HIGHRES_IMU — Stage 8

## Итог

Для live Kimera-VIO выбран поток MAVLink `HIGHRES_IMU` (message id 105) от Matek H743.

Рабочая конфигурация:

- UART Matek: UART3
- Raspberry Pi: `/dev/ttyAMA0`
- MAVLink: MAVLink2
- baud rate: 460800
- message: `HIGHRES_IMU` (105)
- target rate: 200 Hz
- acceleration: `xacc/yacc/zacc`, m/s^2
- angular velocity: `xgyro/ygyro/zgyro`, rad/s
- IMU timestamp: `time_usec`

## Исследование частоты

При 115200 baud поток упирался примерно в 159 Hz при запросах 200–300 Hz. После перехода UART3 на 460800 baud ограничение канала исчезло.

Результаты теста на 460800 baud:

| Requested | RX rate | IMU timestamp rate | Result |
|---:|---:|---:|---|
| 50 Hz | 49.90 Hz | 50.00 Hz | accepted |
| 100 Hz | 99.80 Hz | 100.00 Hz | accepted |
| 150 Hz | 166.35 Hz | 166.70 Hz | accepted, scheduler quantization |
| 200 Hz | 199.61 Hz | 200.00 Hz | accepted, selected operating point |
| 300 Hz | 332.71 Hz | 333.40 Hz | accepted, scheduler quantization and higher jitter |
| 400 Hz | 0 Hz | 0 Hz | denied |

`200 Hz` выбран как рабочая частота: она воспроизводится точно, имеет низкий jitter и достаточна для дальнейшей интеграции Kimera-VIO.

## 30-second validation

Сырой лог:

`/home/vio/highres_imu_200hz_30s.csv`

Результат контрольного прогона:

- capture time: 30.002 s
- samples: 5988
- RX rate: 199.591 Hz
- IMU timestamp rate: 200.001 Hz
- timestamp period min: 4.245 ms
- timestamp period mean: 5.000 ms
- timestamp period median: 5.002 ms
- timestamp period P95: 5.275 ms
- timestamp period P99: 5.374 ms
- timestamp period max: 5.769 ms
- absolute jitter mean: 0.125 ms
- absolute jitter P95: 0.325 ms
- absolute jitter P99: 0.581 ms
- absolute jitter max: 0.769 ms
- non-positive timestamp deltas: 0
- large timestamp gaps: 0
- estimated missing IMU samples: 0

Число samples не обязано быть ровно 6000 из-за границ окна приёма. Внутри реально записанной последовательности `time_usec` разрывов не обнаружено.

## Вывод

Stage 8 пройден. Matek H743 предоставляет все необходимые для Kimera-VIO данные gyro + accelerometer с собственным IMU timestamp и устойчивой частотой 200 Hz через UART 460800 baud. Следующий этап — привести `HIGHRES_IMU.time_usec` и timestamp OV9281/V4L2 к общей временной шкале и измерить camera-to-IMU offset.