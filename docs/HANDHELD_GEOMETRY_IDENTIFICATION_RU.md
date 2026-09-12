# Handheld geometry identification

Этот режим предназначен для оценки монтажной геометрии OV9281 и TF-Luna без шарнира.

## Что пишется

- MJPG кадры OV9281;
- camera V4L2 timestamp и monotonic receive timestamp;
- TF-Luna range;
- FC ATTITUDE;
- FC HIGHRES_IMU, с fallback на RAW_IMU.

Запись не публикует OpticalFlow/ExternalNav и не меняет параметры FC.

## Зачем нужен ChArUco

При движении руками неизвестный перенос FC нельзя отделить от camera lever arm по OpticalFlow и attitude alone.
Неподвижная метрическая ChArUco-доска создаёт внешний 6DoF reference для камеры.
После этого camera pose + FC IMU dynamics позволяют оценивать lever arm камеры, а camera-plane geometry + TF-Luna
дают независимые ограничения на взаимное положение Luna и камеры.

## Запуск

```bash
bash tools/run_handheld_geometry_record.sh
```

Рекомендуется 60–70 секунд плавного движения с pitch/roll/yaw возбуждением и статикой в начале/конце.
