# JT-Zero Ground Motion MVP

Эта ветка предназначена для рабочего контура навигации, а не для дальнейшей подгонки настольного теста 500 мм.

## Зафиксированная архитектура

OV9281 + TF-Luna + attitude FC -> Ground Motion V3 -> метрические VX/VY + quality/valid -> MAVLink -> ArduPilot EKF.

## Что переносим из диагностики

- V3 ray-to-ground-plane геометрию.
- Синхронизацию attitude к timestamp кадра.
- CAD lever arm Luna->camera: body FLU примерно [+49.16,+0.22,0] мм. Z пока не выдумываем.
- Контроль качества optical flow / RANSAC.

## Что НЕ переносим как production-калибровку

- H20/H21 искусственные ветки.
- FIXED H 195/200/205 мм.
- Подгонку масштаба под известные 500 мм.
- Focal sweep ради совпадения настольного теста.

## Ограничение настольного теста

На высоте около 20 см один квант TF-Luna 10 мм даёт примерно 4.8% изменения метрического масштаба V3. Поэтому настольный тест больше не используется для подгонки процентов масштаба.

## Следующая реализация

1. Один production estimator без A/B веток.
2. Выход: timestamp, dx/dy, vx/vy, height, quality, valid.
3. Fail-closed: при плохом flow/height/attitude данные не отправлять в FC как валидные.
4. MAVLink publisher для ArduPilot.
5. Сначала проверка телеметрии на FC без замыкания управления, затем EKF integration.
