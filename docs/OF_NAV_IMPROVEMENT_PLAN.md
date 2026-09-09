# JT-Zero — Optical-Flow Navigation Improvement Plan

Цель: OV9281 + TF-Luna + FC gyro/attitude -> воспроизводимое metric Vx,Vy.

## Старый план
- 1/8–4/8: завершены.
- 5/8: ЗАМОРОЖЕН. Новые физические проходы запрещены до готовности deterministic recorder/replay.

## R1 — deterministic recorder/replay
- R1 1/6 — ТЕКУЩИЙ: зафиксировать контракт данных и provenance (Git/Kimera/binary/params), аудитировать реальные timestamps и явные пробелы.
- R1 2/6 — асинхронная запись raw OV9281 frames; frame_id + capture timestamp; bounded queue; drop accounting.
- R1 3/6 — timestamped synchronized TF-Luna sample, исправление race/stale/join; точная привязка IMU/ATT.
- R1 4/6 — deterministic replay на записанном dataset; одинаковый результат при повторных запусках.
- R1 5/6 — ровно три записи: STATIC, MOVE500-dev, MOVE500-control.
- R1 6/6 — A/B алгоритмов на одном входе и локализация остаточной ошибки.

## Запреты до R1 4/6
- не делать новые 500-мм проходы;
- не вводить empirical scale coefficient;
- не делать причинных выводов из разных физических прогонов;
- не восстанавливать timestamps равномерным распределением между START/END.

## Критерий готовности R1
Replay одного dataset должен быть детерминирован, а каждое visual-motion update должно ссылаться на реальные frame0/frame1 и их timestamps.
