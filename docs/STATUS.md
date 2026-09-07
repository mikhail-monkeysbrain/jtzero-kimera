# JT-Zero P11 / V21-V23 — текущий статус

Назначение: компактная operational memory. Полная хронология остаётся в `docs/ASSISTANT_ACTION_LOG.md`.

Последнее обновление: 2026-09-07.

## Главный вопрос

Почему при стендовом горизонтальном движении возникает direction-dependent VIO/PIM Z/attitude asymmetry, хотя TF-Luna показывает практически неизменную физическую высоту?

## Что подтверждено

- Ложный вертикальный velocity появляется уже на PIM/inertial prediction до основной backend correction.
- Backend accelerometer bias не является единственным источником directional Z.
- В FC HIGHRES_IMU наблюдается A/B-зависимый stationary acceleration signal.
- Два независимых raw position runs дали norm split:
  - 151131: B-A = -0.039198 m/s²;
  - 160342: B-A = -0.036468 m/s².
- Эта **межпрогонная воспроизводимость пока предварительная: n=2**.
- В отдельном paired vector run 162057 три A→B пары дали acceleration-vector direction change примерно 2.2–2.4°; основная часть vector delta перпендикулярна gravity vector (~0.39–0.40 m/s²), при гораздо меньшем norm drop.
- External world-fixed phone video single-run projective test не поддержал реальный body tilt порядка 2.3°: raw 2-D edge change ~1.33° объясняется перспективой; остаточная projective deviation в B порядка нескольких десятых градуса. **Это предварительный discriminator n=1, не окончательное исключение механики.**

## Важная количественная сверка V21/V23 ↔ P11

Не смешивать две разные величины.

1. Stationary **norm split** ~0.037–0.039 m/s²:
   - относительная величина ~0.0038 g;
   - грубый g-equivalent angle ~0.22°.
   - Сам по себе этот norm split не объясняет VIO attitude asymmetry ~1.6–1.7°.

2. Stationary **vector-direction split** из P11 paired vector analysis:
   - angle(A,B) ~2.2–2.4°;
   - perpendicular delta ~0.39–0.40 m/s².
   - Это тот же порядок величины, что V21/V23 attitude difference ~1.6–1.7°.

Вывод: утверждение «IMU effect в 8 раз меньше VIO effect» верно только если сравнивать VIO angle с norm split, что некорректно. По vector-direction magnitude явления сопоставимы. Но причинная передача IMU→PIM→VIO **ещё не доказана**: нужен явный gain/transfer reconciliation по времени и осям.

## Что НЕ установлено

- Почему физическая позиция A/B меняет FC acceleration vector/norm.
- Является ли это MEMS/FC processing, механическим напряжением, кабелями, локальной вибрацией/нагрузкой или другим фактором.
- Объясняет ли P11 IMU vector split весь V21/V23 VIO attitude/position asymmetry или только часть.
- Есть ли filter amplification / weak observability mechanism, превращающий IMU error в VIO asymmetry.
- Переносится ли bench A/B effect на свободный полёт.

## Заблокированные / закрытые ветки

- Pure monocular homography → physical tilt: закрыто; translation/plane terms неразделимы надёжно.
- Onboard stereo plane-normal route: не доведён до причинного результата из-за rectification/matching instability и физического ограничения стенда.
- Fixed-target stereo stability test: физически невыполним на текущем стенде без удержания БПЛА руками.
- ChArUco pose из P11 stereo run: невалидно для этого dataset; target не гарантированно виден обеим камерам.

## Предварительные результаты, требующие независимого повтора

- raw A/B norm split: n=2 independent runs → нужен минимум ещё 1–2 идентичных raw position run.
- external world-fixed projective tilt discriminator: n=1 → нужен второй независимый внешний video pass перед сильным исключением mechanical tilt.
- ruler-pass 181857 и stereo run 185426: single-run geometry evidence; не использовать как окончательный causal proof.

## Следующие наиболее ценные действия

1. **Проверить rigid rotation по raw gyro.** Raw stationary gravity-vector tilt уже совпал с FC tilt на всех 6 legs: A->B RAW median 2.3329° vs FC 2.4188°; B->A RAW 2.3103° vs FC 2.2162°. Следующий discriminator — `tools/analyze_v23_gyro_vs_gravity_rotation.py`: интегрировать raw HIGHRES_IMU gyro между endpoint plateaus и проверить, предсказывает ли он тот же ~2.3° gravity-vector rotation. Это отделит реальное вращение IMU/body от accel bias/misalignment-like vector shift.

2. **Повторить дешёвый raw A/B position test** ещё минимум 1–2 раза без изменения протокола, чтобы поднять независимую выборку выше n=2.

3. **Повторить external world-fixed video pass** с той же жёсткой кромкой и неподвижным телефоном, если механический tilt используется как discriminator.

## Обязательный preflight перед новым кодом

- Прочитать этот STATUS и релевантный участок ACTION_LOG.
- Проверить, не повторяется ли действие.
- Для geometry/calibration code сначала открыть фактический calibration/config и проверить sensor order, stereo axis/type, frames, units, K/D/R/T/P.
- Перед причинным выводом выполнить order-of-magnitude reconciliation.
- n<3 independent physical runs = предварительный результат.
