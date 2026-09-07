# JT-Zero: дерево расследования ошибки VIO на тесте 500 мм

> Этот файл — живой журнал расследования. После каждого значимого теста фиксируются результат, вывод и новые подгипотезы. Неподтверждённые причины не объявляются установленными.

## 1. Формулировка проблемы

Цель текущего этапа — получить на стенде воспроизводимое измерение поступательного перемещения БПЛА на 500 мм с корректным возвратом B→A.

Наблюдаемая проблема: Kimera-VIO может достаточно правдоподобно измерить первый участок A→B, но в отдельных тестах даёт ошибку масштаба, паразитное вертикальное/боковое перемещение, изменение roll/pitch и особенно сильный развал на обратном медленном участке B→A.

Характерный контрольный прогон V25:
- A→B: 448.85 мм при истинных 500 мм, ошибка -51.15 мм;
- dz: +57.66 мм;
- при B→A состояние вместо возврата начало уходить далеко от A;
- measured run был дополнительно инвалидирован loop stall 735.130 ms;
- stationary warmup при этом был стабильным на миллиметровом уровне.

Следовательно, задача состоит не в подгонке одного коэффициента масштаба, а в поиске механизма, который превращает реальное поступательное движение в ошибочные ускорение, ориентацию и положение VIO.

## 2. Правила расследования

1. За один причинный тест меняем одну существенную вещь.
2. До теста задаём критерий PASS/FAIL.
3. Успех и неуспех одинаково фиксируются.
4. Если тест открывает несколько возможных причин, они становятся дочерними подгипотезами, а не смешиваются.
5. Не возвращаем отвергнутую гипотезу без новых данных.
6. Отдельно различаем:
   - первопричину ошибки траектории;
   - вторичные эффекты;
   - инфраструктурные сбои (USB, UART, loop stall).
7. Калибровочные параметры не подбираются для улучшения результата без физического/экспериментального основания.

## 3. Текущее дерево гипотез

### Гипотеза 1 — неверное разделение начального наклона и accelerometer bias

**Статус: АКТИВНАЯ, ведущая.**

Суть: на stationary startup горизонтальная составляющая акселерометра может интерпретироваться Kimera как bias акселерометра при принудительном начальном RPY≈0, вместо того чтобы сначала согласовать roll/pitch с направлением gravity.

#### Тест 1.1 — forensic stationary accel/bias

Результат:
- RAW mean FLU accel: примерно [-0.261, +0.184, +9.800] м/с²;
- gravity-derived tilt: roll ≈ +1.074°, pitch ≈ +1.527° в используемой accel-конвенции;
- late backend BA ≈ [-0.326, +0.234, -0.007] м/с²;
- bias, необходимый для согласования RAW с backend attitude, ≈ [-0.281, +0.178, -0.010] м/с²;
- расхождение с backend BA ≈ 0.072 м/с².

**Вывод:** большой BA связан с несогласованностью stationary acceleration и принятой backend attitude; это не выглядит как чисто случайный поздний drift.

Тест породил подгипотезы:
- 1A: большой BA создаётся уже в InitializationFromImu;
- 1B: большой BA возникает позже из-за coupling attitude/bias в backend.

#### Подгипотеза 1A — большой BA создаётся непосредственно при IMU initialization

##### Тест 1A.1 — JT-IMU-INIT

В Kimera-VIO добавлена opt-in диагностика `JTZERO_DIAG_IMU_INIT`, не меняющая математику initialization.

Наблюдение:
- `initRPYdeg=[0 -0 0]`;
- `initBA≈[-0.344,+0.234,+0.008]` м/с².

**Результат: PASS — подгипотеза подтверждена.**

**Вывод:** значительная часть большого accelerometer bias существует уже в начальном состоянии Kimera. Он не является исключительно поздним результатом оптимизации.

##### Тест 1A.2 — gravity-aligned initialization

**Статус: ПАТЧ ГОТОВ, ожидается stationary A/B инициализации.**

Новый факт из исходного кода Kimera: `UtilsOpenCV::AlignGravityVectors()` считает векторы уже совмещёнными при `|1-dot| < 1e-3`. Это соответствует угловому deadband примерно 2.56°. В нашем диагностическом startup реальный наклон gravity был около 2.4°, поэтому функция возвращала identity rotation. После этого почти вся горизонтальная компонента mean acceleration попадала в `initBA`.

Это напрямую объясняет наблюдение:
- `meanAcc≈[-0.344,+0.234,+9.818]`;
- `initRPY=[0,0,0]`;
- `initBA≈[-0.344,+0.234,+0.008]`.

В Kimera-VIO добавлен отдельный opt-in режим `JTZERO_GRAVITY_ALIGNED_IMU_INIT=1`, который использует точное gravity alignment с существенно меньшим epsilon и не меняет стандартное поведение Kimera без переменной окружения.

План теста: на подтверждённом stationary startup включить этот режим и проверить initial roll/pitch и bias.

Не менять одновременно:
- camera T_BS;
- ZXY;
- staged ZUPT;
- gravity feedback policy;
- IMU noise parameters.

Критерии:
- PASS-A: `|initBA.xy|` существенно уменьшается относительно текущих ~0.4 м/с²;
- PASS-B: stationary warmup остаётся стабильным;
- PASS-C: 500 мм A→B/B→A показывает существенное уменьшение паразитных dz/наклона/closure error;
- FAIL: BA остаётся большим либо траектория практически не улучшается.

Если PASS-A, но FAIL-C → перейти к 1B и другим гипотезам: initialization действительно неверна, но не является достаточной причиной ошибки траектории.

#### Подгипотеза 1B — backend повторно создаёт неправильную связку attitude/bias

**Статус: ОТКРЫТА, проверять после 1A.2 при необходимости.**

Основание: PREPOST показывал, что optimizer способен существенно менять R/P и velocity при почти неизменном большом BA.

Тест 1B.1:
- сравнить init state, PRE/POST и эволюцию BA после gravity-aligned init;
- определить, возвращается ли BA к большим значениям после правильного seed.

---

### Гипотеза 2 — ошибка camera↔body extrinsics (T_BS)

**Статус: ОТКРЫТА, но не первая в очереди.**

Суть: неточная ориентация камеры относительно FLU body может смешивать реальную продольную трансляцию с вертикальной/боковой компонентой и ориентацией.

Что уже было сделано:
- исследовались корректировки T_BS;
- активный диагностический профиль содержит кандидат с коррекцией Rx(-1.5°), Ry(-5.5°);
- один только подбор T_BS не дал устойчивого окончательного решения всей A→B→A задачи.

Тест 2.1:
- проводить только после развязки Hypothesis 1;
- использовать физически обоснованный/калиброванный T_BS;
- сравнить симметрию A→B и B→A, dz и изменение R/P.

Критерий: улучшение должно повторяться на нескольких проходах и направлениях, а не только на одном A→B.

---

### Гипотеза 3 — LOW_DISPARITY/ZUPT и логика stationary/motion искажают состояние

**Статус: ЧАСТИЧНО ПРОВЕРЕНА.**

Суть: mono frontend при малой disparity может объявлять отсутствие движения; стандартная backend-логика добавляет zero-velocity/no-motion factors. На медленном B→A это потенциально способно подавлять реальное движение.

Что сделано:
- введён staged ZUPT;
- первые candidate streak не получают сразу полный no-motion constraint;
- проведены тесты с различной скоростью, в том числе медленным AB-BA и средним AB.

Наблюдение: staged ZUPT улучшает методологию обработки stationary transition, но не объясняет обнаруженный большой BA уже на initialization.

**Вывод:** это может быть дополнительной причиной ошибки медленного участка, но не объясняет Hypothesis 1A.

Тест 3.1:
- после исправления initialization повторить одинаковую траекторию на двух контролируемых скоростях;
- сопоставить frontend status/inliers/disparity с моментом возникновения ошибки.

---

### Гипотеза 4 — ошибка преобразования IMU frame / gyro correction

**Статус: СИЛЬНО ОСЛАБЛЕНА, не закрыта полностью.**

Проверено:
- используется FRD→FLU: accel=(x,-y,-z), gyro=(x,-y,-z);
- исследовалась Z→XY gyro correction;
- gravity feedback отключался во время deliberate translation;
- yaw-тесты использовались для диагностики паразитных roll/pitch.

Текущий stationary init defect возникает до того, как его можно объяснить только ZXY/gravity feedback.

Следующие тесты по этой ветке делать только при новых противоречащих данных после Hypothesis 1.

---

### Гипотеза 5 — временная несинхронность/задержки pipeline

**Статус: ОТДЕЛЬНАЯ инфраструктурная ветка.**

Известно:
- были диагностированы timestamps USB+CSI;
- V25 имеет контроль state age и loop stalls;
- в одном measured leg зарегистрирован stall 735.130 ms, из-за чего run корректно помечен INVALID;
- существовали и прогоны без >100 ms stalls.

**Вывод:** stall способен испортить конкретный прогон, но не объясняет воспроизводимый stationary initialization BA.

Тест 5.1:
- каждый причинный прогон принимается для анализа траектории только при отсутствии invalidating stalls;
- stalled run не используется как доказательство PASS/FAIL других гипотез.

---

### Гипотеза 6 — проблемы visual frontend / недостаточная геометрическая информация mono

**Статус: ОТКРЫТА.**

Суть: качество features/inliers само по себе не гарантирует хорошую наблюдаемость масштаба и движения, особенно при малой disparity и медленном перемещении.

Проверять после Hypothesis 1 и отдельно от ZUPT:
- detected/tracked features;
- mono inliers/putatives;
- status;
- скорость движения;
- момент начала divergence.

Критерий: если после исправления IMU initialization ошибка остаётся и коррелирует с frontend geometry/status, ветка повышает приоритет.

## 4. Внешние проблемы, которые не следует смешивать с VIO-причиной

### USB OV9281
Камера периодически исчезала из V4L2 после USB disconnect/reset. Был сделан универсальный выбор capture node по устройству/by-id. Физический disconnect остаётся аппаратным событием и делает прогон невалидным.

### FC UART
Для FC подтверждён MAVLink heartbeat на `/dev/ttyAMA0 @ 460800`. Проверен UART loopback. Это отдельная транспортная проблема и не является текущим объяснением VIO drift.

## 5. Очередь действий

Текущий порядок:

1. **Hypothesis 1A / Test 1A.2:** gravity-aligned IMU initialization.
2. Если initBA уменьшается — статический контроль, затем один A→B→A.
3. Если BA снова растёт после initialization — Hypothesis 1B.
4. Если IMU state становится корректным, но translation остаётся неправильной — Hypothesis 2.
5. Затем контролируемая проверка speed/LOW_DISPARITY — Hypothesis 3.
6. Visual observability/frontend — Hypothesis 6.
7. Hypothesis 4 возвращается только при новых данных.
8. Любой run с pipeline invalidation сначала разбирается по Hypothesis 5 и не используется для причинных выводов.

## 6. Критерий завершения расследования

Проблема считается решённой не по одному удачному A→B. Требуется:
- корректный stationary startup без необъяснимого большого BA;
- воспроизводимый A→B около 500 мм;
- воспроизводимый B→A с возвратом к исходной точке;
- отсутствие систематического большого dz;
- отсутствие систематического изменения roll/pitch от чистой трансляции;
- несколько повторов с сопоставимым результатом;
- валидный pipeline без USB disconnect и invalidating loop stalls.

## 7. Журнал обновлений

### 2026-09-07 — создано дерево расследования
Зафиксирована текущая причинная структура после обнаружения `JT-IMU-INIT`. Ведущая ветка — Hypothesis 1A. Следующий причинный эксперимент — gravity-aligned initialization.

### 2026-09-07 — локализован механизм нулевого initial tilt
Проверка исходника `UtilsOpenCV::AlignGravityVectors()` показала deadband `|1-dot| < 1e-3`, который эквивалентен примерно 2.56°. Наблюдаемый stationary tilt JT-Zero (~2.4° по полному вектору gravity) попадал внутрь этого deadband, поэтому initialization возвращала identity rotation и записывала горизонтальную gravity-компоненту в accelerometer bias. В Kimera-VIO подготовлен opt-in exact-gravity patch. Следующий шаг — stationary проверка `initRPY/initBA`.


### 2026-09-07 — Test 1A.2 PASS: exact gravity initialization fixes startup seed

Stationary test with `JTZERO_GRAVITY_ALIGNED_IMU_INIT=1` confirmed the causal prediction.

Observed:
- `initMode=jtzero_exact_gravity`;
- `meanAcc=[-0.277451,+0.188860,+9.79085]`;
- `initRPYdeg=[1.10507,1.62290,0.015652]` instead of identity;
- `initBA=[+0.000379,-0.000258,-0.013391]`, so the former large horizontal startup bias is essentially removed;
- first VIO state preserves the gravity-derived attitude;
- mandatory 12 s stationary warmup PASS with drift from first `[5.669,-3.149,+2.630] mm` and stable=8.

Conclusion: Hypothesis 1A is causally confirmed. Kimera's default gravity-alignment deadband was a real initialization defect for the JT-Zero startup geometry.

Important: this does NOT yet prove the 500 mm measurement problem is solved. No A→B→A motion was executed in this run; the harness therefore ends with `PIPELINE RESULT: FAIL / MEASUREMENT RESULT: FAIL` because the requested closure sequence was incomplete, not because the stationary initialization test failed.

New observation for the next branch: during stationary backend optimization, accelerometer bias continues to move after initialization (especially Z, and smaller XY changes), while RPY remains close to the gravity-derived attitude. Next causal test must separate the now-fixed initialization error from subsequent backend bias evolution and then run the same 500 mm A→B→A test with exact-gravity initialization enabled.


### 2026-09-07 — полный A→B→A×2 после exact-gravity init

Результат причинного теста после исправления Hypothesis 1A:

- LEG1 A→B = 491.62 мм, ошибка -8.38 мм, dz +18.54 мм;
- LEG2 B→A = 528.52 мм, ошибка +28.52 мм, dz -35.95 мм;
- LEG3 A→B = 501.41 мм, ошибка +1.41 мм, dz +27.64 мм;
- LEG4 B→A = 531.71 мм, ошибка +31.71 мм, dz -6.97 мм;
- A→B mean = 496.52 мм;
- B→A mean = 530.12 мм;
- overall mean = 513.32 мм;
- reversal angles = 178.56° и 179.80°;
- pipeline = PASS;
- один loop stall 217.915 ms, но run не был помечен pipeline-invalid.

Сравнение с предыдущим характерным V25:
- раньше A→B мог быть 448.85 мм с сильным развалом B→A;
- после exact-gravity init оба A→B близки к 500 мм, а B→A больше не разваливается и возвращается почти в противоположном направлении.

**Вывод:** Hypothesis 1A была реальной крупной первопричиной и её исправление резко улучшило метрическую стабильность и устранило катастрофический reversal divergence.

Однако проблема НЕ закрыта:
- сохраняется устойчивый directional bias: B→A примерно +30 мм (+6%);
- сохраняется паразитный dz;
- VIO roll/pitch заметно меняются во время чистой трансляции.

Это создаёт следующую развилку:

#### Подгипотеза 1B.1 — после правильного seed backend снова создаёт значимый accelerometer bias
Проверить BA start/end по каждому leg после exact-gravity init.

#### Подгипотеза 2A — camera/body или visual geometry создаёт ложное изменение attitude при трансляции
Сопоставить изменение VIO R/P с FC ATTITUDE на каждом leg. Если FC остаётся почти неподвижным, а VIO меняет R/P, ошибка внутренняя для VIO/visual geometry.

Следующий тест — offline forensic без нового физического прогона: `tools/analyze_v25_post_gravity_init.py`.


### 2026-09-07 — correction: VIO vs FC attitude comparison needed FRD→FLU sign mapping

The first post-gravity-init forensic analyzer compared VIO FLU Euler deltas directly with raw FC ATTITUDE deltas. That is not a valid like-for-like comparison: under the project's FRD→FLU convention, relative roll keeps its sign while pitch and yaw invert.

Example from the full run:
- LEG1 VIO dRPY = [-2.034,-1.834,-0.056] deg;
- FC raw dRPY = [-2.035,+1.762,+0.041] deg;
- after FRD→FLU sign mapping FC ≈ [-2.035,-1.762,-0.041] deg.

Thus the apparent ~3.6° residual tilt reported by the old analyzer is largely a convention artifact, not evidence that FC stayed fixed while VIO tilted internally.

The analyzer `tools/analyze_v25_post_gravity_init.py` has been corrected to print both raw FC and FC-mapped-to-FLU deltas.

Implication for the hypothesis tree:
- Hypothesis 2A ("VIO attitude changes while FC is stationary") is NOT supported by the uncorrected output and must not be treated as established.
- The physical stand/manipulation may really be changing roll/pitch during translation, and VIO appears to follow those changes closely.
- Hypothesis 1B remains relevant because accelerometer bias grows substantially again during measured legs even after a near-zero gravity-aligned initialization.
- The remaining directional scale bias (A→B ≈ correct, B→A ≈ +6%) must now be analyzed with corrected attitude comparison before prioritizing camera extrinsics.


### 2026-09-07 — corrected post-gravity-init forensic result

After correcting FC attitude signs into FLU, VIO and FC attitude changes agree closely on all four measured legs:
- LEG1 residual tilt 0.072°;
- LEG2 residual tilt 0.195°;
- LEG3 residual tilt 0.144°;
- LEG4 residual tilt 0.112°.

Therefore the previously suspected "VIO tilts internally while FC stays fixed" branch is not supported. The observed roll/pitch changes are largely physical and confirmed by FC.

At the same time, accelerometer bias becomes substantial again after the exact-gravity startup:
- LEG1 |ΔBA| = 0.22055 m/s²;
- LEG2 |ΔBA| = 0.04401 m/s²;
- LEG3 |ΔBA| = 0.01559 m/s²;
- LEG4 |ΔBA| = 0.08945 m/s².

But raw |ΔBA| alone does not explain scale: LEG1 has the largest bias change while its distance is close to correct, whereas B→A legs have smaller |ΔBA| yet systematic +5.7…+6.3% scale.

This creates a narrower child hypothesis:

#### Подгипотеза 1B.2 — direction-dependent projection of estimated accelerometer bias
The relevant quantity may be not total |BA| but its projection in world coordinates along the measured leg. Test with `tools/analyze_v25_bias_projection.py`.

In parallel, Hypothesis 3 remains open: if bias projection does not track the directional scale error, run the existing LOW_DISPARITY/ZUPT guard on this exact dataset.


### 2026-09-07 — Test 1B.2: accelerometer-bias projection along each leg

Command: `python3 tools/analyze_v25_bias_projection.py`

Result:
- LEG1 A→B: scale 0.9832, mean BA_along = -0.03823 m/s².
- LEG2 B→A: scale 1.0570, mean BA_along = +0.18045 m/s².
- LEG3 A→B: scale 1.0028, mean BA_along = -0.15603 m/s².
- LEG4 B→A: scale 1.0634, mean BA_along = +0.18485 m/s².
- Direction means: A→B scale 0.9930 / BA_along -0.09713 m/s²; B→A scale 1.0602 / BA_along +0.18265 m/s².

Conclusion: **Hypothesis 1B.2 gains strong support.** Both B→A legs independently reproduce almost the same positive along-leg estimated accelerometer bias (+0.180/+0.185 m/s²) together with almost the same positive scale error (+5.7/+6.3%). A→B has the opposite BA projection and mean scale close to 1.0.

Important caveat: the printed `0.5*a*t²` values (several metres) are only dimensional scale indicators. They are not predicted VIO position error because BA is an estimated state inside the coupled optimizer and is actively compensated. Do not interpret those metre values literally.

#### Подгипотеза 1B.3 — is BA direction dependence caused by actual FC accelerometer asymmetry or by VIO bias-state estimation?

Next test must separate raw measured acceleration from the backend-estimated BA on each direction. For every leg:
1. transform/match raw FC IMU into the same FLU/world convention;
2. compare raw acceleration statistics projected along the measured leg;
3. compare those projections with backend BA_along;
4. check whether the sign flip follows the raw sensor or appears only in the optimizer state.

Decision:
- raw FC acceleration shows the same repeatable directional asymmetry → inspect FC accelerometer calibration, mounting/frame semantics, vibration/hand-motion excitation;
- raw FC acceleration does **not** show it while backend BA does → bias-state estimation / visual-inertial coupling becomes the primary branch.

Hypothesis 3 (LOW_DISPARITY/ZUPT) remains open as an independent guard and should still be tested on this dataset; it is not yet eliminated by Test 1B.2.


### 2026-09-07 — Test Hypothesis 3: slow-motion / ZUPT guard

Command: `python3 tools/analyze_v25_slow_motion_zupt_guard.py`

All four measured legs returned `PASS_GUARD`:
- LEG1 A→B: 65.1% LOW_DISPARITY, 491.62 mm, PASS_GUARD.
- LEG2 B→A: 59.0% LOW_DISPARITY, 528.52 mm, PASS_GUARD.
- LEG3 A→B: 43.6% LOW_DISPARITY, 501.41 mm, PASS_GUARD.
- LEG4 B→A: 42.9% LOW_DISPARITY, 531.71 mm, PASS_GUARD.

Observed pose plateaus are concentrated at the beginning/end of legs and do not indicate that the stationary constraint froze the completed real translation.

Conclusion: **the current simple Hypothesis 3 (LOW_DISPARITY/ZUPT freezes real slow motion and directly causes the directional distance error) is not supported by this dataset.** Keep it as a regression/guard branch, not the primary root-cause branch.

Priority remains Hypothesis 1B.2/1B.3:
- B→A independently repeats BA_along ≈ +0.18 m/s² and scale ≈ 1.06;
- next discriminator is raw FC acceleration projected along each leg versus backend-estimated BA projection.
