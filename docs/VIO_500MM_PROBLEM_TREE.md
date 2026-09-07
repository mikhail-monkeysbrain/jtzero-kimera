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

> Ниже — краткая навигационная структура расследования. Подробный журнал и численные результаты сохранены ниже и не заменяются этой схемой.

- **Проблема P0 — VIO не даёт воспроизводимое измерение 500 мм A→B→A**
  - **H1 — начальная ориентация gravity и accelerometer bias**
    - **T1.1 — stationary accel/bias forensic**
      - Результат: большой BA связан с несогласованностью stationary acceleration и backend attitude.
      - Породил:
        - **H1A — BA создаётся при IMU initialization**
          - **T1A.1 — JT-IMU-INIT**
            - PASS: Kimera стартовала с RPY≈0 и большим BA.xy.
          - **T1A.2 — exact gravity initialization**
            - PASS: найден deadband AlignGravityVectors; exact-gravity почти обнулил initial BA.xy и правильно задал roll/pitch.
            - Полный 500-мм тест:
              - A→B существенно улучшился.
              - Катастрофический reversal divergence исчез.
              - Осталась ошибка масштаба/dz.
            - Итог: **H1A подтверждена и исправлена; это реальная найденная первопричина, но не единственная причина остаточной ошибки.**
        - **H1B — backend повторно создаёт BA после правильного seed**
          - **T1B.1 — post-gravity-init forensic**
            - PASS: BA снова растёт после корректной инициализации.
          - **T1B.2 — projection BA along leg**
            - Поддержка: backend BA показывал направленную асимметрию.
          - **T1B.3 — raw FC accel vs backend BA**
            - Результат: raw FC acceleration асимметрию не повторяет.
            - Породил:
              - **H1B.4 — BA создаётся внутри visual-inertial optimization**
                - **T1B.4 — BA timeline vs frontend**
                  - Поддержка: основные изменения BA происходят во время VALID visual motion.
                - **H1B.5 — BA слишком свободен из-за accelerometer_random_walk**
                  - **T1B.5 — ARW 0.003 → 0.0003**
                    - FAIL: траектория ухудшилась.
                    - Итог: простое ужесточение ARW отвергнуто как решение.
                    - Ограничение вывода: ручные профили движения разных прогонов оказались несопоставимы.
  - **H2 — camera↔body extrinsics T_BS**
    - Ранее исследовались корректировки T_BS.
    - Текущий статус: открыта, но нет достаточного основания объявлять её причиной остаточной ошибки.
  - **H3 — LOW_DISPARITY/ZUPT подавляет реальное медленное движение**
    - **T3.1 — slow-motion/ZUPT guard**
      - PASS_GUARD на 4 legs.
      - Итог: простая версия «ZUPT замораживает движение и создаёт directional error» ослаблена; оставить как regression guard.
  - **H4 — IMU frame / gyro correction**
    - FRD→FLU проверен; ZXY и gravity-feedback исследовались.
    - Статус: сильно ослаблена; возвращаться только при новых противоречащих данных.
  - **H5 — timing / pipeline stalls**
    - Stalls способны инвалидировать отдельный run.
    - Не объясняют найденный initialization defect.
    - Правило: invalid run не использовать для причинных выводов.
  - **H6 — visual geometry / mono scale observability / motion excitation**
    - **T6.1 — BA vs visual quality**
      - Первый анализ оказался привязан к ARW=0.0003 dataset и не может доказывать baseline-причину.
      - Это выявило ошибку provenance; после этого введено обязательное архивирование runs.
    - **T6.2 — fresh archived baseline**
      - Результат значительно отличается от предыдущего baseline.
      - Породил:
        - **H6A — ручной профиль движения является сильной мешающей переменной**
          - **T6A.1 — сравнение архивных runs**
            - Результат: длительность, скорость и visual quality заметно различаются одновременно с scale.
            - Статус: **поддержана как экспериментальный confounder, но не доказана как первопричина VIO.**
          - **Следующий тест**
            - Механический линейный стенд **исключён как невозможный по условиям проекта**.
            - Использовать untimed ручной протокол: START/END по Space/Enter; физическая метка 500 мм; сопоставимость оценивать постфактум по записанному профилю движения.
            - Делать несколько повторов CONTROL и TEST, а не сравнивать по одному run.
            - До сравнения параметров автоматически проверять сопоставимость runs по duration, mean/max speed, FC attitude excursion, VALID/LOW_DISPARITY fraction и inlier statistics.
            - Если runs несопоставимы — результат **INCONCLUSIVE**, а не PASS/FAIL параметра.

### Методологический контроль для всех следующих экспериментов

- **До запуска**
  - сформулировать одну проверяемую гипотезу;
  - указать одну изменяемую переменную;
  - перечислить неизменяемые параметры;
  - заранее записать prediction;
  - заранее записать критерии SUPPORTED / REJECTED / INCONCLUSIVE;
  - определить минимальное число повторов.
- **Во время запуска**
  - не менять методику движения между CONTROL и TEST;
  - сохранять commit JT-Zero и Kimera, params и environment;
  - автоматически архивировать каждый run сразу после завершения;
  - фиксировать stalls/disconnects как invalidation, а не как результат гипотезы.
- **Перед причинным сравнением**
  - проверить сопоставимость физического движения;
  - проверить frontend quality;
  - проверить pipeline validity;
  - если различия входного воздействия велики — не делать причинный вывод.
- **После теста**
  - обновить это дерево;
  - записать результат даже при FAIL/INCONCLUSIVE;
  - новые объяснения оформлять дочерними подгипотезами;
  - не менять задним числом критерий успеха.

### Ближайший шаг

- Не менять пока параметры Kimera.
- Сначала стандартизировать **ручной** 500-мм протокол без механического стенда.
- Получить серию повторов baseline ARW=0.003 exact-gravity.
- Измерить естественный разброс baseline при максимально одинаковом ручном движении.
- Только после этого повторять A/B параметрические тесты.



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


### 2026-09-07 — Test 1B.3: raw FC acceleration versus backend-estimated BA

Command: `python3 tools/analyze_v25_raw_vs_backend_bias.py`

Result:
- A→B direction mean: scale 0.9930, RAW_along -0.00653 m/s², backend BA_along -0.09713 m/s².
- B→A direction mean: scale 1.0602, RAW_along -0.00024 m/s², backend BA_along +0.18265 m/s².
- Individual raw along-leg means remain close to zero: LEG1 -0.00650, LEG2 -0.00121, LEG3 -0.00655, LEG4 +0.00073 m/s².
- Backend BA, in contrast, has a large repeatable directional split.

Conclusion: **raw FC acceleration does not reproduce the backend BA directional asymmetry.** This substantially weakens the sensor/calibration/mechanical-asymmetry branch as the direct source of the ~6% B→A scale error. The asymmetry appears after the raw IMU enters the VIO estimation pipeline.

#### Подгипотеза 1B.4 — backend bias-state / visual-inertial coupling creates a direction-dependent BA estimate
Priority: HIGH.

Next discriminator should determine *when* BA separates from the raw-IMU-consistent value relative to visual motion:
- trace BA_along versus keyframe/time for each leg;
- align it with frontend status/disparity/inlier changes and motion onset/stop;
- determine whether BA moves mainly during visual VALID motion, during LOW_DISPARITY intervals, or during endpoint settling.

This is now a deeper internal VIO branch, not a broad hardware/FC branch.


### 2026-09-07 — Test 1B.4: when does BA move?

Command: `python3 tools/analyze_v25_bias_timeline_vs_frontend.py`

Result:
- LEG1: VALID contributes sum|dBA|=0.46843, signed=-0.21636; LOW_DISPARITY only 0.00993.
- LEG2: VALID contributes 0.13636, signed=-0.04556; LOW_DISPARITY only 0.00989.
- LEG3: VALID contributes 0.09398, signed=+0.01354; LOW_DISPARITY only 0.01035.
- LEG4: VALID contributes 0.16810, signed=+0.04959; LOW_DISPARITY only 0.00513.

Conclusion: **BA changes overwhelmingly during VALID visual motion, not during LOW_DISPARITY stationary handling.** This strongly weakens endpoint/ZUPT as the mechanism that creates the directional bias and strengthens the visual-inertial coupling / scale-observability branch.

#### Подгипотеза 1B.5 — accelerometer bias is too free during visual-inertial optimization

Controlled causal test:
- baseline `accelerometer_random_walk = 0.003`;
- test value `0.0003` (10× tighter);
- all other parameters and exact-gravity initialization unchanged.

Runner: `tools/run_v25_arw_0003_test.sh`.

Decision:
- if BA_along directional split and B→A +6% scale error both shrink, bias-state freedom is causal;
- if BA is constrained but scale error remains, BA is mainly a symptom and visual scale/geometry becomes primary;
- if the run degrades globally, 0.0003 is too restrictive and the result still informs the model.


### 2026-09-07 — Test 1B.5 FAIL: 10× tighter accelerometer random walk degrades V25

Controlled change:
- baseline `accelerometer_random_walk = 0.003`;
- test `accelerometer_random_walk = 0.0003`;
- exact-gravity init, staged ZUPT and the remaining V25 configuration unchanged;
- pipeline valid, LOOP STALLS >100 ms = 0.

Result:
- LEG1 A→B = 535.92 mm, error +35.92 mm, dz -47.93 mm;
- LEG2 B→A = 536.83 mm, error +36.83 mm, dz +199.53 mm;
- LEG3 A→B = 611.68 mm, error +111.68 mm, dz -51.97 mm;
- LEG4 B→A = 597.54 mm, error +97.54 mm, dz +240.17 mm;
- A→B mean = 573.80 mm;
- B→A mean = 567.19 mm;
- overall mean = 570.49 mm, scale = 1.140985;
- reversal angles remain near opposite: 178.14° / 176.49°;
- pipeline PASS, measurement FAIL.

Interpretation: **simply preventing accelerometer bias from adapting is not a fix.** The trajectory becomes substantially worse, especially scale and vertical displacement. Therefore the backend needs appreciable BA freedom to reconcile the current visual/inertial observations.

This is a causal result:
- Hypothesis 1B.5 in its simple form ("BA is too free; tighten ARW and distance improves") is rejected.
- The observed BA evolution is not safely removable as an independent nuisance state; it is participating in compensation of another inconsistency/weakly-observable mode.
- Because BA changes were already shown to occur mainly during VALID visual motion, priority shifts further toward the source that forces this compensation: visual/IMU scale observability, motion excitation, or visual geometry/model consistency.
- Do not tune ARW further as the next step. Restore baseline 0.003 for subsequent tests.

Next branch: quantify whether the remaining error follows visual motion geometry/mono scale evidence within each VALID segment, rather than changing another optimizer prior.


### 2026-09-07 — visual-quality analysis after ARW=0.0003 run: dataset provenance correction

The command `tools/analyze_v25_bias_vs_visual_quality.py` was run after the controlled ARW=0.0003 experiment. Therefore it analyzed the **degraded ARW=0.0003 dataset**, because V25 CSV files in `/home/vio/jtzero_500mm_v25_*.csv` are overwritten by each new run.

Observed on the ARW=0.0003 dataset:
- LEG1 BA change is concentrated in weak VALID visual updates, especially inlier ratio 0.20–0.40 and <0.20.
- LEG2 shows a very large LOW_DISPARITY contribution (`sum|dBA|=0.52840`) that was not present in the earlier baseline dataset.
- Global VALID bins still show the largest signed BA changes in weak bins (<0.40), but this cannot be used as evidence for the baseline remaining +6% B→A error because the optimizer prior was intentionally changed and the trajectory degraded strongly.

Conclusion: this result is valid **only for the ARW=0.0003 failure mode**. It must not be merged with the baseline causal tree as if it were the same dataset.

Methodological action:
- preserve every future V25 dataset under a run-specific tag before starting another run;
- re-run the visual-quality analysis on a baseline ARW=0.003 exact-gravity dataset before deciding whether weak visual geometry is causal for the original residual error.


### 2026-09-07 — experiment archival safeguard enabled

The degraded `ARW=0.0003` run was archived locally before any new V25 run:
`/home/vio/jtzero_runs/20260907_114023_v25_ARW_0003_FAIL`.

To prevent future dataset provenance mistakes, a dedicated baseline runner was added:
`tools/run_v25_baseline_exact_archived.sh`.

It:
- forces the baseline `ARW=0.003` parameter profile;
- keeps exact-gravity initialization enabled;
- runs the same V25 A→B→A×2 test;
- automatically archives all V25 CSV outputs with the actual parameter directory immediately after the run.

Next action: obtain a fresh archived baseline dataset and only then re-run the visual-quality/BA correlation on that baseline.


### 2026-09-07 — fresh baseline ARW=0.003 exact-gravity run shows large run-to-run variability

Archived run:
`/home/vio/jtzero_runs/20260907_114328_v25_BASELINE_ARW_003_EXACT`

Result:
- LEG1 A→B = 520.44 mm, error +20.44 mm, dz -2.12 mm;
- LEG2 B→A = 590.95 mm, error +90.95 mm, dz +100.63 mm;
- LEG3 A→B = 523.47 mm, error +23.47 mm, dz -17.22 mm;
- LEG4 B→A = 580.58 mm, error +80.58 mm, dz +58.96 mm;
- A→B mean = 521.95 mm;
- B→A mean = 585.76 mm;
- overall scale = 1.10772;
- reversal angles remain close to 180°;
- pipeline PASS;
- one loop stall 354.857 ms and one VIO jump at KF118 (`dP=116.59 mm`, `dt=464.02 ms`).

This baseline is much worse than the earlier exact-gravity baseline (A→B ≈496.5 mm, B→A ≈530.1 mm). Therefore the remaining error is not yet reproducible as a fixed +6% directional scale bias. Manual motion profile / excitation and/or transient visual-inertial conditions have substantial influence.

Methodological consequence:
- do not interpret the ARW=0.0003 vs 0.003 pair as a clean parameter A/B test yet, because the two manual motion profiles may differ strongly;
- first compare archived runs for duration, speed, frontend quality and motion excitation;
- future causal parameter tests should use more controlled motion or at least matched speed/duration windows.

New tool: `tools/compare_v25_archived_runs.py`.


### 2026-09-07 — manual protocol corrected: no timers, GUI-only operator control

Constraint clarified:
- a fixed movement timer is not acceptable because the operator may need more or less time for a valid 500 mm pass;
- CLI/metronome control is not used for the physical movement protocol;
- mechanical linear stand remains unavailable.

Action:
- removed `tools/v25_manual_motion_metronome.py`;
- V25 HUD now exposes a clickable GUI action button in the existing OpenCV window;
- the operator clicks **START** only when the vehicle is physically ready and stationary;
- after start stabilization the operator moves 500 mm at a comfortable, smooth speed;
- the operator clicks **END** only after reaching the physical 500 mm mark and stopping;
- no movement-duration target is imposed;
- keyboard remains only for emergency/normal quit (Q/ESC), not for START/END.

Methodological implication:
- reproducibility will be judged *after the run* from recorded motion profile (duration, velocity, FC attitude, frontend quality), not forced by a timer;
- multiple runs are accepted for A/B comparison only when their measured input profiles are sufficiently similar;
- otherwise the comparison is INCONCLUSIVE.


### 2026-09-07 — operator control finalized: untimed GUI with Space/Enter START/END

Final interaction rule for V25 manual 500 mm tests:
- no movement timer;
- operator works in the V25 GUI window;
- **Space or Enter** starts the current leg when the vehicle is stationary and ready;
- after the 500 mm physical movement and stop, **Space or Enter** ends the leg;
- mouse click on the action area may remain as an optional duplicate control, but keyboard Space/Enter is the primary workflow;
- Q/ESC exits the test.

This preserves the original fast operator workflow while keeping the experiment untimed.


### 2026-09-07 — V25 GUI mouse callback removed after Qt startup crash

Observed:
`[FATAL] OpenCV(4.10.0) ... window_QT.cpp:753 ... NULL window handler in cvSetMouseCallback`.

Cause is in the newly added optional mouse-control path, not in VIO/Kimera. The callback was registered against the Qt HighGUI window and caused startup failure in the current VNC/Qt environment.

Action:
- removed mouse callback support entirely;
- START/END is again **Space or Enter only** in the existing GUI window;
- no timers;
- Q/ESC exits.

This run produced no VIO measurement data and must not be counted as an experiment result.


### 2026-09-07 — CLEAN MANUAL PROFILE 01: directional error reproduced without stalls

Current run:
- exact-gravity initialization enabled;
- baseline ARW=0.003;
- untimed START/END by Space/Enter;
- pipeline PASS;
- LOOP STALLS >100 ms = 0.

Results:
- LEG1 A→B = 504.88 mm, dz -14.77 mm;
- LEG2 B→A = 554.67 mm, dz +36.40 mm;
- LEG3 A→B = 514.68 mm, dz -4.29 mm;
- LEG4 B→A = 541.65 mm, dz +104.50 mm;
- A→B mean = 509.78 mm;
- B→A mean = 548.16 mm;
- overall mean = 528.97 mm, scale = 1.057941;
- pair reversal angles = 179.27° / 179.42°.

Interpretation:
- the cleaner manual run is substantially better than the immediately preceding baseline, especially A→B;
- nevertheless, the B→A overestimation repeats on both return legs;
- the same return direction also shows positive dz, strongest on LEG4;
- because there are no loop stalls, timing/pipeline stalls do not explain this run.

Status for the problem tree: **PРИБЛИЗИЛИСЬ** — the directional residual survives in a cleaner, stall-free run, so it is less likely to be only a run-quality artifact.

Next action before changing any parameter:
1. archive this exact dataset;
2. on this same dataset run raw-FC-vs-backend-BA, post-gravity-init attitude comparison, and BA-vs-frontend timeline;
3. compare A→B versus B→A within this one run. No new physical run until that analysis is complete.


### 2026-09-07 — MANUAL_PROFILE_01_CLEAN forensic: backend BA is not present in raw FC acceleration

Dataset archive:
- `/home/vio/jtzero_runs/20260907_122838_v25_MANUAL_PROFILE_01_CLEAN`

Same-run forensic results:
- A→B mean scale = 1.0196; RAW_along = -0.01472 m/s²; backend BA_along = -0.18708 m/s².
- B→A mean scale = 1.0963; RAW_along = -0.00988 m/s²; backend BA_along = +0.31039 m/s².
- Raw FC along-axis acceleration is therefore near-zero and nearly direction-symmetric, while backend BA is large and changes sign with direction.
- VIO attitude agrees very closely with FC after FRD→FLU conversion: mean residual tilt A→B 0.071°, B→A 0.035°.
- Therefore the residual scale/dz error is not explained by an internal VIO-vs-FC roll/pitch disagreement.
- BA evolution is concentrated primarily around visual-motion/transition updates; LOW_DISPARITY also contributes on later legs, so it remains a secondary coupling path rather than a clean standalone cause.

Tree update:
- **H1B.4 — backend visual-inertial optimization creates direction-dependent accelerometer-bias state: strongly supported.**
  - **T1B.4a — same-run raw FC vs backend BA:** PASS for optimizer-origin interpretation.
  - **T1B.4b — same-run VIO vs FC attitude:** PASS; attitude mismatch rejected as explanation of this residual.
  - **T1B.4c — BA timeline vs frontend:** mixed but informative; visual-motion updates dominate LEG1/2/3, while LEG4 has substantial LOW_DISPARITY contribution.
  - New child hypothesis:
    - **H1B.4.1 — monocular visual scale/translation constraints are being traded against BA during translation.**
      - Next test must inspect visual translation/parallax/feature geometry versus backend BA and scale on this archived dataset before changing Kimera parameters.
    - **H1B.4.2 — motion→stationary transition factors can amplify an already-created BA error.**
      - Secondary; test only after H1B.4.1 because it cannot explain the raw-vs-backend directional BA by itself.

Status: **ПРИБЛИЗИЛИСЬ.** A clean same-run comparison removed two broad explanations: physical FC acceleration asymmetry and VIO attitude mismatch. The investigation is now narrower: the large directional BA is generated inside the visual-inertial estimation/coupling path.


### 2026-09-07 — methodological correction: along-leg BA sign flip may be a projection artifact

Important correction to H1B.2/H1B.3 interpretation:

Previous analyzers projected backend BA onto **each leg's own motion direction**. Because B→A reverses the unit direction vector, a persistent world-frame BA automatically changes the sign of its along-leg projection even if the BA vector itself does not reverse.

Therefore:
- `BA_along < 0` on A→B and `BA_along > 0` on B→A is **not by itself evidence of direction-dependent bias generation**;
- the same persistent BA vector can produce exactly that sign pattern under opposite projection axes;
- conclusions that relied on the sign flip alone must be downgraded pending a fixed-axis analysis.

What remains valid:
- raw FC acceleration stays near zero while backend BA becomes large;
- VIO attitude agrees closely with FC after FRD→FLU mapping;
- backend BA evolves during the run and participates in the residual scale problem.

New discriminator:
- `tools/analyze_v25_bias_common_axis.py`
- use one fixed world-horizontal A→B axis for all four legs;
- if BA on that common axis keeps one sign across reversals, the earlier "directional BA" interpretation was a coordinate/projection artifact;
- if common-axis BA itself reverses with motion direction, true direction-dependent estimator behavior remains supported.

Status: **ОТКАТИЛИСЬ** on the narrow claim "BA itself flips with direction" because the prior metric was not frame-invariant. The broader finding "backend BA is large while raw FC acceleration is near zero" remains supported.


### 2026-09-07 — fixed-axis BA result: prior directional-sign interpretation rejected

On `MANUAL_PROFILE_01_CLEAN`, one fixed A→B world axis was used for all legs.

Result:
- LEG1 A→B BA_common mean = -0.06492 m/s²;
- LEG2 B→A BA_common mean = -0.26419 m/s²;
- LEG3 A→B BA_common mean = -0.30898 m/s²;
- LEG4 B→A BA_common mean = -0.35883 m/s².

The sign remains negative across all reversals. Therefore the earlier A→B negative / B→A positive `BA_along` pattern was indeed caused by projecting onto opposite leg directions.

Conclusion:
- **the claim that backend BA itself reverses sign with motion direction is rejected**;
- backend BA instead appears to persist/drift in one world-frame direction through the run;
- BA magnitude grows substantially over time, but this alone does not map cleanly to scale error: LEG3 has a larger |BA_common| than LEG2 while its scale is much closer to 1.0;
- therefore persistent BA remains an important state-estimation symptom, but is not yet sufficient to explain the B→A scale excess.

Tree consequence:
- H1B.2 directional-sign branch is closed as a projection artifact;
- H1B.4 remains open in a narrower form: why does backend BA drift/persist away from raw FC acceleration?
- H6/manual-motion/visual-observability branch remains necessary to explain why B→A scale is worse even when BA does not reverse.

Next discriminator:
- compare A→B and B→A **within this same archived run** for duration, speed distribution, acceleration proxy, FC attitude excursion, VALID/LOW_DISPARITY ratio and inlier quality.
- Tool: `tools/analyze_v25_same_run_motion_visual.py`.


### 2026-09-07 — methodological correction: backend speed cannot prove input-motion differences

The same-run motion/visual comparison showed:
- B→A has larger reported VIO speed and shorter duration on average;
- LEG4 also has much lower VALID fraction than LEG3.

However, the speed used by that analyzer was `backend speed_m_s`, i.e. a VIO output. Since scale is the quantity under investigation, using VIO speed to prove that the physical input motion was different is circular.

Therefore:
- duration from operator START/END remains a valid independent observable;
- FC attitude excursion remains independent enough for comparison;
- frontend status/inlier quality is also independent of backend scale;
- **backend speed distribution must not be used as causal evidence for different physical motion excitation.**

New test:
- `tools/analyze_v25_raw_motion_profile.py`
- uses only raw FC IMU, FC ATTITUDE and operator event timestamps;
- compares horizontal acceleration RMS/p90/max, total dynamic acceleration, gyro, attitude spans and duration between legs;
- no backend velocity is used.

Status: **ОТКАТИЛИСЬ partially on the claim that B→A was physically faster based on VIO speed.** The observed scale asymmetry itself remains valid, as does the frontend-quality difference on LEG4.
