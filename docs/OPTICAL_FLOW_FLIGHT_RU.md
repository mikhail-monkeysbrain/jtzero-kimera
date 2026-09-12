# JT-Zero — OpticalFlow flight launcher

Дата: 2026-09-12  
Ветка: `ground-motion-mvp`

## Назначение

После PASS pre-hover bench используется отдельный flight launcher.

Flight-контур:

```
OV9281 -> OPTICAL_FLOW
TF-Luna -> DISTANCE_SENSOR
ArduPilot EKF3 -> horizontal velocity / relative position
```

## Принципиальные отличия от bench

Flight launcher:

- не использует synthetic `bench-height=0.60`;
- не масштабирует flow через `real_height / 0.60`;
- передаёт в FC настоящий TF-Luna range;
- не включает guided bench;
- работает непрерывно до Ctrl+C;
- не ARM-ит FC и не меняет flight mode;
- не меняет параметры ArduPilot.

Перед запуском выполняется read-only audit параметров FC.

## Проверяемая конфигурация

- `FLOW_TYPE=5`
- `FLOW_OPTIONS=0`
- `FLOW_ORIENT_YAW=0`
- `FLOW_FXSCALER=0`
- `FLOW_FYSCALER=0`
- `EK3_FLOW_DELAY=0`
- `EK3_SRC1_POSXY=0`
- `EK3_SRC1_VELXY=5`
- `EK3_SRC1_POSZ=2`
- `EK3_SRC1_VELZ=0`
- `EK3_SRC1_YAW=0`

Текущий `JTZERO_FLOW_FOCAL_SCALE=1.1060`.

## Запуск

Сначала только аудит:

```bash
bash tools/audit_optical_flow_flight_params.sh
```

Затем publisher:

```bash
bash tools/run_optical_flow_flight.sh
```

До первого моторного hover необходимо отдельно выполнить обычный ArduPilot pre-arm/preflight и убедиться,
что TF-Luna реально видит землю в рабочем диапазоне.
