# Проверка fusion ExternalNav в EKF3

Дата: 2026-09-10
Ветка: `ground-motion-mvp`

## Что проверялось

Нужно было отделить две разные вещи:

1. приём `VISION_SPEED_ESTIMATE` автопилотом;
2. фактическое использование этой скорости EKF3.

Обычный физический сдвиг стенда для такой проверки неудобен: на одно и то же движение одновременно реагируют и Ground Motion, и IMU, поэтому корреляция `GM velocity -> FC velocity` сама по себе не доказывает fusion.

## Контроль A/B

При физическом движении были получены похожие реакции `GLOBAL_POSITION_INT.vx` как с:

```text
EK3_SRC1_VELXY = 6   # ExternalNav
```

так и с:

```text
EK3_SRC1_VELXY = 0   # None
```

Поэтому этот тест был признан методологически недостаточным для доказательства fusion.

## Стационарный синтетический тест

БПЛА оставался полностью неподвижным и DISARMED.

В `VISION_SPEED_ESTIMATE` подавался искусственный профиль скорости NED North:

```text
0..3 c   :  0.00 m/s
3..7 c   : +0.15 m/s
7..11 c  : -0.15 m/s
11..14 c :  0.00 m/s
```

Параметр:

```text
EK3_SRC1_VELXY = 6
```

Результат `GLOBAL_POSITION_INT.vx`:

```text
ZERO_PRE   n=60 mean_fc_vN= 0.000000 m/s  max|fc_vN|=0.00
PLUS_015   n=80 mean_fc_vN=+0.076875 m/s  max|fc_vN|=0.11
MINUS_015  n=80 mean_fc_vN=-0.062250 m/s  max|fc_vN|=0.13
ZERO_POST  n=60 mean_fc_vN=-0.042667 m/s  max|fc_vN|=0.13
```

`mean_fc_vE` во всех фазах оставался 0.

## Вывод

Fusion горизонтальной ExternalNav velocity в EKF3 подтверждён экспериментально.

Ключевой аргумент: БПЛА физически не двигался, поэтому IMU не мог создать наблюдаемое изменение знака скорости. При этом оценённая FC скорость:

- была 0 до подачи искусственного измерения;
- ушла в положительную сторону при `+0.15 m/s`;
- затем сменила знак при `-0.15 m/s`;
- после возврата команды к 0 начала релаксировать обратно.

Следовательно, цепочка

```text
Ground Motion / VISION_SPEED_ESTIMATE
        -> ArduPilot AP_VisualOdom
        -> EKF3 ExternalNav velocity fusion
```

работает.

## Что этот тест НЕ проверяет

Этот тест не подтверждает качество Ground Motion на реальном полёте и не определяет оптимальные EKF noise/gate параметры. Он подтверждает только факт, что ExternalNav velocity действительно попадает в EKF3 и влияет на его оценку скорости.

## Текущая рабочая конфигурация SRC1

```text
EK3_SRC1_POSXY = 0    # None
EK3_SRC1_VELXY = 6    # ExternalNav / Ground Motion
EK3_SRC1_POSZ  = 2    # RangeFinder / TF-Luna
EK3_SRC1_VELZ  = 0    # None
EK3_SRC1_YAW   = 0    # None
EK3_SRC_OPTIONS = 0
VISO_QUAL_MIN   = 0
VISO_POS_X      = 0
VISO_POS_Y      = 0
VISO_POS_Z      = 0
```

На этом этап диагностики факта fusion закрывается.
