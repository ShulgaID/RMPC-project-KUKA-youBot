# Структура проекта (после чистки)

Этот документ описывает АКТУАЛЬНУЮ структуру проекта после удаления
устаревшего кода и введения единого файла параметров.

## Единый файл параметров

**Все важные значения задачи лежат в одном файле:**

```
src/youbot_perception/config/mission_params.yaml
```

Точки интереса (ячейки A/B/C), цвет кубика и запасной цвет, радиус препятствий
и их разброс/частота, seed, габарит робота, скорости, допуски, число попыток
детекции — всё там. Меняете значение в этом файле → оно подхватывается всеми
нодами. Код трогать не нужно.

Механика: launch-файлы загружают YAML в namespace `/mission_config`. Ноды
читают значения через хелпер `mission_config.py` (приоритет: приватный
параметр ноды → `/mission_config/<группа>/<ключ>` → дефолт). То есть любой
параметр всё ещё можно точечно переопределить из launch, не трогая общий файл.

## Используемые пакеты

| Пакет | Назначение | Где задействован |
|---|---|---|
| `youbot_perception` | детекция куба, миссия pick&place, спавн объектов, URDF для Gazebo, центральный конфиг | sim + real |
| `youbot_controller` | room_generator (комната/точки/препятствия), RRT-планировщик, mission_config, контроллеры руки/схвата | sim + real |
| `youbot_description` | URDF/меши youBot | sim + real |
| `youbot_simulation/youbot_gazebo_control` | контроллеры суставов для Gazebo (`joint_state_controller`) | sim |
| `youbot_driver_ros_interface` | ROS-обёртка драйвера реального робота | real |
| `youbot_driver` | низкоуровневый драйвер youBot (EtherCAT) | real |
| `brics_actuator` | msg-зависимость драйвера/контроллеров | sim + real |

## Активные скрипты

`youbot_perception/scripts/`
- `cube_detector.py` — детекция цвета/позиции кубика по USB-камере;
- `pick_place_mission.py` — конечный автомат миссии;
- `spawn_task_objects.py` — расстановка кубика и картонок в Gazebo;
- `fix_permissions.sh` — утилита прав.

`youbot_controller/scripts/`
- `room_generator.py` — комната, сетка, точки A/B/C, препятствия;
- `rrt_planner.py` — RRT с объездом;
- `mission_config.py` — чтение единого конфига.

## Визуализация (RViz)

Готовая сцена RViz: `youbot_controller/config/mission_view.rviz`, запускается
через `youbot_controller/launch/rviz.launch` (или `pick_place_sim.launch
rviz:=true`). Показывает комнату/препятствия/точки (`/room/markers`), угловые
маркеры (`/field/markers`), запланированный путь (`/mission_planned_path`,
зелёный), фактически пройденный путь (`/mission_actual_path`, красный, из
`/odom`), модель робота и картинку камеры (`/cube/debug_image`). Это заменяет
прежний `visualization_node.py` и добавляет фактический след для сравнения
симуляции с реальностью.

## Точки входа (launch)

- Симуляция: `roslaunch youbot_perception pick_place_sim.launch`
- Реальный робот: `roslaunch youbot_perception pick_place_real.launch`

Оба грузят `mission_params.yaml` в `/mission_config`.

## Что было удалено (устаревшие попытки)

Старый стек управления, не используемый ни симуляцией, ни реальным роботом:
- скрипты `youbot_controller/scripts/`: `controller.py`, `youbot_controller.py`,
  `trajectory_planner.py`, `visualization_node.py`;
- launch'и `youbot_controller/launch/`: `youbot_controller_core.launch`,
  `youbot_controller.launch`, `youbot_my_sim.launch`, `youbot_sim.launch`,
  `gazebo_youbot.launch`;
- пакет `youbot_mecanum_controller` целиком;
- дубль `src/scripts/youbot_controller.py`;
- неиспользуемые Gazebo-пакеты: `youbot_gazebo_robot`, `youbot_gazebo_worlds`,
  метапакет `youbot_simulation` (пустой);
- мусор системы контроля версий (`.svn`), `*.save`, `__pycache__`.

## EtherCAT

Решение повторяющейся ошибки EtherCAT на реальном роботе —
`docs/ETHERCAT_TROUBLESHOOTING.md`.
