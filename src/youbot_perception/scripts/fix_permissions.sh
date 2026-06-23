#!/usr/bin/env bash
# Возвращает флаг +x всем исполняемым Python-узлам проекта.
# Запуск:  bash fix_permissions.sh
set -e

# Каталог src проекта определяем относительно этого скрипта
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$(cd "$HERE/../.." && pwd)"   # .../src

echo "src = $SRC"

chmod +x "$SRC"/youbot_perception/scripts/cube_detector.py
chmod +x "$SRC"/youbot_perception/scripts/pick_place_mission.py
chmod +x "$SRC"/youbot_perception/scripts/spawn_task_objects.py

chmod +x "$SRC"/youbot_controller/scripts/youbot_controller.py        2>/dev/null || true
chmod +x "$SRC"/youbot_controller/scripts/room_generator.py          2>/dev/null || true
chmod +x "$SRC"/youbot_controller/scripts/visualization_node.py      2>/dev/null || true
chmod +x "$SRC"/youbot_controller/scripts/trajectory_planner.py      2>/dev/null || true
chmod +x "$SRC"/youbot_controller/scripts/controller.py              2>/dev/null || true
chmod +x "$SRC"/youbot_controller/scripts/rrt_planner.py             2>/dev/null || true
chmod +x "$SRC"/youbot_mecanum_controller/scripts/mecanum_controller.py 2>/dev/null || true

echo "Готово: права на исполняемые узлы выставлены."
