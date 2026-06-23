# RMPC · KUKA youBot — детекция, захват и перестановка кубика по цвету

Готовый к запуску проект ROS Noetic (Ubuntu 20.04, Gazebo + RViz).
Это исходный проект `RMPC-project-KUKA-youBot` **плюс** добавленный пакет
`src/youbot_perception`, который решает задачу:

1. приехать в точку **B** и определить **цвет** кубика 5×5 см по камере
   (red / green / blue);
2. **захватить** кубик и перенести его в точку **C**;
3. поставить кубик на картонку **соответствующего цвета**
   (red→red, green→green, blue→blue);
4. работает в **Gazebo**, тот же код запускается на **реальном роботе**.

Подробная пошаговая методичка — в `docs/` (PDF) и в
`src/youbot_perception/README.md`.

---

## 1. Установка зависимостей (один раз)

```bash
sudo apt update
sudo apt install terminator -y

# ROS-контроллеры (колёса/рука/схват)
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers \
 ros-noetic-gazebo-ros-control ros-noetic-velocity-controllers \
 ros-noetic-effort-controllers ros-noetic-joint-state-controller \
 ros-noetic-position-controllers ros-noetic-joint-trajectory-controller -y

# навигация/телеуправление
sudo apt install ros-noetic-navigation ros-noetic-teleop-twist-keyboard \
 ros-noetic-gmapping ros-noetic-amcl ros-noetic-map-server -y

# планировщик траектории
sudo apt install python3-scipy python3-numpy -y

# зрение (детекция цвета) + расстановка объектов
sudo apt install ros-noetic-cv-bridge ros-noetic-vision-opencv \
 ros-noetic-image-transport ros-noetic-tf2-geometry-msgs \
 ros-noetic-gazebo-msgs python3-opencv -y

# надёжный захват в Gazebo (если пакета нет — см. примечание ниже)
sudo apt install ros-noetic-gazebo-grasp-plugin -y || true
```

> **Если `gazebo-grasp-plugin` не ставится** из репозитория — либо соберите
> `gazebo-pkgs` из исходников, либо просто удалите блок
> `<plugin name="gazebo_grasp_fix">` из
> `src/youbot_perception/urdf/youbot_task.urdf.xacro`. Тогда захват держится
> на трении (менее надёжно, но запуск не сломается).

---

## 2. Размещение и сборка

Положите проект так, чтобы папка `src` оказалась внутри catkin-воркспейса
`~/youbot_ws`:

```bash
mkdir -p ~/youbot_ws
# скопируйте содержимое этого архива в ~/youbot_ws так, чтобы получилось:
#   ~/youbot_ws/src/youbot_perception/...
#   ~/youbot_ws/src/youbot_controller/...
#   ... и остальные пакеты

cd ~/youbot_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ~/youbot_ws/devel/setup.bash
```

Права на исполняемые узлы уже выставлены в архиве. Если после
копирования/распаковки флаг слетел — выполните готовый скрипт:

```bash
bash ~/youbot_ws/src/youbot_perception/scripts/fix_permissions.sh
```

---

## 3. Запуск задачи в Gazebo (один launch)

```bash
source /opt/ros/noetic/setup.bash
source ~/youbot_ws/devel/setup.bash
roslaunch youbot_perception pick_place_sim.launch
```

Поднимется Gazebo с youBot и камерой, контроллеры, комната с точками A/B/C,
кубик случайного цвета в B и три картонки в C, узел детекции и миссия.
Через ~10 с робот сам поедет: **B → детекция цвета → захват → C →
постановка на картонку нужного цвета → возврат в A**.

Зафиксировать цвет кубика для отладки:

```bash
roslaunch youbot_perception pick_place_sim.launch cube_color:=red
```

Проверка детекции в отдельной панели:

```bash
rostopic echo /cube/color           # определённый цвет
rqt_image_view /cube/debug_image    # кадр с разметкой
```

---

## 4. Запуск на реальном роботе

```bash
# 1) поднимите драйвер вашей камеры (Astra/Xtion/RealSense), публикующий
#    RGB и выровненное (registered) облако точек
# 2) запустите задачу с реальными топиками камеры:
roslaunch youbot_perception pick_place_real.launch \
  rgb_topic:=/camera/rgb/image_raw \
  points_topic:=/camera/depth_registered/points
```

Перед этим убедитесь, что есть TF от кадра камеры до `odom`, и при
необходимости подгоните HSV-пороги (`COLOR_RANGES` в
`cube_detector.py`) и позы руки (`ARM_*` в `pick_place_mission.py`).

---

## 5. Что добавлено к исходному проекту

```
src/youbot_perception/
├── scripts/
│   ├── cube_detector.py        # детекция цвета + 3D-позиция кубика (RGB-D)
│   ├── pick_place_mission.py   # конечный автомат: B→захват→C→постановка по цвету
│   ├── spawn_task_objects.py   # (Gazebo) кубик случайного цвета + 3 картонки
│   └── fix_permissions.sh      # вернуть +x всем узлам проекта
├── launch/
│   ├── pick_place_sim.launch   # полный запуск в Gazebo
│   └── pick_place_real.launch  # запуск на реальном роботе
├── urdf/
│   └── youbot_task.urdf.xacro  # youBot + башенная 3D-камера + grasp-fix
├── package.xml
├── CMakeLists.txt
└── README.md
```

Остальные пакеты (`youbot_controller`, `youbot_mecanum_controller`,
`youbot_description`, `youbot_simulation`, драйверы) — без изменений.
