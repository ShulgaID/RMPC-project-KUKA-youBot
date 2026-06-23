# youbot_perception — детекция, захват и перестановка кубика по цвету

Пакет решает задачу для KUKA youBot (ROS Noetic, Gazebo + реальный робот):

1. По приезду в точку **B** найти кубик 5×5 см и **определить его цвет**
   (red / green / blue) по камере.
2. **Захватить** кубик и перенести в точку **C**.
3. В точке C лежат три картонки red/green/blue — поставить кубик на картонку
   **соответствующего цвета**.
4. Работает в **Gazebo**, и тот же код запускается на **реальном роботе**.

## Состав

| Файл | Назначение |
| --- | --- |
| `scripts/cube_detector.py` | Детекция цвета кубика по HSV + 3D-позиция из облака точек RGB-D, трансформация в `odom`. Публикует `/cube/color` (String) и `/cube/pose` (PoseStamped). |
| `scripts/pick_place_mission.py` | Конечный автомат миссии: едет в B → ждёт цвет от детектора → захват → едет в C → подъезжает к картонке нужного цвета → ставит кубик → возврат в A. |
| `scripts/spawn_task_objects.py` | (Только Gazebo) спавнит кубик **случайного** цвета в B и три картонки в C. Истинный цвет НЕ передаётся в детектор — это честная проверка. |
| `urdf/youbot_task.urdf.xacro` | youBot + башенная 3D-камера (`tower_cam3d`) + `gazebo_grasp_fix` для надёжного захвата. |
| `launch/pick_place_sim.launch` | Полный запуск в Gazebo. |
| `launch/pick_place_real.launch` | Запуск на реальном роботе (драйвер + камера + детекция + миссия). |

## Запуск в Gazebo

```bash
catkin build youbot_perception        # или catkin_make
source devel/setup.bash
roslaunch youbot_perception pick_place_sim.launch
# зафиксировать цвет кубика для отладки:
roslaunch youbot_perception pick_place_sim.launch cube_color:=green
```

Полезные топики для проверки:
- `/cube/color` — определённый цвет;
- `/cube/debug_image` — кадр с разметкой найденного кубика (смотреть в `rqt_image_view`);
- `/cube/pose` — позиция кубика в `odom`.

## Запуск на реальном роботе

```bash
roslaunch youbot_perception pick_place_real.launch \
    rgb_topic:=/camera/rgb/image_raw \
    points_topic:=/camera/depth_registered/points
```

Перед этим отдельно поднимите драйвер камеры (openni2 / astra / realsense),
который публикует RGB и **выровненное** (registered) облако точек, и убедитесь,
что есть TF от кадра камеры до `odom`. HSV-пороги в `cube_detector.py`
подгоняются под реальное освещение.

## Что переиспользуется из основного проекта

- движение базы — `youbot_mecanum_controller` (`/cmd_vel`, одометрия `/odom`);
- рука/схват — `arm_controller` / `gripper_controller` (`JointTrajectory`);
- точки A/B/C и препятствия — `room_generator` (`/room/point_*`).

## Зависимости

`rospy, cv_bridge, python3-opencv, numpy, tf2_ros, tf2_geometry_msgs,
gazebo_msgs`. Для надёжного захвата в Gazebo — плагин `gazebo_grasp_fix`
(пакет `gazebo-pkgs` / `roboticsgroup_gazebo_plugins`). Если он не установлен,
удалите блок `gazebo_grasp_fix` из URDF — захват будет держаться на трении.

## Что нужно докрутить под свою сцену

- Позы руки `ARM_*` в `pick_place_mission.py` подобраны как разумная отправная
  точка; уточните углы под фактическую высоту кубика/картонки (проверяйте в
  RViz/Gazebo по `/joint_states`).
- Раскладку картонок (`pad_red/green/blue`) задайте под реальные координаты.
