#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# spawn_task_objects.py
# -----------------------------------------------------------------------------
# Расставляет в Gazebo объекты задачи:
#   - кубик 5x5x5 см СЛУЧАЙНОГО цвета (red/green/blue) в точке B;
#   - три плоские картонки red/green/blue в точке C.
#
# Цвет кубика выбирается случайно (можно зафиксировать параметром ~cube_color
# или ~seed), и НЕ передаётся в узлы восприятия/миссии — его задача определить
# цвет камерой. Это честная проверка детекции.
#
# Координаты берём из room_generator (/room/point_B, /room/point_C), иначе
# из своих параметров. Картонки кладём теми же смещениями, что и
# pick_place_mission по умолчанию (red: -0.5 по Y, green: 0, blue: +0.5).
#
# Работает только в симуляции (использует /gazebo/spawn_sdf_model).
# =============================================================================

import random

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose


GAZEBO_RGBA = {
    'red':   '1 0 0 1',
    'green': '0 1 0 1',
    'blue':  '0 0 1 1',
}


def cube_sdf(name, rgba, size=0.05, mass=0.05):
    i = (1.0 / 6.0) * mass * size * size
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="col">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="vis">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def pad_sdf(name, rgba, sx=0.25, sy=0.25, sz=0.002):
    # Плоская цветная «картонка» БЕЗ коллизии — это просто метка на полу,
    # как настоящий тонкий картон. Без коллизии база свободно проезжает по
    # ней и не «залезает» на пластину.
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def make_pose(x, y, z):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = 1.0
    return p


def main():
    rospy.init_node('spawn_task_objects', anonymous=False)

    # Дождаться, пока room_generator выставит точки (он может стартовать
    # чуть позже спавнера). Ждём появления /room/point_B до 20 c.
    t_end = rospy.Time.now() + rospy.Duration(20.0)
    while not rospy.is_shutdown() and not rospy.has_param('/room/point_B'):
        if rospy.Time.now() > t_end:
            rospy.logwarn("room_generator не выставил /room/point_B — беру умолчания")
            break
        rospy.sleep(0.3)

    point_B = rospy.get_param('/room/point_B', rospy.get_param('~point_B', [1.0, 1.0]))
    point_C = rospy.get_param('/room/point_C', rospy.get_param('~point_C', [4.0, 4.0]))

    seed = rospy.get_param('~seed', None)
    if seed is not None:
        random.seed(int(seed))

    forced = rospy.get_param('~cube_color', '')
    if forced in GAZEBO_RGBA:
        cube_color = forced
    else:
        cube_color = random.choice(['red', 'green', 'blue'])

    rospy.loginfo("spawn_task_objects: кубик цвета '%s' в B=%s, картонки в C=%s",
                  cube_color, point_B, point_C)
    # НАМЕРЕННО не публикуем цвет — его должна определить камера.
    rospy.set_param('~ground_truth_cube_color', cube_color)  # только для отладки

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    # Удалить прежние (если перезапуск).
    for nm in ['task_cube', 'pad_red', 'pad_green', 'pad_blue']:
        try:
            delete(nm)
        except Exception:
            pass
    rospy.sleep(0.5)

    def spawn_retry(name, sdf, pose, tries=3):
        for k in range(tries):
            try:
                resp = spawn(name, sdf, '', pose, 'world')
                if getattr(resp, 'success', True):
                    rospy.loginfo("  заспавнен %s", name)
                    return True
                rospy.logwarn("  спавн %s не удался: %s", name,
                              getattr(resp, 'status_message', ''))
            except Exception as e:
                rospy.logwarn("  спавн %s исключение: %s", name, e)
            rospy.sleep(0.5)
        return False

    # Кубик в B — приподнят на 2 см, чтобы аккуратно лечь на пол.
    bx, by = point_B[0], point_B[1]
    spawn_retry('task_cube', cube_sdf('task_cube', GAZEBO_RGBA[cube_color]),
                make_pose(bx, by, 0.06))

    # Три картонки в C.
    cx, cy = point_C[0], point_C[1]
    pads = {
        'red':   (cx, cy - 0.5),
        'green': (cx, cy),
        'blue':  (cx, cy + 0.5),
    }
    for color, (px, py) in pads.items():
        spawn_retry(f'pad_{color}', pad_sdf(f'pad_{color}', GAZEBO_RGBA[color]),
                    make_pose(px, py, 0.0025))

    rospy.loginfo("Объекты расставлены. (ground truth цвет кубика: %s)", cube_color)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
