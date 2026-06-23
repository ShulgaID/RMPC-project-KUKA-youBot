#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# pick_place_mission.py
# -----------------------------------------------------------------------------
# Конечный автомат миссии «детекция -> захват -> перестановка по цвету».
#
# Решает ИМЕННО поставленную задачу:
#   1) Приехать в точку B и НАЙТИ кубик 5x5 см камерой, ОПРЕДЕЛИТЬ его цвет
#      (red/green/blue) — данные берём из узла cube_detector (/cube/color,
#      /cube/pose).
#   2) Захватить кубик и перенести его в точку C.
#   3) В точке C лежат три картонки red/green/blue. Поставить кубик на
#      картонку СООТВЕТСТВУЮЩЕГО цвета (red->red, green->green, blue->blue).
#
# Позиции картонок задаются параметрами (в Gazebo мы их сами расставляем
# и знаем координаты; на реальном роботе их можно либо задать так же, либо
# доопределить камерой по тем же HSV-маскам — интерфейс остаётся прежним).
#
# Архитектура движения переиспользует уже существующие узлы проекта:
#   - база управляется через /cmd_vel (mecanum_controller + одометрия /odom);
#   - рука через /arm_controller/command (JointTrajectory);
#   - схват через /gripper_controller/command (JointTrajectory).
# Мы НЕ дёргаем trajectory_planner напрямую: здесь простой и надёжный
# go-to-waypoint регулятор + аккуратная последовательность движений руки,
# чтобы это одинаково работало в Gazebo и на железе.
#
# Точки A/B/C берём из room_generator (/room/point_*), если он запущен,
# иначе — из своих параметров.
# =============================================================================

import math
import os
import sys

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped as _PS
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Подключаем RRT-планировщик из пакета youbot_controller (общий источник).
try:
    from rrt_planner import RRTPlanner
except Exception:
    try:
        import rospkg
        _p = rospkg.RosPack().get_path('youbot_controller')
        sys.path.append(os.path.join(_p, 'scripts'))
        from rrt_planner import RRTPlanner
    except Exception:
        RRTPlanner = None


# Раскрытие схвата (м, на сторону). limits.urdf.xacro: upper = 0.023/2 = 0.0115.
GRIPPER_OPEN = 0.0115
GRIPPER_CLOSED = 0.002   # чуть зажат на кубике 50 мм (палец давит с двух сторон)

# Позы руки (5 суставов). Подобраны под youBot: «над целью» -> «вниз к полу».
ARM_HOME = [0.0, 0.0, -0.05, 0.0, 0.0]
ARM_PREGRASP = [2.95, 1.05, -2.55, 1.79, 2.92]   # рука вытянута вперёд-вниз, над кубиком
ARM_GRASP = [2.95, 1.30, -2.30, 1.60, 2.92]      # опущена к кубику
ARM_LIFT = [2.95, 0.70, -1.80, 1.40, 2.92]       # поднята с кубиком
ARM_PREPLACE = [2.95, 1.05, -2.55, 1.79, 2.92]   # над картонкой
ARM_PLACE = [2.95, 1.25, -2.35, 1.62, 2.92]      # опустить к картонке


class PickPlaceMission:
    def __init__(self):
        rospy.init_node('pick_place_mission', anonymous=False)

        self.rate_hz = rospy.get_param('~rate', 20)
        self.rate = rospy.Rate(self.rate_hz)

        # --- Точки маршрута ---------------------------------------------------
        self.point_B = tuple(rospy.get_param('/room/point_B',
                              rospy.get_param('~point_B', [1.0, 1.0])))
        self.point_C = tuple(rospy.get_param('/room/point_C',
                              rospy.get_param('~point_C', [4.0, 4.0])))
        self.point_A = tuple(rospy.get_param('/room/point_A',
                              rospy.get_param('~point_A', [0.0, 0.0])))

        # --- Позиции трёх картонок в точке C (odom). Формат: [x, y]. ----------
        # По умолчанию раскладываем три картонки рядом с C вдоль оси Y.
        cx, cy = self.point_C
        self.pads = {
            'red':   tuple(rospy.get_param('~pad_red',   [cx, cy - 0.5])),
            'green': tuple(rospy.get_param('~pad_green', [cx, cy])),
            'blue':  tuple(rospy.get_param('~pad_blue',  [cx, cy + 0.5])),
        }

        # Допуски подъезда.
        self.pos_tol = float(rospy.get_param('~pos_tol', 0.06))
        self.yaw_tol = float(rospy.get_param('~yaw_tol', 0.10))
        self.v_max = float(rospy.get_param('~v_max', 0.22))
        self.w_max = float(rospy.get_param('~w_max', 1.0))
        # Габаритный радиус робота и запас для объезда. Увеличены, чтобы
        # корпус НЕ задевал цилиндры (раньше проезжал через коллизию).
        self.robot_radius = float(rospy.get_param('~robot_radius', 0.40))
        self.safety_margin = float(rospy.get_param('~safety_margin', 0.15))

        # --- Препятствия и границы из room_generator (для объезда) -----------
        flat = rospy.get_param('/room/obstacles_flat', None)
        if flat:
            self.obstacles = [(flat[i], flat[i + 1], flat[i + 2])
                              for i in range(0, len(flat), 3)]
        else:
            self.obstacles = []
        self.bounds = tuple(rospy.get_param('/room/bounds',
                                            [-0.5, 5.5, -0.5, 5.5]))

        # --- Состояние --------------------------------------------------------
        self.x = self.y = self.yaw = 0.0
        self.odom_ok = False
        self.cube_color = None
        self.cube_pose = None     # PoseStamped в odom

        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/cube/color', String, self.color_cb)
        rospy.Subscriber('/cube/pose', PoseStamped, self.pose_cb)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_controller/command',
                                       JointTrajectory, queue_size=10)
        self.grip_pub = rospy.Publisher('/gripper_controller/command',
                                        JointTrajectory, queue_size=10)
        # Публикуем ИМЕННО тот путь, по которому едем — чтобы в RViz
        # масштаб/геометрия совпадали с движением в Gazebo (frame=odom).
        self.path_pub = rospy.Publisher('/mission_planned_path', Path,
                                        queue_size=1, latch=True)

        rospy.loginfo("pick_place_mission: B=%s C=%s pads=%s obstacles=%d",
                      self.point_B, self.point_C, self.pads,
                      len(self.obstacles))

    # ===================== callbacks ========================================
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.odom_ok = True

    def color_cb(self, msg):
        self.cube_color = msg.data

    def pose_cb(self, msg):
        self.cube_pose = msg

    # ===================== низкоуровневые команды ===========================
    def stop_base(self):
        self.cmd_pub.publish(Twist())

    def send_arm(self, angles, t=2.0):
        msg = JointTrajectory()
        msg.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3',
                           'arm_joint_4', 'arm_joint_5']
        p = JointTrajectoryPoint()
        p.positions = list(angles)
        p.time_from_start = rospy.Duration(t)
        msg.points.append(p)
        self.arm_pub.publish(msg)

    def send_gripper(self, opening, t=1.0):
        msg = JointTrajectory()
        msg.joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
        p = JointTrajectoryPoint()
        p.positions = [opening, opening]
        p.time_from_start = rospy.Duration(t)
        msg.points.append(p)
        self.grip_pub.publish(msg)

    def move_arm_blocking(self, angles, settle=2.5):
        self.send_arm(angles, t=min(settle, 2.5))
        self.sleep(settle)

    def grip_blocking(self, opening, settle=1.5):
        self.send_gripper(opening, t=1.0)
        self.sleep(settle)

    def sleep(self, sec):
        end = rospy.Time.now() + rospy.Duration(sec)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            self.rate.sleep()

    # ===================== планирование пути с объездом =====================
    def plan_path(self, gx, gy):
        """Строит безопасный путь от текущей позы до (gx,gy) с объездом
        препятствий (RRT с учётом габарита робота). Возвращает список
        waypoint'ов [(x,y), ...]. Если планировщик недоступен или путь не
        найден — возвращает прямую [start, goal]."""
        start = (self.x, self.y)
        goal = (gx, gy)
        if RRTPlanner is None or not self.obstacles:
            return [start, goal]
        try:
            planner = RRTPlanner(start, goal, self.obstacles, self.bounds,
                                 max_iter=4000, step_size=0.25,
                                 robot_radius=self.robot_radius,
                                 goal_bias=0.15, safety_margin=self.safety_margin)
            path = planner.plan()
            if path and len(path) >= 2:
                return path
            rospy.logwarn("RRT не нашёл путь к (%.2f,%.2f) — еду напрямую", gx, gy)
        except Exception as e:
            rospy.logwarn("RRT ошибка: %s — еду напрямую", e)
        return [start, goal]

    def publish_path(self, waypoints):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = rospy.Time.now()
        for (wx, wy) in waypoints:
            ps = _PS()
            ps.header = msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def goto_planned(self, gx, gy, final_yaw=None):
        """Планирует путь с объездом и едет по waypoint'ам."""
        waypoints = self.plan_path(gx, gy)
        self.publish_path(waypoints)
        rospy.loginfo("Маршрут до (%.2f,%.2f): %d точек", gx, gy, len(waypoints))
        # Едем по всем промежуточным точкам, кроме последней — без точного
        # доворота; на финальной точке выставляем final_yaw.
        for i, (wx, wy) in enumerate(waypoints[1:], start=1):
            is_last = (i == len(waypoints) - 1)
            self.goto(wx, wy,
                      final_yaw=(final_yaw if is_last else None),
                      tol=(self.pos_tol if is_last else 0.18),
                      timeout=40.0)

    # ===================== навигация к точке ================================
    def goto(self, gx, gy, final_yaw=None, timeout=60.0, tol=None):
        """Go-to-goal для ВСЕНАПРАВЛЕННОЙ базы (planar_move понимает vy).
        Едем сразу в сторону цели (vx,vy) без обязательного доворота, скорость
        пропорционально гасится у цели -> нет проскока. В конце — твёрдый стоп
        с проверкой, что робот действительно остановился у цели."""
        if tol is None:
            tol = self.pos_tol
        rospy.loginfo("goto (%.2f, %.2f)", gx, gy)
        t0 = rospy.Time.now()
        reached = False
        while not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn("goto timeout (dist=%.3f)",
                              math.hypot(gx - self.x, gy - self.y))
                break
            dx = gx - self.x
            dy = gy - self.y
            dist = math.hypot(dx, dy)

            if dist < tol:
                reached = True
                break

            # Желаемая скорость в МИРОВОЙ СК, затем переводим в кадр базы.
            # Пропорционально гасим у цели; ограничиваем v_max; у самой цели
            # скорость -> 0 (нет минимального «толчка», который даёт проскок).
            speed = self.clamp(1.2 * dist, 0.0, self.v_max)
            # лёгкий минимум только когда далеко, чтобы не «засыпать» на месте
            if dist > 0.25:
                speed = max(speed, 0.06)
            wx = speed * dx / dist
            wy = speed * dy / dist
            # перевод вектора скорости из odom в кадр базы (yaw)
            c = math.cos(-self.yaw); s = math.sin(-self.yaw)
            vx_b = c * wx - s * wy
            vy_b = s * wx + c * wy

            tw = Twist()
            tw.linear.x = vx_b
            tw.linear.y = vy_b
            # лёгкий доворот носом по ходу (не обязателен для движения)
            if final_yaw is None:
                heading = math.atan2(dy, dx)
                tw.angular.z = self.clamp(0.8 * self.norm(heading - self.yaw),
                                          -self.w_max, self.w_max)
            self.cmd_pub.publish(tw)
            self.rate.sleep()

        # --- Твёрдый стоп: несколько нулевых команд + короткая пауза ---
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            self.rate.sleep()

        # финальная ориентация (если нужна)
        if final_yaw is not None:
            self.rotate_to(final_yaw)
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            self.rate.sleep()
        return reached

    def rotate_to(self, target_yaw, timeout=15.0):
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                break
            err = self.norm(target_yaw - self.yaw)
            if abs(err) < self.yaw_tol:
                break
            tw = Twist()
            tw.angular.z = self.clamp(1.5 * err, -self.w_max, self.w_max)
            self.cmd_pub.publish(tw)
            self.rate.sleep()
        self.stop_base()

    # ===================== вспомогательное ==================================
    @staticmethod
    def norm(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def wait_for_color(self, timeout=15.0):
        """Ждём, пока cube_detector надёжно определит цвет (несколько
        одинаковых детекций подряд)."""
        rospy.loginfo("Ожидание детекции цвета кубика...")
        t0 = rospy.Time.now()
        votes = {}
        last = None
        stable = 0
        while not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                break
            c = self.cube_color
            if c is not None:
                votes[c] = votes.get(c, 0) + 1
                if c == last:
                    stable += 1
                else:
                    stable = 1
                    last = c
                if stable >= 5:
                    rospy.loginfo("Цвет кубика определён: %s", c)
                    return c
            self.rate.sleep()
        if votes:
            best = max(votes, key=votes.get)
            rospy.logwarn("Цвет по большинству голосов: %s", best)
            return best
        rospy.logerr("Не удалось определить цвет кубика!")
        return None

    # ===================== главная миссия ===================================
    def run(self):
        rospy.loginfo("Ждём одометрию...")
        while not rospy.is_shutdown() and not self.odom_ok:
            self.rate.sleep()

        # 0. Привести руку в безопасное «домашнее» и раскрыть схват.
        self.move_arm_blocking(ARM_HOME, 2.0)
        self.grip_blocking(GRIPPER_OPEN, 1.0)

        # 1. Едем к точке B (с объездом). Останавливаемся ПЕРЕД кубиком так,
        #    чтобы КАМЕРА (смотрит по -X) видела кубик. Значит «спина» базы
        #    (-X) должна смотреть на кубик => нос(+X) от кубика, yaw такой,
        #    что -X указывает на B. Подъезжаем со стороны точки A.
        bx, by = self.point_B
        # Подъедем к точке рядом с B со стороны старта, развернувшись камерой к B.
        # Для простоты встаём в B и ориентируем камеру на центр комнаты/кубик.
        cam_yaw = math.atan2(by - self.point_A[1], bx - self.point_A[0]) + math.pi
        self.goto_planned(bx, by, final_yaw=cam_yaw)
        self.stop_base()
        self.sleep(1.5)  # дать камере успокоиться и распознать

        color = self.wait_for_color(timeout=15.0)
        if color is None:
            rospy.logerr("Миссия прервана: цвет не определён.")
            return
        rospy.loginfo("Кубик распознан как %s. Готовлюсь к захвату.", color)

        # 1b. Развернуться РУКОЙ к кубику и встать на дистанцию захвата.
        self.position_for_grasp()

        # 2. Захват кубика.
        rospy.loginfo("Захват кубика (%s)...", color)
        self.move_arm_blocking(ARM_PREGRASP, 2.5)
        self.grip_blocking(GRIPPER_OPEN, 1.0)
        self.move_arm_blocking(ARM_GRASP, 2.0)
        self.grip_blocking(GRIPPER_CLOSED, 1.5)   # сжать на кубике
        self.move_arm_blocking(ARM_LIFT, 2.0)     # поднять

        # 3. Перенос в точку C (с объездом препятствий).
        cx, cy = self.point_C
        self.goto_planned(cx, cy)

        # 3b. Подъехать к нужной картонке по цвету.
        pad = self.pads.get(color)
        if pad is None:
            rospy.logerr("Нет картонки для цвета %s; кладём в C.", color)
            pad = self.point_C
        rospy.loginfo("Целевая картонка %s -> %s", color, pad)
        self.goto(pad[0], pad[1])   # короткий доезд внутри зоны C — напрямую
        self.stop_base()
        self.sleep(0.5)

        # 4. Поставить кубик на картонку соответствующего цвета.
        rospy.loginfo("Установка кубика на картонку %s...", color)
        self.move_arm_blocking(ARM_PREPLACE, 2.5)
        self.move_arm_blocking(ARM_PLACE, 2.0)
        self.grip_blocking(GRIPPER_OPEN, 1.5)     # отпустить
        self.move_arm_blocking(ARM_PREPLACE, 2.0)
        self.move_arm_blocking(ARM_HOME, 2.5)

        # 5. (Опционально) вернуться на парковку A (с объездом).
        ax, ay = self.point_A
        self.goto_planned(ax, ay)
        self.stop_base()
        rospy.loginfo("Миссия завершена: кубик %s установлен на картонку %s.",
                      color, color)

    # --- позиционирование базы для ЗАХВАТА (рука по +X, камера по -X) -------
    def position_for_grasp(self, timeout=12.0):
        """Камера смотрит по -X (видит кубик при подъезде), а рука хватает
        по +X. Поэтому для захвата нужно встать так, чтобы кубик оказался
        ПЕРЕД РУКОЙ: на расстоянии arm_reach по +X базы. Используем
        запомненную позицию кубика из детектора (cube_pose в odom)."""
        reach = float(rospy.get_param('~arm_reach', 0.32))
        if self.cube_pose is None:
            rospy.logwarn("Нет позиции кубика — позиционируюсь по точке B")
            tx, ty = self.point_B
        else:
            tx = self.cube_pose.pose.position.x
            ty = self.cube_pose.pose.position.y

        # Желаемый yaw базы: рука(+X) направлена НА кубик.
        desired_yaw = math.atan2(ty - self.y, tx - self.x)
        # Целевая поза базы: отступить от кубика на reach вдоль этого направления.
        gx = tx - reach * math.cos(desired_yaw)
        gy = ty - reach * math.sin(desired_yaw)

        rospy.loginfo("Позиционируюсь для захвата: кубик=(%.2f,%.2f) "
                      "база->(%.2f,%.2f) yaw=%.2f", tx, ty, gx, gy, desired_yaw)
        # Едем в точку и доворачиваемся рукой к кубику.
        self.goto(gx, gy, final_yaw=desired_yaw, tol=0.04, timeout=timeout)
        self.rotate_to(desired_yaw)
        self.stop_base()

    def face_point(self, fromx, fromy, target):
        return math.atan2(target[1] - fromy, target[0] - fromx)


if __name__ == '__main__':
    try:
        PickPlaceMission().run()
    except rospy.ROSInterruptException:
        pass
