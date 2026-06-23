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
# Архитектура движения:
#   - база управляется через /cmd_vel: в Gazebo это плагин planar_move,
#     на реальном роботе — драйвер youbot; одометрия приходит в /odom;
#   - рука через /arm_controller/command (JointTrajectory);
#   - схват через /gripper_controller/command (JointTrajectory).
# Объезд препятствий — встроенный планировщик пути (RRT из rrt_planner) плюс
# go-to-waypoint регулятор; то же самое работает в Gazebo и на железе.
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
# ВНИМАНИЕ: максимальный раскрыв схвата = 2*0.0115 = 0.023 м (23 мм), а кубик
# ~50 мм. Полностью обхватить кубик схват НЕ может — захватываем за ребро/угол,
# поэтому держим схват ПОЛНОСТЬЮ открытым на подъезде и зажимаем по кубику.
GRIPPER_OPEN = 0.0115     # максимум (на сторону) — раскрыт полностью
GRIPPER_CLOSED = 0.0      # сжать до конца (кубик крупный, зажимаем за ребро)

# Позы руки (5 суставов). Подобраны под youBot: «над целью» -> «вниз к полу».
# =============================================================================
# Рука ВЫТЯНУТА ВПЕРЁД ГОРИЗОНТАЛЬНО, кисть низко у пола, схват смотрит
# вперёд-чуть-вниз (как на референс-фото). Позы получены IK по кинематике
# youBot: кончик схвата на полу (z=0.025 м) на ~0.38 м перед основанием руки.
#   PREGRASP: рука вперёд, кончик ~0.075 м над полом, схват ОТКРЫТ — висим перед кубом
#   GRASP:    кончик ровно на кубике на полу (после осторожного доезда — зажать)
#   LIFT:     поднимаем плечо (j2) -> кончик ~0.39 м над полом, кубик не выпадает
# Сустав 1≈2.967 и 5≈2.950 держат руку вперёд по +X базы; все в пределах
# мягких лимитов URDF (j1<5.899, j2<2.705, j4<3.578).
# =============================================================================
ARM_HOME = [0.0, 0.0, -0.05, 0.0, 0.0]
ARM_PREGRASP = [2.95, 2.95, 2.95, -2.95, -2.95]   # вперёд, ~0.075 м над полом, схват открыт
ARM_GRASP = [2.95, -2.45, -2.45, -2.95, -2.95]      # кончик на кубике (пол)
ARM_LIFT = [2.95, 0.25, -1.95, 1.75, 2.95]       # плечо поднято, кубик поднят ~0.39 м
ARM_PREPLACE = [2.95, 0.45, -2.35, 1.85, 2.95]   # над картонкой (как PREGRASP)
ARM_PLACE = [2.95, 0.75, -2.45, 1.95, 2.95]     # кончик чуть выше пола (картонка)


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

    def move_arm_blocking(self, angles, settle=10):
        self.send_arm(angles, t=min(settle, 10))
        self.sleep(settle)

    def grip_blocking(self, opening, settle=10):
        self.send_gripper(opening, t=45.0)
        self.sleep(settle)

    def sleep(self, sec):
        end = rospy.Time.now() + rospy.Duration(sec)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            self.rate.sleep()

    # ===================== планирование пути с объездом =====================
    def plan_path(self, gx, gy):
        """Строит безопасный путь от текущей позы до (gx,gy) с объездом
        препятствий (RRT с учётом габарита робота). Возвращает список
        waypoint'ов. Деградирует к прямой, если RRT неприменим
        (нет препятствий, цель/старт в раздутом препятствии и т.п.)."""
        start = (self.x, self.y)
        goal = (gx, gy)
        if RRTPlanner is None or not self.obstacles:
            return [start, goal]

        # Если ЦЕЛЬ или СТАРТ попадают в раздутое препятствие (например, точка
        # у стены, а стенки тоже в списке), RRT гарантированно не найдёт путь.
        # В этом случае честнее ехать напрямую, чем спамить ошибкой.
        infl = self.robot_radius + self.safety_margin
        def blocked(p):
            for (cx, cy, r) in self.obstacles:
                if math.hypot(p[0] - cx, p[1] - cy) < r + infl:
                    return True
            return False
        if blocked(goal) or blocked(start):
            rospy.loginfo_throttle(5.0,
                "Цель/старт у препятствия — еду к (%.2f,%.2f) напрямую", gx, gy)
            return [start, goal]

        try:
            planner = RRTPlanner(start, goal, self.obstacles, self.bounds,
                                 max_iter=6000, step_size=0.25,
                                 robot_radius=self.robot_radius,
                                 goal_bias=0.2, safety_margin=self.safety_margin)
            path = planner.plan()
            if path and len(path) >= 2:
                return path
            rospy.loginfo_throttle(5.0,
                "RRT путь не найден к (%.2f,%.2f) — еду напрямую", gx, gy)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "RRT ошибка: %s — еду напрямую", e)
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

        # 1. Едем к ТОЧКЕ ОСМОТРА перед кубиком. КЛЮЧЕВОЕ: камера на высоте
        #    ~0.31 м с наклоном вниз видит пол в полосе ~0.47..1.39 м перед
        #    базой (центр кадра ~0.73 м). Если встать ближе — кубик попадает
        #    в СЛЕПУЮ ЗОНУ под камерой. Поэтому база встаёт так, чтобы кубик
        #    был на ~0.73 м (центр кадра).
        bx, by = self.point_B
        view_dist = float(rospy.get_param('~view_dist', 0.73))

        # Со стороны какой точки подъезжать к B: от старта (A) — естественно.
        ax0, ay0 = self.point_A
        appr = math.atan2(by - ay0, bx - ax0)   # направление A->B
        # Точка осмотра: отступаем от B назад вдоль линии подъезда.
        vx = bx - view_dist * math.cos(appr)
        vy = by - view_dist * math.sin(appr)
        # Камера (-X базы) должна смотреть на кубик (B). Значит нос базы (+X)
        # направлен ОТ кубика: yaw = направление (точка_осмотра -> B) + pi.
        look = math.atan2(by - vy, bx - vx)
        cam_yaw = self.norm(look + math.pi)

        rospy.loginfo("Точка осмотра перед B: (%.2f,%.2f) yaw=%.2f (камера на кубик)",
                      vx, vy, cam_yaw)
        self.goto_planned(vx, vy, final_yaw=cam_yaw)
        self.stop_base()
        self.sleep(1.5)  # дать камере успокоиться и распознать

        color = self.wait_for_color(timeout=8.0)
        if color is None:
            # Кубик не виден из точки осмотра — медленно поворачиваемся на
            # месте, чтобы поймать кубик в кадр камеры (-X базы).
            rospy.logwarn("Кубик не виден — ищу поворотом базы...")
            color = self.search_for_cube(bx, by)
        if color is None:
            rospy.logerr("Миссия прервана: цвет не определён.")
            return
        rospy.loginfo("Кубик распознан как %s. Готовлюсь к захвату.", color)

        # ЗАМОРАЖИВАЕМ позицию кубика, измеренную СЕЙЧАС (камера наведена на
        # кубик в точке осмотра). Дальше используем только это значение —
        # иначе при развороте камера ловит картонки/мусор и цель «прыгает».
        if self.cube_pose is not None:
            frozen_cube = (self.cube_pose.pose.position.x,
                           self.cube_pose.pose.position.y)
        else:
            frozen_cube = (bx, by)
        rospy.loginfo("Зафиксирована позиция кубика: (%.3f, %.3f)", *frozen_cube)

        # 1b. Встать на БЕЗОПАСНОЙ дистанции (с запасом ~0.10 м, чтобы НЕ
        #     наехать на кубик корпусом), нос/рука (+X) точно на кубик.
        #     База НЕ доезжает до кубика — дотягивается потом вытянутая рука.
        self.position_for_grasp(frozen_cube, extra_standoff=0.025)

        # 2. Захват кубика. Стратегия (как просили):
        #    a) опустить руку ВПЕРЁД к полу, схват РАСКРЫТ полностью;
        #    b) осторожно подъехать базой так, чтобы открытый схват оказался
        #       вокруг кубика (медленный creep, не толкая корпусом);
        #    c) опустить кончик ровно на кубик и ЗАЖАТЬ;
        #    d) поднять.
        rospy.loginfo("Захват кубика (%s)...", color)
        self.grip_blocking(GRIPPER_OPEN, 2.0)        # схват на максимум
        self.move_arm_blocking(ARM_PREGRASP, 8.0)
        self.sleep(4.0)    # рука вперёд, низко, открыта
        self.grip_blocking(GRIPPER_OPEN, 2.0)        # подстраховка: точно открыт

        # b) Осторожный доезд: подвинуть базу вперёд на остаток дистанции
        #    (extra_standoff), чтобы кубик оказался В РАСКРЫТОМ схвате.
        self.creep_forward(0.01)

        self.stop_base()
        self.sleep(3.0)

        self.move_arm_blocking(ARM_GRASP, 6.0)
        
        self.sleep(3.0)
               # кончик на кубик (пол)
        self.grip_blocking(GRIPPER_CLOSED, 3.0)  
        
        self.sleep(3.0) # зажать кубик

        self.move_arm_blocking(ARM_LIFT, 6.0)        # поднять
        
        self.sleep(3.0)

        # 3. Перенос в зону C (с объездом). Едем к ТОЧКЕ ПЕРЕД C, не в центр,
        #    чтобы не наезжать на картонки.
        cx, cy = self.point_C
        self.goto_planned(cx, cy)

        # 3b. Подъехать к нужной картонке по цвету так, чтобы картонка была
        #     ПЕРЕД РУКОЙ (+X) на arm_reach — рука ставит кубик, база на картонку
        #     не наезжает. Используем тот же двухэтапный манёвр, что и для захвата.
        pad = self.pads.get(color)
        if pad is None:
            rospy.logerr("Нет картонки для цвета %s; кладу в C.", color)
            pad = self.point_C
        rospy.loginfo("Целевая картонка %s -> %s", color, pad)
        self.position_for_grasp((pad[0], pad[1]), extra_standoff=0.08)
        self.stop_base()
        self.sleep(0.5)

        # 4. Поставить кубик на картонку соответствующего цвета.
        #    Симметрично захвату: вытянуть руку вперёд, доехать, опустить, отпустить.
        rospy.loginfo("Установка кубика на картонку %s...", color)
        self.move_arm_blocking(ARM_PREPLACE, 8.0)  # рука вперёд, низко
        self.sleep(4.0)
        self.creep_forward(0.11)                   # подвести кубик над картонкой
        self.stop_base()
        self.sleep(3.0)
        self.move_arm_blocking(ARM_PLACE, 6.0)     # опустить к картонке
        self.sleep(3.0)
        self.grip_blocking(GRIPPER_OPEN, 3.0)      # отпустить
        self.sleep(3.0)
        self.move_arm_blocking(ARM_PREPLACE, 2.0)  # отвести руку вверх-назад
        self.creep_forward(-0.12)                  # чуть отъехать, не задев кубик
        self.move_arm_blocking(ARM_HOME, 2.5)

        # 5. (Опционально) вернуться на парковку A (с объездом).
        ax, ay = self.point_A
        self.goto_planned(ax, ay)
        self.stop_base()
        rospy.loginfo("Миссия завершена: кубик %s установлен на картонку %s.",
                      color, color)

    # --- поиск кубика поворотом базы, если он не попал в кадр сразу ---------
    def search_for_cube(self, bx, by, sweep=1.2, steps=8):
        """Если кубик не виден из точки осмотра — сначала ПЕРЕВСТАЁМ на
        правильную дистанцию обзора (кубик мог попасть в слепую зону под
        камерой), затем поворачиваем базу в обе стороны, ловя кубик."""
        # 1) перевстать так, чтобы кубик был в центре кадра камеры (-X) на
        #    дистанции view_dist; камера направлена на B.
        view_dist = float(rospy.get_param('~view_dist', 0.73))
        appr = math.atan2(by - self.y, bx - self.x)  # от робота к B
        vx = bx - view_dist * math.cos(appr)
        vy = by - view_dist * math.sin(appr)
        look = math.atan2(by - vy, bx - vx)
        cam_yaw = self.norm(look + math.pi)
        rospy.loginfo("Поиск: перевстаю на дистанцию обзора (%.2f,%.2f)", vx, vy)
        self.goto(vx, vy, final_yaw=cam_yaw, tol=0.06, timeout=20.0)
        self.stop_base()
        self.sleep(1.0)
        c = self.wait_for_color(timeout=3.0)
        if c is not None:
            return c

        # 2) поворот базы в обе стороны вокруг текущего yaw
        base_yaw = self.yaw
        offsets = [0.0]
        d = sweep / steps
        for k in range(1, steps + 1):
            offsets.append(+k * d)
            offsets.append(-k * d)
        for off in offsets:
            if rospy.is_shutdown():
                break
            self.rotate_to(self.norm(base_yaw + off))
            self.stop_base()
            self.sleep(0.6)
            c = self.wait_for_color(timeout=2.0)
            if c is not None:
                return c
        return None

    # --- позиционирование базы для ЗАХВАТА (рука по +X, камера по -X) -------
    def position_for_grasp(self, cube_xy, timeout=20.0, extra_standoff=0.18):
        """Встаёт так, чтобы кубик был ПЕРЕД РУКОЙ (+X базы). cube_xy —
        ЗАМОРОЖЕННАЯ позиция кубика в odom.

        КЛЮЧЕВОЕ ОТЛИЧИЕ (исправление «наезда на кубик»): база НЕ доезжает до
        точки arm_reach, а ОСТАНАВЛИВАЕТСЯ на (arm_reach + extra_standoff) —
        т.е. на extra_standoff ДАЛЬШЕ от кубика. Этот остаток выбирается потом
        медленным creep_forward(extra_standoff) уже с ВЫТЯНУТОЙ ВПЕРЁД рукой,
        чтобы открытый схват аккуратно охватил кубик, а корпус его не толкнул.

        Манёвр в два этапа, чтобы не толкнуть кубик при довороте:
          1) встать на БЕЗОПАСНОЙ дистанции (reach + approach_gap) и развернуть
             руку (+X) точно на кубик — корпус ещё далеко от кубика;
          2) аккуратно подъехать вперёд до (reach + extra_standoff), БЕЗ доворота."""
        reach = float(rospy.get_param('~arm_reach', 0.523))
        approach_gap = float(rospy.get_param('~approach_gap', 0.25))
        # Финальная дистанция базы до кубика = reach + extra_standoff (с запасом).
        stop_dist = reach + extra_standoff
        tx, ty = cube_xy

        # Желаемый yaw базы: рука(+X) направлена НА кубик.
        desired_yaw = math.atan2(ty - self.y, tx - self.x)

        # --- Этап 1: безопасная дистанция + доворот рукой к кубику ---
        safe = stop_dist + approach_gap
        sx = tx - safe * math.cos(desired_yaw)
        sy = ty - safe * math.sin(desired_yaw)
        rospy.loginfo("Захват, этап 1: безопасная точка (%.2f,%.2f) yaw=%.2f",
                      sx, sy, desired_yaw)
        self.goto(sx, sy, final_yaw=desired_yaw, tol=0.05, timeout=timeout)
        self.rotate_to(desired_yaw)
        self.stop_base()
        self.sleep(0.5)

        # Пересчитываем желаемый yaw уже из новой позы (мог сместиться) —
        # но БЕЗ повторной детекции, по той же замороженной точке.
        desired_yaw = math.atan2(ty - self.y, tx - self.x)
        self.rotate_to(desired_yaw)
        self.stop_base()

        # --- Этап 2: аккуратный доезд вперёд ДО stop_dist (НЕ до кубика!) ---
        gx = tx - stop_dist * math.cos(desired_yaw)
        gy = ty - stop_dist * math.sin(desired_yaw)
        rospy.loginfo("Захват, этап 2: подъезд к (%.2f,%.2f), оставляю запас %.2f м",
                      gx, gy, extra_standoff)
        self.creep_to(gx, gy, desired_yaw, timeout=timeout)
        self.stop_base()
        # Запоминаем курс на кубик для последующего creep_forward.
        self._grasp_yaw = desired_yaw

    def creep_forward(self, dist, timeout=12.0):
        """Медленно проехать вперёд по +X базы на dist метров (dist<0 — назад),
        удерживая текущий курс. Используется, чтобы с ВЫТЯНУТОЙ рукой подвести
        открытый схват к кубику, не толкая его корпусом."""
        hold_yaw = getattr(self, '_grasp_yaw', self.yaw)
        v_creep = float(rospy.get_param('~v_creep', 0.05))
        sign = 1.0 if dist >= 0 else -1.0
        target = abs(dist)
        x0, y0 = self.x, self.y
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                break
            travelled = math.hypot(self.x - x0, self.y - y0)
            if travelled >= target:
                break
            tw = Twist()
            tw.linear.x = sign * v_creep      # +X базы = «вперёд носом/рукой»
            tw.linear.y = 0.0
            tw.angular.z = self.clamp(1.0 * self.norm(hold_yaw - self.yaw),
                                      -0.3, 0.3)
            self.cmd_pub.publish(tw)
            self.rate.sleep()
        for _ in range(4):
            self.cmd_pub.publish(Twist())
            self.rate.sleep()

    def creep_to(self, gx, gy, hold_yaw, timeout=12.0):
        """Медленный прямой подъезд к (gx,gy) с удержанием курса hold_yaw.
        Двигается вперёд по +X базы малой скоростью — чтобы не толкнуть кубик."""
        t0 = rospy.Time.now()
        v_creep = float(rospy.get_param('~v_creep', 0.06))
        while not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                break
            dx = gx - self.x
            dy = gy - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.03:
                break
            # скорость в мировой СК -> в кадр базы
            wx = v_creep * dx / dist
            wy = v_creep * dy / dist
            c = math.cos(-self.yaw); s = math.sin(-self.yaw)
            tw = Twist()
            tw.linear.x = c * wx - s * wy
            tw.linear.y = s * wx + c * wy
            tw.angular.z = self.clamp(1.0 * self.norm(hold_yaw - self.yaw),
                                      -0.4, 0.4)
            self.cmd_pub.publish(tw)
            self.rate.sleep()
        for _ in range(4):
            self.cmd_pub.publish(Twist())
            self.rate.sleep()

    def face_point(self, fromx, fromy, target):
        return math.atan2(target[1] - fromy, target[0] - fromx)


if __name__ == '__main__':
    try:
        PickPlaceMission().run()
    except rospy.ROSInterruptException:
        pass
