#!/usr/bin/env python3
# trajectory_planner.py – полная версия с RRT и сплайнами

import math
import numpy as np
# (CubicSpline больше не нужен: безопасный путь берём из RRT)
from rrt_planner import RRTPlanner
import rospy

class TrajectoryPlanner:
    def __init__(self):
        # ----------------------------------------------------------------
        # Данные о комнате берём из ноды room_generator (через ROS-параметры).
        # Это единственный источник истины: точки A/B/C, препятствия, границы.
        # Если параметров нет (нода не запущена) — используем безопасные
        # умолчания, совместимые со старым поведением (область 5x5).
        # ----------------------------------------------------------------
        # B — точка кубика (pick), C — точка склада (place), A — парковка/home.
        self.object_pos = tuple(rospy.get_param('/room/point_B', [1.0, 1.0]))  # B: pick
        self.marker_pos = tuple(rospy.get_param('/room/point_C', [4.0, 4.0]))  # C: place
        # A (парковка) — это home; если задана в room, используем её,
        # иначе home определится по стартовой одометрии в init_mission.
        park = rospy.get_param('/room/point_A', None)
        self.park_pos = tuple(park) if park is not None else None
        self.home_pos = None

        # Препятствия (стенки + виртуальные цилиндры) из room_generator.
        flat = rospy.get_param('/room/obstacles_flat', None)
        if flat:
            self.obstacles = [(flat[i], flat[i + 1], flat[i + 2])
                              for i in range(0, len(flat), 3)]
        else:
            # Fallback — старый жёстко заданный набор.
            self.obstacles = [
                (1.5, 2.0, 0.4),
                (2.5, 1.5, 0.35),
                (3.0, 3.0, 0.45),
                (2.0, 3.5, 0.3),
                (3.5, 2.5, 0.25)
            ]
        self.bounds = tuple(rospy.get_param('/room/bounds', [-0.5, 4.5, -0.5, 4.5]))

        # Временные параметры
        self.time_approach = 4.0
        self.time_grasp = 2.0
        self.time_move = 8.0
        self.time_place = 2.0
        self.time_return = 5.0
        self.total_time = self.time_approach + self.time_grasp + self.time_move + self.time_place + self.time_return

        # Углы руки
        self.home_angles = [0.01, 0.01, -0.1, 0.01, 0.01]
        self.grasp_angles = [0.5, 0.8, -1.2, 0.3, 0.0]
        self.lift_angles = [0.5, 0.8, -0.5, 0.8, 0.0]
        self.place_angles = [0.5, 0.8, -0.2, -0.5, 0.0]

        # Пути и сплайны
        self.path_approach = []
        self.path_move = []
        self.path_return = []
        self.spline_approach = None
        self.spline_move = None
        self.spline_return = None
        self.start_pose = None
        self.current_phase = 'INIT'

    def init_mission(self, start_pose):
        rospy.loginfo("=== init_mission called with start_pose: (%.2f, %.2f, %.2f) ===", *start_pose)
        self.start_pose = start_pose
        # home (точка A / парковка): берём заданную точку A из room_generator,
        # иначе — стартовую позу робота.
        if self.park_pos is not None:
            self.home_pos = self.park_pos
        else:
            self.home_pos = (start_pose[0], start_pose[1])

        rospy.loginfo("Планирование путей RRT...")
        self.path_approach = self._plan_path(self.home_pos, self.object_pos)
        self.path_move = self._plan_path(self.object_pos, self.marker_pos)
        self.path_return = self._plan_path(self.marker_pos, self.home_pos)

        rospy.loginfo("Paths lengths: approach=%d, move=%d, return=%d", len(self.path_approach), len(self.path_move), len(self.path_return))
        self.spline_approach = self._create_spline_or_none(self.path_approach)
        self.spline_move = self._create_spline_or_none(self.path_move)
        self.spline_return = self._create_spline_or_none(self.path_return)

        # Длительности фаз ПРОПОРЦИОНАЛЬНЫ длине пути при заданной средней
        # скорости. Иначе на длинном объездном пути робот «спешит» и срезает.
        v_avg = float(rospy.get_param('~v_avg', 0.22))   # м/с, средняя скорость
        self.time_approach = max(3.0, self._path_len(self.path_approach) / v_avg)
        self.time_move = max(3.0, self._path_len(self.path_move) / v_avg)
        self.time_return = max(3.0, self._path_len(self.path_return) / v_avg)
        self.total_time = (self.time_approach + self.time_grasp +
                           self.time_move + self.time_place + self.time_return)
        rospy.loginfo("Длительности фаз: approach=%.1f move=%.1f return=%.1f c",
                      self.time_approach, self.time_move, self.time_return)

        if self.spline_approach is None:
            rospy.logwarn("RRT не нашёл путь к объекту, используется прямая линия")
        if self.spline_move is None:
            rospy.logwarn("RRT не нашёл путь к маркеру, используется прямая линия")
        if self.spline_return is None:
            rospy.logwarn("RRT не нашёл путь обратно в home, используется прямая линия")
        rospy.loginfo("Планирование завершено.")

    @staticmethod
    def _path_len(points):
        if not points or len(points) < 2:
            return 0.0
        return sum(math.hypot(points[i + 1][0] - points[i][0],
                              points[i + 1][1] - points[i][1])
                   for i in range(len(points) - 1))

    def _plan_path(self, start, goal):
        planner = RRTPlanner(start, goal, self.obstacles, self.bounds,
                             max_iter=4000, step_size=0.25,
                             robot_radius=rospy.get_param('~robot_radius', 0.32),
                             goal_bias=0.15, safety_margin=0.05)
        return planner.plan()

    def _create_spline_or_none(self, points):
        # ВАЖНО: раньше здесь строился кубический сплайн через все узлы RRT.
        # Сплайн «срезал углы» и выгибался ВНУТРЬ препятствий — робот их задевал.
        # Теперь мы НЕ искажаем геометрию: возвращаем сам безопасный путь RRT
        # (после shortcut он уже короткий и проверен на коллизии), равномерно
        # пересемплированный по длине дуги. _get_pose_from_spline ниже умеет
        # работать с таким представлением как с кусочно-линейным путём.
        if points and len(points) > 1:
            return ('polyline', self._resample_polyline(points))
        return None

    @staticmethod
    def _resample_polyline(points, n=100):
        """Равномерно по длине дуги раскладывает ломаную на n+1 точек."""
        import numpy as np
        pts = np.array(points, dtype=float)
        seg = np.sqrt(((pts[1:] - pts[:-1]) ** 2).sum(axis=1))
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        total = cum[-1] if cum[-1] > 1e-9 else 1.0
        ts = cum / total
        out = []
        for s in np.linspace(0, 1, n + 1):
            # найти сегмент
            k = int(np.searchsorted(ts, s, side='right')) - 1
            k = max(0, min(k, len(pts) - 2))
            t0, t1 = ts[k], ts[k + 1]
            local = 0.0 if t1 <= t0 else (s - t0) / (t1 - t0)
            p = pts[k] + local * (pts[k + 1] - pts[k])
            out.append((float(p[0]), float(p[1])))
        return out

    def _interpolate_angles(self, start_angles, end_angles, progress):
        return [start_angles[i] + progress * (end_angles[i] - start_angles[i]) for i in range(len(start_angles))]

    def _get_pose_from_spline(self, spline, t, default_start, default_end):
        if default_start is None or default_end is None:
            return 0.0, 0.0, 0.0
        if spline is not None and isinstance(spline, tuple) and spline[0] == 'polyline':
            pts = spline[1]
            t = max(0.0, min(1.0, t))
            idx = t * (len(pts) - 1)
            i = int(idx)
            i = max(0, min(i, len(pts) - 2))
            local = idx - i
            x = pts[i][0] + local * (pts[i + 1][0] - pts[i][0])
            y = pts[i][1] + local * (pts[i + 1][1] - pts[i][1])
            # курс — по касательной к ломаной
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            theta = math.atan2(dy, dx) if (dx != 0 or dy != 0) else 0.0
        else:
            sx, sy = default_start
            ex, ey = default_end
            x = sx + t * (ex - sx)
            y = sy + t * (ey - sy)
            theta = math.atan2(ey - sy, ex - sx)
        return x, y, theta

    def get_desired_state(self, t):
        if t < 0:
            t = 0.0

        if t < self.time_approach:
            self.current_phase = 'APPROACH'
            local_t = t / self.time_approach
            x, y, theta = self._get_pose_from_spline(
                self.spline_approach, local_t,
                self.home_pos, self.object_pos
            )
            arm_angles = self.home_angles
            gripper = 0.02
            return {'x': x, 'y': y, 'theta': theta,
                    'arm_angles': arm_angles, 'gripper': gripper}

        t_grasp_start = self.time_approach
        if t < t_grasp_start + self.time_grasp:
            self.current_phase = 'GRASP'
            local_t = (t - t_grasp_start) / self.time_grasp
            x, y = self.object_pos
            theta = 0.0
            arm_angles = self._interpolate_angles(self.home_angles, self.grasp_angles, local_t)
            gripper = 0.02 * (1.0 - local_t)
            return {'x': x, 'y': y, 'theta': theta,
                    'arm_angles': arm_angles, 'gripper': gripper}

        t_move_start = self.time_approach + self.time_grasp
        if t < t_move_start + self.time_move:
            self.current_phase = 'MOVE'
            local_t = (t - t_move_start) / self.time_move
            x, y, theta = self._get_pose_from_spline(
                self.spline_move, local_t,
                self.object_pos, self.marker_pos
            )
            arm_angles = self.lift_angles
            gripper = 0.0
            return {'x': x, 'y': y, 'theta': theta,
                    'arm_angles': arm_angles, 'gripper': gripper}

        t_place_start = t_move_start + self.time_move
        if t < t_place_start + self.time_place:
            self.current_phase = 'PLACE'
            local_t = (t - t_place_start) / self.time_place
            x, y = self.marker_pos
            theta = 0.0
            arm_angles = self._interpolate_angles(self.lift_angles, self.place_angles, local_t)
            if local_t < 0.5:
                gripper = 0.0
            else:
                gripper = 0.02 * (local_t - 0.5) / 0.5
            return {'x': x, 'y': y, 'theta': theta,
                    'arm_angles': arm_angles, 'gripper': gripper}

        else:
            self.current_phase = 'RETURN'
            t_return_start = t_place_start + self.time_place
            local_t = (t - t_return_start) / self.time_return
            if local_t > 1.0:
                local_t = 1.0
            x, y, theta = self._get_pose_from_spline(
                self.spline_return, local_t,
                self.marker_pos, self.home_pos
            )
            arm_angles = self._interpolate_angles(self.place_angles, self.home_angles, local_t)
            gripper = 0.02
            return {'x': x, 'y': y, 'theta': theta,
                    'arm_angles': arm_angles, 'gripper': gripper}