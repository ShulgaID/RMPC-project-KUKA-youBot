#!/usr/bin/env python3
# trajectory_planner.py – полная версия с RRT и сплайнами

import math
import numpy as np
from scipy.interpolate import CubicSpline
from rrt_planner import RRTPlanner
import rospy

class TrajectoryPlanner:
    def __init__(self):
        # Координаты в области 5×5
        self.object_pos = (1.0, 1.0)
        self.marker_pos = (4.0, 4.0)
        self.home_pos = None

        # Препятствия
        self.obstacles = [
            (1.5, 2.0, 0.4),
            (2.5, 1.5, 0.35),
            (3.0, 3.0, 0.45),
            (2.0, 3.5, 0.3),
            (3.5, 2.5, 0.25)
        ]
        self.bounds = (-0.5, 4.5, -0.5, 4.5)

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
        self.home_pos = (start_pose[0], start_pose[1])

        rospy.loginfo("Планирование путей RRT...")
        self.path_approach = self._plan_path(self.home_pos, self.object_pos)
        self.path_move = self._plan_path(self.object_pos, self.marker_pos)
        self.path_return = self._plan_path(self.marker_pos, self.home_pos)

        rospy.loginfo("Paths lengths: approach=%d, move=%d, return=%d", len(self.path_approach), len(self.path_move), len(self.path_return))
        self.spline_approach = self._create_spline_or_none(self.path_approach)
        self.spline_move = self._create_spline_or_none(self.path_move)
        self.spline_return = self._create_spline_or_none(self.path_return)

        if self.spline_approach is None:
            rospy.logwarn("RRT не нашёл путь к объекту, используется прямая линия")
        if self.spline_move is None:
            rospy.logwarn("RRT не нашёл путь к маркеру, используется прямая линия")
        if self.spline_return is None:
            rospy.logwarn("RRT не нашёл путь обратно в home, используется прямая линия")
        rospy.loginfo("Планирование завершено.")

    def _plan_path(self, start, goal):
        planner = RRTPlanner(start, goal, self.obstacles, self.bounds, max_iter=800, step_size=0.2)
        return planner.plan()

    def _create_spline_or_none(self, points):
        if len(points) > 1:
            pts = np.array(points)
            t = np.linspace(0, 1, len(pts))
            spline_x = CubicSpline(t, pts[:,0])
            spline_y = CubicSpline(t, pts[:,1])
            return (spline_x, spline_y)
        return None

    def _interpolate_angles(self, start_angles, end_angles, progress):
        return [start_angles[i] + progress * (end_angles[i] - start_angles[i]) for i in range(len(start_angles))]

    def _get_pose_from_spline(self, spline, t, default_start, default_end):
        if default_start is None or default_end is None:
            return 0.0, 0.0, 0.0
        if spline is not None:
            x = spline[0](t)
            y = spline[1](t)
            dx = spline[0].derivative()(t)
            dy = spline[1].derivative()(t)
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