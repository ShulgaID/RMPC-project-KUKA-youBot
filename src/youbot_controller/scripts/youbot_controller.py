#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion

from controller import Controller
from trajectory_planner import TrajectoryPlanner


class YouBotControllerNode:
    def __init__(self):
        rospy.init_node('youbot_controller', anonymous=True)

        self.rate_hz = rospy.get_param('~rate', 50)
        self.rate = rospy.Rate(self.rate_hz)
        self.dt = 1.0 / self.rate_hz

        # Состояние робота
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_theta = 0.0
        self.arm_positions = [0.0] * 5
        self.odom_received = False          # пришла ли хоть одна одометрия

        self.trajectory = TrajectoryPlanner()
        self.trajectory_initialized = False

        self.controller = Controller(
            linear_gains=(0.8, 0.0, 0.1),
            angular_gains=(1.5, 0.0, 0.2),
            arm_gains=(1.0, 0.0, 0.0),
            dt=self.dt
        )

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arm_cmd_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.gripper_cmd_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

        rospy.loginfo("YouBot Controller Node initialized. Starting mission...")

    # Колбэк одометрии: ТОЛЬКО обновляет позу. Тяжёлое планирование сюда не кладём.
    def odom_callback(self, msg):
        self.base_x = msg.pose.pose.position.x
        self.base_y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.base_theta = yaw
        self.odom_received = True

    def joint_states_callback(self, msg):
        for i in range(5):
            name = f'arm_joint_{i+1}'
            if name in msg.name:
                idx = msg.name.index(name)
                self.arm_positions[i] = msg.position[idx]

    def send_base_command(self, vx, vy, omega):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

    def send_arm_command(self, angles):
        msg = JointTrajectory()
        msg.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(0.1)
        msg.points.append(point)
        self.arm_cmd_pub.publish(msg)

    def send_gripper_command(self, opening):
        msg = JointTrajectory()
        msg.joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
        point = JointTrajectoryPoint()
        point.positions = [opening, opening]
        point.time_from_start = rospy.Duration(0.1)
        msg.points.append(point)
        self.gripper_cmd_pub.publish(msg)

    def run(self):
        # 1. Ждём первую одометрию (вне колбэка, не блокируя приём)
        rospy.loginfo("Ожидание первой одометрии...")
        while not rospy.is_shutdown() and not self.odom_received:
            self.rate.sleep()

        # 2. Планируем миссию ОДИН раз, в основном потоке
        start_pose = (self.base_x, self.base_y, self.base_theta)
        rospy.loginfo("init_mission со старта (%.2f, %.2f, %.2f)", *start_pose)
        try:
            self.trajectory.init_mission(start_pose)
            self.trajectory_initialized = True
            rospy.loginfo("Миссия инициализирована.")
        except Exception as e:
            rospy.logerr("Ошибка init_mission: %s", e)
            return

        # 3. Старт отсчёта времени миссии — ПОСЛЕ планирования
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - start_time
            desired = self.trajectory.get_desired_state(t)

            current_pose = {'x': self.base_x, 'y': self.base_y, 'theta': self.base_theta}
            current_arm = self.arm_positions[:]

            vx, vy, omega = self.controller.compute_base_control(current_pose, desired)
            arm_cmd = self.controller.compute_arm_control(current_arm, desired['arm_angles'])
            gripper_cmd = desired['gripper']

            # Остановка в фазе PLACE: расстояние ДО ЦЕЛИ (а не до начала координат)
            if self.trajectory.current_phase == 'PLACE':
                dist_to_goal = math.hypot(desired['x'] - self.base_x,
                                          desired['y'] - self.base_y)
                if dist_to_goal < 0.1:
                    vx, vy, omega = 0.0, 0.0, 0.0

            self.send_base_command(vx, vy, omega)
            self.send_arm_command(arm_cmd)
            self.send_gripper_command(gripper_cmd)

            rospy.loginfo_throttle(1,
                f"t={t:.2f}, phase={self.trajectory.current_phase}, "
                f"goal=({desired['x']:.2f},{desired['y']:.2f}), "
                f"curr=({self.base_x:.2f},{self.base_y:.2f}), "
                f"cmd=({vx:.2f},{vy:.2f},{omega:.2f})")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = YouBotControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass