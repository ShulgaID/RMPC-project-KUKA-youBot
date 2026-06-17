#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion

# Импортируем наши модули (должны лежать в той же папке scripts)
from controller import Controller
from trajectory_planner import TrajectoryPlanner


class YouBotControllerNode:
    # Главный класс узла управления.
    # Инициализирует ROS-узел, подписки, публикации, планировщик и контроллер.
    # В цикле run() вычисляет управление и отправляет команды.
    def __init__(self):
        # Инициализация ROS-узла
        rospy.init_node('youbot_controller', anonymous=True)

        # Настройки частоты управления
        self.rate_hz = rospy.get_param('~rate', 50)          # частота цикла управления (Гц)
        self.rate = rospy.Rate(self.rate_hz)
        self.dt = 1.0 / self.rate_hz                         # шаг дискретизации (сек)

        # Состояние робота (получаем из топиков)
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_theta = 0.0                                # ориентация базы (рад)
        self.arm_positions = [0.0] * 5                       # текущие углы 5 суставов руки

        # Инициализация планировщика траекторий (генерирует желаемое состояние)
        self.trajectory = TrajectoryPlanner()

        # Инициализация контроллера (PID для базы, пропорциональный для руки)
        self.controller = Controller(
            linear_gains=(0.8, 0.05, 0.1),   # Kp, Ki, Kd для линейной скорости
            angular_gains=(1.5, 0.1, 0.2),   # Kp, Ki, Kd для угловой скорости
            arm_gains=(1.0, 0.0, 0.0),       # пропорциональный регулятор для руки
            dt=self.dt
        )

        # Подписки на топики
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # Публикации команд
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Для контроллеров руки и схвата используем trajectory_msgs/JointTrajectory
        self.arm_cmd_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.gripper_cmd_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

        rospy.loginfo("YouBot Controller Node initialized. Starting mission...")

    # Обратные связь для получения состояния робота
    def odom_callback(self, msg):
        self.base_x = msg.pose.pose.position.x
        self.base_y = msg.pose.pose.position.y
        orient = msg.pose.pose.orientation
        # Преобразуем кватернион в углы Эйлера (берём только рыскание)
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.base_theta = yaw

    # Callback для состояния суставов: обновляет текущие углы
    def joint_states_callback(self, msg):
        for i in range(5):
            name = f'arm_joint_{i+1}'
            if name in msg.name:
                idx = msg.name.index(name)
                self.arm_positions[i] = msg.position[idx]
        # Состояние схвата можно получить аналогично, но в данной версии не используется

    # Отправка управляющих команд
    # Отправляет команду скорости для мобильной платформы
    def send_base_command(self, vx, vy, omega):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

    #Отправляет команду позиционирования для 5 суставов руки
    def send_arm_command(self, angles):
        msg = JointTrajectory()
        msg.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(0.1)   # небольшое время для выполнения
        msg.points.append(point)
        self.arm_cmd_pub.publish(msg)

    #Отправляет команду раскрытия схвата
    def send_gripper_command(self, opening):
        msg = JointTrajectory()
        msg.joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
        point = JointTrajectoryPoint()
        # Обе губки двигаются симметрично
        point.positions = [opening, opening]
        point.time_from_start = rospy.Duration(0.1)
        msg.points.append(point)
        self.gripper_cmd_pub.publish(msg)

    # Главный цикл управления
    # Основной цикл: получает желаемое состояние от планировщика,
    # вычисляет управляющие сигналы через контроллер и отправляет команды.
    def run(self):
        start_time = rospy.Time.now().to_sec()   # время старта миссии

        while not rospy.is_shutdown():
            # 1. Текущее время от начала миссии (сек)
            t = rospy.Time.now().to_sec() - start_time

            # 2. Получить желаемое состояние (база, рука, схват) от планировщика
            desired = self.trajectory.get_desired_state(t)

            # 3. Получить текущее состояние
            current_pose = {
                'x': self.base_x,
                'y': self.base_y,
                'theta': self.base_theta
            }
            current_arm = self.arm_positions[:]   # копия текущих углов руки

            # 4. Вычислить управление для базы (линейные и угловая скорости)
            vx, vy, omega = self.controller.compute_base_control(current_pose, desired)

            # 5. Вычислить управление для руки (желаемые углы)
            arm_cmd = self.controller.compute_arm_control(current_arm, desired['arm_angles'])

            # 6. Команда для схвата (из планировщика)
            gripper_cmd = desired['gripper']

            # 7. Отправить все команды
            self.send_base_command(vx, vy, omega)
            self.send_arm_command(arm_cmd)
            self.send_gripper_command(gripper_cmd)

            # 8. Логирование для отладки (периодическое, раз в секунду)
            if int(t * 10) % 10 == 0:
                rospy.loginfo_throttle(1,
                    f"Time: {t:.2f}, State: {self.trajectory.current_phase}, "
                    f"Goal: ({desired['x']:.2f}, {desired['y']:.2f}), "
                    f"Curr: ({self.base_x:.2f}, {self.base_y:.2f})"
                )
            # Если мы в фазе PLACE и расстояние до цели < 0.1 м, принудительно останавливаем
            if self.trajectory.current_phase == 'PLACE':
                distance = math.hypot(self.base_x, self.base_y)
                if distance < 0.1:
                    vx, vy, omega = 0.0, 0.0, 0.0

            # 9. Пауза до следующего такта
            self.rate.sleep()

# Точка входа

if __name__ == '__main__':
    try:
        node = YouBotControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass