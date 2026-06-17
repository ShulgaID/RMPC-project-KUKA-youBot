#!/usr/bin/env python3
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class MecanumController:
    def __init__(self):
        rospy.init_node('mecanum_controller', anonymous=True)

        # Параметры платформы
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0475) # м
        self.lx = rospy.get_param('~lx', 0.228) # расстояние от центра до колеса по X
        self.ly = rospy.get_param('~ly', 0.158) # расстояние от центра до колеса по Y

        # Переменные для одометрии
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # Подписка на команды скорости
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Публикация команд в velocity_controllers для каждого колеса
        self.pub_fl = rospy.Publisher('/wheel_joint_fl_velocity_controller/command', Float64, queue_size=10)
        self.pub_fr = rospy.Publisher('/wheel_joint_fr_velocity_controller/command', Float64, queue_size=10)
        self.pub_bl = rospy.Publisher('/wheel_joint_bl_velocity_controller/command', Float64, queue_size=10)
        self.pub_br = rospy.Publisher('/wheel_joint_br_velocity_controller/command', Float64, queue_size=10)

        # Публикация одометрии
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.loginfo("Mecanum Controller initialized")
        self.rate = rospy.Rate(50)

    # Обработчик команд скорости
    def cmd_vel_callback(self, msg):
        # Получаем целевые скорости из сообщения
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # Сохраняем для одометрии
        self.vx = vx
        self.vy = vy
        self.omega = omega

        # Обратная кинематика Mecanum-платформы
        R = self.wheel_radius
        L = self.lx + self.ly
        
        # Скорости колёс (рад/с)
        w_fl = (vx - vy - L * omega) / R
        w_fr = (vx + vy + L * omega) / R
        w_bl = (vx + vy - L * omega) / R
        w_br = (vx - vy + L * omega) / R

        # Публикация в топики velocity_controllers
        self.pub_fl.publish(w_fl)
        self.pub_fr.publish(w_fr)
        self.pub_bl.publish(w_bl)
        self.pub_br.publish(w_br)

        # Обновление одометрии (вычисляется на основе скоростей)
        self.update_odometry(vx, vy, omega)

    # Обновление одометрии (интегрирование скоростей)
    def update_odometry(self, vx, vy, omega):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Если dt слишком мало или нулевое – пропускаем
        if dt <= 0:
            return

        # Интегрируем скорости в глобальной системе координат
        # Сначала переводим скорости из локальной системы робота в глобальную
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Нормализация угла
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Публикация одометрии
        self.publish_odometry()
    
    # Публикация одометрии и трансформации
    def publish_odometry(self):
        current_time = rospy.Time.now()

        # 1. Публикация трансформации odom -> base_footprint (для RViz)
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quat,
            current_time,
            "base_footprint",   # child frame
            "odom"              # parent frame
        )

        # 2. Публикация сообщения Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Позиция
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Ковариация позиции (можно заполнить приблизительно)
        odom_msg.pose.covariance = [0.001, 0, 0, 0, 0, 0,
                                    0, 0.001, 0, 0, 0, 0,
                                    0, 0, 0.001, 0, 0, 0,
                                    0, 0, 0, 0.001, 0, 0,
                                    0, 0, 0, 0, 0.001, 0,
                                    0, 0, 0, 0, 0, 0.001]

        # Скорости (можно заполнить для полноты)
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.omega

        # Ковариация скоростей
        odom_msg.twist.covariance = [0.001, 0, 0, 0, 0, 0,
                                     0, 0.001, 0, 0, 0, 0,
                                     0, 0, 0.001, 0, 0, 0,
                                     0, 0, 0, 0.001, 0, 0,
                                     0, 0, 0, 0, 0.001, 0,
                                     0, 0, 0, 0, 0, 0.001]

        self.odom_pub.publish(odom_msg)

    # Основной цикл (spin, так как всё в callback)
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MecanumController()
        node.run()
    except rospy.ROSInterruptException:
        pass