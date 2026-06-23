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
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0475)  # м
        self.lx = rospy.get_param('~lx', 0.228)  # расстояние от центра до колеса по X
        self.ly = rospy.get_param('~ly', 0.158)  # расстояние от центра до колеса по Y

        # Состояние одометрии
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_time = rospy.Time.now()

        # Подписка на команды скорости
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Публикация команд скорости колёс
        self.pub_fl = rospy.Publisher('/wheel_joint_fl_velocity_controller/command', Float64, queue_size=10)
        self.pub_fr = rospy.Publisher('/wheel_joint_fr_velocity_controller/command', Float64, queue_size=10)
        self.pub_bl = rospy.Publisher('/wheel_joint_bl_velocity_controller/command', Float64, queue_size=10)
        self.pub_br = rospy.Publisher('/wheel_joint_br_velocity_controller/command', Float64, queue_size=10)

        # Публикация одометрии
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # ЕДИНЫЙ таймер: интегрирует одометрию и публикует её (50 Гц)
        self.odom_timer = rospy.Timer(rospy.Duration(0.02), self.odom_update_timer)

        rospy.loginfo("Mecanum Controller initialized")

    # Обработчик команд скорости: только пересчёт колёс + запоминание скоростей
    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z

        # Обратная кинематика Mecanum
        R = self.wheel_radius
        L = self.lx + self.ly
        w_fl = (self.vx - self.vy - L * self.omega) / R
        w_fr = (self.vx + self.vy + L * self.omega) / R
        w_bl = (self.vx + self.vy - L * self.omega) / R
        w_br = (self.vx - self.vy + L * self.omega) / R

        self.pub_fl.publish(w_fl)
        self.pub_fr.publish(w_fr)
        self.pub_bl.publish(w_bl)
        self.pub_br.publish(w_br)

    # Таймер: интегрирование одометрии по реальному dt + публикация
    def odom_update_timer(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        if dt <= 0.0:
            return

        # Интегрируем текущие скорости (из локальной СК в глобальную)
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = math.atan2(math.sin(self.theta + delta_theta),
                                math.cos(self.theta + delta_theta))

        self.publish_odometry(current_time)

    def publish_odometry(self, current_time):
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # TF odom -> base_footprint
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0), quat, current_time,
            "base_footprint", "odom")

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.omega
        self.odom_pub.publish(odom_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MecanumController()
        node.run()
    except rospy.ROSInterruptException:
        pass