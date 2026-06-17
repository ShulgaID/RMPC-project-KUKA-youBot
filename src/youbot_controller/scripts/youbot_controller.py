#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class YouBotController:
    def __init__(self):
        rospy.init_node('youbot_controller')
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_1/arm_controller/command', Float64MultiArray, queue_size=10)
        self.gripper_pub = rospy.Publisher('/arm_1/gripper_controller/command', Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(50)

    def joint_callback(self, msg):
        # Здесь можно реализовать логику управления
        pass

    def run(self):
        while not rospy.is_shutdown():
            # Пример: движение вперёд
            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = YouBotController()
        node.run()
    except rospy.ROSInterruptException:
        pass