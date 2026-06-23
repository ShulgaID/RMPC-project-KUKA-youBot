#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Узел визуализации миссии youBot в RViz.
#
# ВАЖНО: геометрия комнаты, сетка, точки A/B/C и препятствия теперь
# публикуются нодой room_generator (топик /room/markers, latched, и
# параметры /room/*). Этот узел БОЛЬШЕ НЕ дублирует препятствия — он
# только:
#   - накапливает и публикует реальный путь робота (/robot_path);
#   - дублирует подписи точек A/B/C из параметров (на случай, если
#     room-маркеры ещё не пришли) — но основной их источник room_generator.
# Так устранено рассинхронизирование двух списков препятствий.

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class MissionViz:
    def __init__(self):
        rospy.init_node('mission_viz', anonymous=True)

        self.frame = rospy.get_param('/room/frame_id', 'odom')

        # Точки A/B/C из room_generator (для подписи в этом узле — опционально).
        self.point_A = rospy.get_param('/room/point_A', None)  # парковка/home
        self.point_B = rospy.get_param('/room/point_B', None)  # кубик/pick
        self.point_C = rospy.get_param('/room/point_C', None)  # склад/place

        self.marker_pub = rospy.Publisher('/mission_markers', MarkerArray,
                                          queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=1)

        self.start_pos = None
        self.path = Path()
        self.path.header.frame_id = self.frame
        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)
        rospy.loginfo("Mission visualization node started (path + A/B/C labels).")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_pos is None:
            self.start_pos = (x, y)
            rospy.loginfo("Старт зафиксирован: (%.2f, %.2f)", x, y)

        ps = PoseStamped()
        ps.header.frame_id = self.frame
        ps.header.stamp = rospy.Time.now()
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > 2000:
            self.path.poses.pop(0)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

    def _text(self, mid, pos, text):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "mission_labels"
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.85
        m.pose.orientation.w = 1.0
        m.scale.z = 0.22
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 1.0, 1.0, 1.0)
        m.text = text
        return m

    def publish_markers(self, event=None):
        # Если room_generator не запущен, параметры могут появиться позже —
        # перечитываем.
        if self.point_B is None:
            self.point_B = rospy.get_param('/room/point_B', None)
            self.point_C = rospy.get_param('/room/point_C', None)
            self.point_A = rospy.get_param('/room/point_A', None)

        arr = MarkerArray()
        if self.point_A is not None:
            arr.markers.append(self._text(200, self.point_A, "A: PARK/HOME"))
        if self.point_B is not None:
            arr.markers.append(self._text(201, self.point_B, "B: PICK"))
        if self.point_C is not None:
            arr.markers.append(self._text(202, self.point_C, "C: PLACE"))
        if arr.markers:
            self.marker_pub.publish(arr)


if __name__ == '__main__':
    try:
        node = MissionViz()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
