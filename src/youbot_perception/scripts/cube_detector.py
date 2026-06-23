#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# cube_detector.py  (Редакция 3 — обычная USB-вебкамера, без глубины)
# -----------------------------------------------------------------------------
# Детекция кубика 5x5 см по ОБЫЧНОЙ 2D USB-камере (FIFINE K420 и т.п.).
#
# Что делает:
#   1. Подписывается на RGB-изображение /usb_cam/image_raw и параметры
#      камеры /usb_cam/camera_info.
#   2. По HSV-маскам находит красный / зелёный / синий кубик.
#   3. Определяет ЦВЕТ кубика по самой большой цветной области.
#   4. ОБВОДИТ кубик на изображении (контур + bounding box + подпись цвета) и
#      публикует /cube/debug_image.
#   5. Оценивает 3D-позицию кубика БЕЗ ГЛУБИНЫ: луч из камеры через пиксель
#      пересекается с плоскостью пола (z = floor_z в кадре odom). Результат —
#      /cube/pose (PoseStamped) в odom.
#
# Почему так: USB-вебкамера не даёт глубину. Но кубик стоит на полу, поэтому
# достаточно пересечь зрительный луч с известной плоскостью пола — это даёт
# корректную позицию (x, y) кубика в odom.
#
# Параметры (private):
#   ~image_topic     (str)  RGB-топик          (default: /usb_cam/image_raw)
#   ~info_topic      (str)  camera_info        (default: /usb_cam/camera_info)
#   ~target_frame    (str)  кадр результата    (default: odom)
#   ~floor_z         (float)высота пола в odom  (default: 0.0)
#   ~cube_top_z      (float)высота центра грани кубика над полом (default: 0.025)
#   ~min_area_px     (int)  мин. площадь пятна  (default: 300)
# =============================================================================

import math

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3Stamped
import tf2_geometry_msgs  # noqa: F401


COLOR_RANGES = {
    'red': [((0, 90, 60), (10, 255, 255)),
            ((170, 90, 60), (179, 255, 255))],
    'green': [((40, 70, 40), (85, 255, 255))],
    'blue': [((95, 90, 40), (130, 255, 255))],
}
DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}


class CubeDetector:
    def __init__(self):
        rospy.init_node('cube_detector', anonymous=False)

        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.info_topic = rospy.get_param('~info_topic', '/usb_cam/camera_info')
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.floor_z = float(rospy.get_param('~floor_z', 0.0))
        self.cube_top_z = float(rospy.get_param('~cube_top_z', 0.025))
        self.min_area_px = int(rospy.get_param('~min_area_px', 300))

        self.bridge = CvBridge()
        self.K = None              # матрица камеры 3x3
        self.cam_frame = None

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_color = rospy.Publisher('/cube/color', String,
                                         queue_size=1, latch=True)
        self.pub_pose = rospy.Publisher('/cube/pose', PoseStamped,
                                        queue_size=1, latch=True)
        self.pub_debug = rospy.Publisher('/cube/debug_image', Image,
                                         queue_size=1)

        rospy.Subscriber(self.info_topic, CameraInfo, self.info_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

        rospy.loginfo("cube_detector(USB): image=%s info=%s target=%s",
                      self.image_topic, self.info_topic, self.target_frame)

    # ---------------------------------------------------------------------
    def info_cb(self, msg):
        self.K = np.array(msg.K, dtype=float).reshape(3, 3)
        self.cam_frame = msg.header.frame_id

    # ---------------------------------------------------------------------
    def image_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, "cv_bridge: %s", e)
            return

        color, center_px, contour, bbox = self.detect_color(bgr)
        dbg = bgr.copy()

        if color is not None:
            # --- ОБВОДКА кубика на картинке ---
            cv2.drawContours(dbg, [contour], -1, DRAW_BGR[color], 2)
            x, y, w, h = bbox
            cv2.rectangle(dbg, (x, y), (x + w, y + h), DRAW_BGR[color], 2)
            u, v = center_px
            cv2.circle(dbg, (u, v), 4, DRAW_BGR[color], -1)
            cv2.putText(dbg, color.upper(), (x, max(0, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, DRAW_BGR[color], 2)

            self.pub_color.publish(String(data=color))

            pose = self.pixel_to_floor(u, v, msg.header.frame_id or self.cam_frame)
            if pose is not None:
                self.pub_pose.publish(pose)
                rospy.loginfo_throttle(
                    1.0, "cube: %s px=(%d,%d) odom=(%.3f,%.3f)",
                    color, u, v, pose.pose.position.x, pose.pose.position.y)
        else:
            cv2.putText(dbg, "no cube", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))
        except Exception:
            pass

    # ---------------------------------------------------------------------
    def detect_color(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        best = None
        for color, ranges in COLOR_RANGES.items():
            mask = None
            for (lo, hi) in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < self.min_area_px:
                    continue
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                if best is None or area > best[0]:
                    best = (area, color, (u, v), c, cv2.boundingRect(c))
        if best is None:
            return None, None, None, None
        return best[1], best[2], best[3], best[4]

    # ---------------------------------------------------------------------
    def pixel_to_floor(self, u, v, frame):
        """Проецируем пиксель на плоскость пола (z=floor_z+cube_top_z в odom)
        через пересечение зрительного луча с этой плоскостью."""
        if self.K is None or frame is None:
            return None
        fx = self.K[0, 0]; fy = self.K[1, 1]
        cx = self.K[0, 2]; cy = self.K[1, 2]
        if fx == 0 or fy == 0:
            return None

        # луч в оптическом кадре камеры (z вперёд)
        dx = (u - cx) / fx
        dy = (v - cy) / fy
        ray_cam = Vector3Stamped()
        ray_cam.header.frame_id = frame
        ray_cam.header.stamp = rospy.Time(0)
        ray_cam.vector.x = dx
        ray_cam.vector.y = dy
        ray_cam.vector.z = 1.0

        origin_cam = PointStamped()
        origin_cam.header.frame_id = frame
        origin_cam.header.stamp = rospy.Time(0)
        origin_cam.point.x = 0.0
        origin_cam.point.y = 0.0
        origin_cam.point.z = 0.0

        try:
            ray_t = self.tf_buffer.transform(ray_cam, self.target_frame,
                                             timeout=rospy.Duration(0.2))
            org_t = self.tf_buffer.transform(origin_cam, self.target_frame,
                                             timeout=rospy.Duration(0.2))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF %s->%s: %s", frame, self.target_frame, e)
            return None

        ox, oy, oz = org_t.point.x, org_t.point.y, org_t.point.z
        rx, ry, rz = ray_t.vector.x, ray_t.vector.y, ray_t.vector.z
        plane_z = self.floor_z + self.cube_top_z
        if abs(rz) < 1e-6:
            return None
        t = (plane_z - oz) / rz
        if t <= 0:
            return None  # плоскость позади камеры

        out = PoseStamped()
        out.header.frame_id = self.target_frame
        out.header.stamp = rospy.Time.now()
        out.pose.position.x = ox + t * rx
        out.pose.position.y = oy + t * ry
        out.pose.position.z = plane_z
        out.pose.orientation.w = 1.0
        return out


if __name__ == '__main__':
    try:
        CubeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
