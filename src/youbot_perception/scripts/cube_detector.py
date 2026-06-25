#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# cube_detector.py  (Редакция 4 — USB-вебкамера; кубик по цвету + чёрные
#                    угловые маркеры границ рабочей зоны)
# -----------------------------------------------------------------------------
# Детекция по ОБЫЧНОЙ 2D USB-камере (FIFINE K420 и т.п.), без глубины.
#
# Что делает:
#   1. Подписывается на RGB /usb_cam/image_raw и параметры /usb_cam/camera_info.
#   2. КУБИК: по HSV-маскам находит красный/зелёный/синий кубик, определяет
#      цвет, обводит на /cube/debug_image и публикует:
#         /cube/color (String), /cube/pose (PoseStamped в odom).
#   3. УГЛОВЫЕ МАРКЕРЫ ГРАНИЦ: находит ЧЁРНЫЕ квадратные маркеры по углам
#      рабочей зоны. Для каждого берёт ВЕРХНИЙ ВНЕШНИЙ угол (как граница поля),
#      проецирует его на плоскость пола и публикует:
#         /field/corners (geometry_msgs/PolygonStamped, точки в odom);
#         /field/markers (visualization_msgs/MarkerArray для RViz).
#      Маркеры также обводятся на /cube/debug_image.
#
# Позиция БЕЗ ГЛУБИНЫ: зрительный луч через пиксель пересекается с плоскостью
# пола (z = floor_z в odom). Для кубика плоскость приподнята на cube_top_z.
#
# Калибровка камеры теперь НЕ требует шахматной доски: матрица камеры берётся
# из camera_info (usb_cam отдаёт разумную оценку по FOV), а РАЗМЕТКА РАБОЧЕЙ
# ЗОНЫ задаётся чёрными угловыми маркерами, расставленными на известном
# расстоянии в метрах. См. методичку, Раздел B.
#
# Параметры (private):
#   ~image_topic     (str)   RGB-топик          (default: /usb_cam/image_raw)
#   ~info_topic      (str)   camera_info        (default: /usb_cam/camera_info)
#   ~target_frame    (str)   кадр результата    (default: odom)
#   ~floor_z         (float) высота пола в odom  (default: 0.0)
#   ~cube_top_z      (float) высота центра грани кубика над полом (0.025)
#   ~min_area_px     (int)   мин. площадь цветного пятна (300)
#   ~detect_markers  (bool)  искать ли чёрные угловые маркеры (True)
#   ~marker_min_area_px (int) мин. площадь чёрного маркера (400)
#   ~marker_max_area_px (int) макс. площадь (0 = без ограничения)
#   ~black_v_max     (int)   верхний порог V для «чёрного» (60)
#   ~black_s_max     (int)   верхний порог S для «чёрного» (90)
# =============================================================================

import math

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import (PoseStamped, PointStamped, Vector3Stamped,
                               PolygonStamped, Point32)
from visualization_msgs.msg import Marker, MarkerArray
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
        # Макс. площадь пятна кубика (пикс). Больше — это картонка-пластина,
        # отбрасываем. 0 = без ограничения. Для кубика 5 см на дистанции
        # ~0.5–0.8 м разумно ~6000–12000; пластины 25 см дают в разы больше.
        self.cube_max_area = int(rospy.get_param('~cube_max_area_px', 40000))

        # --- фильтр ФОРМЫ пятна кубика ---
        # Кубик может стоять ВЕРТИКАЛЬНО и в кадре выглядеть как прямоугольник
        # выше своей ширины (ar = w/h может быть ~0.5). Поэтому нижнюю границу
        # ar делаем мягче. Диапазон ar и мин. extent — параметры.
        self.cube_ar_min = float(rospy.get_param('~cube_ar_min', 0.35))
        self.cube_ar_max = float(rospy.get_param('~cube_ar_max', 2.2))
        self.cube_extent_min = float(rospy.get_param('~cube_extent_min', 0.55))

        # --- параметры детекции чёрных угловых маркеров ---
        self.detect_markers = bool(rospy.get_param('~detect_markers', True))
        self.marker_min_area = int(rospy.get_param('~marker_min_area_px', 400))
        self.marker_max_area = int(rospy.get_param('~marker_max_area_px', 0))
        self.black_v_max = int(rospy.get_param('~black_v_max', 60))
        self.black_s_max = int(rospy.get_param('~black_s_max', 90))

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
        self.pub_corners = rospy.Publisher('/field/corners', PolygonStamped,
                                           queue_size=1, latch=True)
        self.pub_field_markers = rospy.Publisher('/field/markers', MarkerArray,
                                                 queue_size=1, latch=True)

        rospy.Subscriber(self.info_topic, CameraInfo, self.info_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

        rospy.loginfo("cube_detector(USB): image=%s info=%s target=%s markers=%s",
                      self.image_topic, self.info_topic, self.target_frame,
                      self.detect_markers)

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

        frame = msg.header.frame_id or self.cam_frame
        dbg = bgr.copy()

        # ===== 1) КУБИК ПО ЦВЕТУ =====
        color, center_px, contour, bbox = self.detect_color(bgr)
        if color is not None:
            cv2.drawContours(dbg, [contour], -1, DRAW_BGR[color], 2)
            x, y, w, h = bbox
            cv2.rectangle(dbg, (x, y), (x + w, y + h), DRAW_BGR[color], 2)
            u, v = center_px
            cv2.circle(dbg, (u, v), 4, DRAW_BGR[color], -1)
            cv2.putText(dbg, color.upper(), (x, max(0, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, DRAW_BGR[color], 2)

            self.pub_color.publish(String(data=color))
            pose = self.pixel_to_floor(u, v, frame, self.floor_z + self.cube_top_z)
            if pose is not None:
                self.pub_pose.publish(pose)
                rospy.loginfo_throttle(
                    1.0, "cube: %s px=(%d,%d) odom=(%.3f,%.3f)",
                    color, u, v, pose.pose.position.x, pose.pose.position.y)
        else:
            cv2.putText(dbg, "no cube", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        # ===== 2) ЧЁРНЫЕ УГЛОВЫЕ МАРКЕРЫ ГРАНИЦ =====
        if self.detect_markers:
            self.process_markers(bgr, dbg, frame)

        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))
        except Exception:
            pass

    # ---------------------------------------------------------------------
    def detect_color(self, bgr):
        """Ищет КУБИК (компактное цветное пятно), отсекая большие плоские
        картонки-пластины. Картонки red/green/blue на полу крупные, поэтому
        отбрасываем пятна площадью больше cube_max_area_px и предпочитаем
        квадрато-подобные (extent высокий, стороны близки)."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        best = None  # (score, color, (u,v), contour, bbox)
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
                # ОТСЕКАЕМ КАРТОНКИ: слишком большое пятно — это пластина, не кубик
                if self.cube_max_area > 0 and area > self.cube_max_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                if w == 0 or h == 0:
                    continue
                ar = w / float(h)
                extent = area / float(w * h)        # заполненность bbox
                # кубик: допускаем вытянутый по высоте прямоугольник (стоит
                # вертикально), но требуем хорошую заполненность bbox.
                if not (self.cube_ar_min <= ar <= self.cube_ar_max
                        and extent >= self.cube_extent_min):
                    continue
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                # score: предпочитаем компактные «кубикоподобные», но не самые
                # мелкие — берём площадь, уже ограниченную сверху.
                score = area * extent
                if best is None or score > best[0]:
                    best = (score, color, (u, v), c, (x, y, w, h))
        if best is None:
            return None, None, None, None
        return best[1], best[2], best[3], best[4]

    # ---------------------------------------------------------------------
    def process_markers(self, bgr, dbg, frame):
        """Находит чёрные квадратные маркеры, берёт ВЕРХНИЙ ВНЕШНИЙ угол
        каждого, проецирует на пол и публикует границу рабочей зоны."""
        markers_px = self.detect_black_markers(bgr)

        img_cx = bgr.shape[1] / 2.0
        img_cy = bgr.shape[0] / 2.0

        corners_world = []
        for (cnt, bbox, quad) in markers_px:
            # ВЕРХНИЙ ВНЕШНИЙ угол: из 4 вершин квадрата берём тот, что выше
            # всего (минимальный v) и при этом дальше от центра кадра по
            # горизонтали — это «внешний» верхний угол маркера-уголка.
            top_outer = self.pick_top_outer_corner(quad, img_cx)
            ou, ov = int(top_outer[0]), int(top_outer[1])

            # рисуем маркер и его опорный угол
            cv2.drawContours(dbg, [cnt], -1, (0, 200, 255), 2)
            cv2.circle(dbg, (ou, ov), 6, (0, 165, 255), -1)
            cv2.putText(dbg, "corner", (ou + 6, ov),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

            pose = self.pixel_to_floor(ou, ov, frame, self.floor_z)
            if pose is not None:
                corners_world.append((pose.pose.position.x,
                                      pose.pose.position.y))

        if corners_world:
            self.publish_field(corners_world)
            rospy.loginfo_throttle(2.0, "field corners: %d -> %s",
                                   len(corners_world),
                                   ["(%.2f,%.2f)" % c for c in corners_world])

    # ---------------------------------------------------------------------
    def detect_black_markers(self, bgr):
        """Возвращает список (contour, bbox, quad[4x2]) для чёрных квадратов."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # «Чёрный»: низкая яркость V, невысокая насыщенность S (любой H)
        mask = cv2.inRange(hsv, np.array((0, 0, 0)),
                           np.array((179, self.black_s_max, self.black_v_max)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self.marker_min_area:
                continue
            if self.marker_max_area > 0 and area > self.marker_max_area:
                continue
            # отбираем квадрато-подобные: 4 вершины при аппроксимации и
            # отношение сторон близко к 1
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            x, y, w, h = cv2.boundingRect(c)
            ar = w / float(h) if h > 0 else 0
            if len(approx) == 4 and 0.6 <= ar <= 1.7:
                quad = approx.reshape(-1, 2).astype(float)
                out.append((c, (x, y, w, h), quad))
        return out

    @staticmethod
    def pick_top_outer_corner(quad, img_cx):
        """Из 4 вершин квадрата выбирает верхний внешний угол:
        приоритет — меньший v (выше в кадре); при близких v берём вершину,
        дальше отстоящую от центра кадра по горизонтали (внешнюю)."""
        # сортируем по v (вверх), берём две верхние, из них — внешнюю
        idx = sorted(range(len(quad)), key=lambda i: quad[i][1])
        top_two = [quad[idx[0]], quad[idx[1]]]
        # внешняя = дальше от центра кадра по |u - img_cx|
        return max(top_two, key=lambda p: abs(p[0] - img_cx))

    # ---------------------------------------------------------------------
    def publish_field(self, corners_world):
        # PolygonStamped с углами рабочей зоны (в odom)
        poly = PolygonStamped()
        poly.header.frame_id = self.target_frame
        poly.header.stamp = rospy.Time.now()
        for (wx, wy) in corners_world:
            p = Point32()
            p.x = wx; p.y = wy; p.z = self.floor_z
            poly.polygon.points.append(p)
        self.pub_corners.publish(poly)

        # Маркеры RViz: сферы в углах + линия границы
        arr = MarkerArray()
        for i, (wx, wy) in enumerate(corners_world):
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = rospy.Time.now()
            m.ns = 'field_corners'; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = wx; m.pose.position.y = wy
            m.pose.position.z = self.floor_z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.0; m.color.a = 1.0
            arr.markers.append(m)
        # замкнутая линия по углам
        from geometry_msgs.msg import Point as GPoint
        line = Marker()
        line.header.frame_id = self.target_frame
        line.header.stamp = rospy.Time.now()
        line.ns = 'field_boundary'; line.id = 1000
        line.type = Marker.LINE_STRIP; line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 0.0; line.color.a = 1.0
        line.pose.orientation.w = 1.0
        for (wx, wy) in corners_world + corners_world[:1]:
            gp = GPoint(); gp.x = wx; gp.y = wy; gp.z = self.floor_z
            line.points.append(gp)
        arr.markers.append(line)
        self.pub_field_markers.publish(arr)

    # ---------------------------------------------------------------------
    def pixel_to_floor(self, u, v, frame, plane_z):
        """Проекция пикселя на горизонтальную плоскость z=plane_z (в odom)
        через пересечение зрительного луча с этой плоскостью."""
        if self.K is None or frame is None:
            return None
        fx = self.K[0, 0]; fy = self.K[1, 1]
        cx = self.K[0, 2]; cy = self.K[1, 2]
        if fx == 0 or fy == 0:
            return None

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
        if abs(rz) < 1e-6:
            return None
        t = (plane_z - oz) / rz
        if t <= 0:
            return None

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
