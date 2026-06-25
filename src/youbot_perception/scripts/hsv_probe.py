#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hsv_probe.py — диагностический узел цвета. Запускается ПАРАЛЛЕЛЬНО с миссией
во время реального заезда и пишет в файл, что видит камера: средние BGR и HSV
в центре кадра + в области найденного кубика (если cube_detector публикует
/cube/color). Помогает понять, почему красный читается как синий, прямо в
условиях настоящей попытки.

Запуск (НОУТБУК):
  export ROS_MASTER_URI=http://10.0.0.1:11311
  source ~/youbot_ws/devel/setup.bash
  rosrun youbot_perception hsv_probe.py
  # или: python3 hsv_probe.py

Параметры (необязательно):
  _image_topic:=/usb_cam/image_raw   топик картинки
  _out:=~/hsv_log.csv                файл лога (CSV)
  _rate:=2.0                         сколько раз в секунду писать строку
  _patch:=15                         半-размер окна выборки (px) вокруг центра

Файл лога — CSV, открывается в Excel. Колонки:
  time, encoding, B, G, R, H, S, V, cube_color
где cube_color — что в этот момент выдал детектор (/cube/color), если есть.
"""
import os
import csv
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class HsvProbe:
    def __init__(self):
        rospy.init_node('hsv_probe', anonymous=True)
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        out = rospy.get_param('~out', '~/hsv_log.csv')
        self.out = os.path.expanduser(out)
        self.rate_hz = float(rospy.get_param('~rate', 2.0))
        self.patch = int(rospy.get_param('~patch', 15))

        self.bridge = CvBridge()
        self.last_img = None        # (encoding, bgr)
        self.cube_color = ''        # последнее значение /cube/color

        rospy.Subscriber(self.image_topic, Image, self.img_cb, queue_size=1)
        rospy.Subscriber('/cube/color', String, self.color_cb, queue_size=1)

        # создаём файл с заголовком, если его ещё нет
        new = not os.path.exists(self.out)
        self.f = open(self.out, 'a', newline='')
        self.writer = csv.writer(self.f)
        if new:
            self.writer.writerow(['time', 'encoding', 'B', 'G', 'R',
                                  'H', 'S', 'V', 'cube_color'])
            self.f.flush()

        rospy.loginfo("hsv_probe: пишу в %s, топик %s, %.1f Гц",
                      self.out, self.image_topic, self.rate_hz)

    def color_cb(self, msg):
        self.cube_color = msg.data

    def img_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_img = (msg.encoding, bgr)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "hsv_probe: bridge error: %s", e)

    def sample(self):
        if self.last_img is None:
            return None
        encoding, bgr = self.last_img
        h, w = bgr.shape[:2]
        p = self.patch
        cy, cx = h // 2, w // 2
        roi_bgr = bgr[cy - p:cy + p, cx - p:cx + p].reshape(-1, 3)
        b, g, r = roi_bgr.mean(axis=0)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        roi_hsv = hsv[cy - p:cy + p, cx - p:cx + p].reshape(-1, 3)
        hh, ss, vv = roi_hsv.mean(axis=0)
        return (encoding, b, g, r, hh, ss, vv, self.cube_color)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            s = self.sample()
            if s is not None:
                encoding, b, g, r, hh, ss, vv, color = s
                t = rospy.Time.now().to_sec()
                self.writer.writerow(['%.3f' % t, encoding,
                                      '%.0f' % b, '%.0f' % g, '%.0f' % r,
                                      '%.0f' % hh, '%.0f' % ss, '%.0f' % vv,
                                      color])
                self.f.flush()
                # дублируем в консоль, чтобы видеть вживую
                rospy.loginfo("BGR=(%.0f,%.0f,%.0f) HSV=(%.0f,%.0f,%.0f) cube=%s",
                              b, g, r, hh, ss, vv, color or '-')
            rate.sleep()
        self.f.close()


if __name__ == '__main__':
    try:
        HsvProbe().run()
    except rospy.ROSInterruptException:
        pass
