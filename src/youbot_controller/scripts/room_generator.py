#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# room_generator.py
# -----------------------------------------------------------------------------
# Отдельная нода, которая ЕДИНОЖДЫ строит "комнату" и является единственным
# источником истины о препятствиях для остального проекта.
#
# Что делает нода:
#   1. Создаёт закрытое пространство n x m (комната) со СТЕНКАМИ, чтобы робот
#      не уехал за её пределы. Стенки представлены как препятствия-отрезки,
#      разбитые на цилиндры (планировщик у нас круговой), а также публикуются
#      как визуальные маркеры-кубы.
#   2. Делит комнату сеткой на секции (число ячеек зависит от размеров комнаты).
#   3. Резервирует три ячейки под точки A, B, C:
#         A - парковка (home): робот выезжает и возвращается сюда;
#         B - точка кубика (pick): откуда робот берёт предмет;
#         C - точка склада (place): куда робот кладёт кубик.
#   4. Во ВСЕ остальные ячейки кладёт от 0 до 2 ВИРТУАЛЬНЫХ цилиндров
#      (в реальности их нет) с небольшим разбросом радиуса и положения
#      внутри ячейки. Используется random.seed -> результат воспроизводим.
#      Гарантируется проходимость: препятствия ужимаются/прорежаются так,
#      чтобы между ними и точками A/B/C всегда оставался проезд.
#
# Размеры комнаты ПАРАМЕТРИЗОВАНЫ (ROS-параметры). На реальном роботе комната
# будет другого размера — достаточно поменять параметры, код менять не нужно.
#
# Нода публикует результат тремя способами, чтобы остальные узлы читали
# из ОДНОГО места:
#   - ROS-параметры:   /room/...  (геометрия, точки, список препятствий);
#   - топик /room/obstacles  (std_msgs/Float32MultiArray) — [cx,cy,r, cx,cy,r,...]
#     удобно подписаться планировщику;
#   - топик /room/markers    (visualization_msgs/MarkerArray, latched) — для RViz.
# =============================================================================

import math
import random

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class RoomGenerator:
    def __init__(self):
        rospy.init_node('room_generator', anonymous=False)

        # ---------------------- Параметры комнаты ----------------------------
        # Размеры комнаты в метрах (n x m). В реальности будут другими -> меняем
        # только эти параметры.
        self.room_w = float(rospy.get_param('~room_width', 5.0))   # n, вдоль X
        self.room_h = float(rospy.get_param('~room_height', 5.0))  # m, вдоль Y
        # Левый-нижний угол комнаты в кадре odom (обычно робот стартует в (0,0),
        # поэтому удобно держать парковку рядом с началом координат).
        self.origin_x = float(rospy.get_param('~origin_x', 0.0))
        self.origin_y = float(rospy.get_param('~origin_y', 0.0))

        # Желаемый размер ячейки сетки (м). Число ячеек считается от размеров
        # комнаты -> сетка автоматически подстраивается под любую комнату.
        self.cell_size = float(rospy.get_param('~cell_size', 1.0))

        # Толщина стенок (м) и высота для визуализации.
        self.wall_thickness = float(rospy.get_param('~wall_thickness', 0.10))
        self.wall_height = float(rospy.get_param('~wall_height', 0.5))

        # Радиус робота (м) — берётся для гарантии проходимости. youBot ~0.58x0.38,
        # берём вписанный радиус с запасом.
        self.robot_radius = float(rospy.get_param('~robot_radius', 0.35))
        # Минимальный зазор между препятствиями/стенками и роботом.
        self.clearance = float(rospy.get_param('~clearance', 0.10))

        # Диапазон радиусов виртуальных цилиндров (м).
        self.obst_r_min = float(rospy.get_param('~obstacle_r_min', 0.15))
        self.obst_r_max = float(rospy.get_param('~obstacle_r_max', 0.25))
        # Максимум препятствий на ячейку (0..max). По ТЗ — 2.
        self.max_per_cell = int(rospy.get_param('~max_obstacles_per_cell', 2))

        # Зерно ГСЧ — ОБЯЗАТЕЛЬНО для воспроизводимости.
        self.seed = int(rospy.get_param('~seed', 42))

        # Кадр координат — тот же, в котором едет робот.
        self.frame = rospy.get_param('~frame_id', 'odom')

        # Какие ячейки сетки назначить под A, B, C. Если не заданы — выбираются
        # автоматически (углы/центр). Формат параметра: "col,row".
        self.cell_A = rospy.get_param('~cell_A', '')   # парковка / home
        self.cell_B = rospy.get_param('~cell_B', '')   # кубик / pick
        self.cell_C = rospy.get_param('~cell_C', '')   # склад / place

        # ------------------------- Издатели ----------------------------------
        self.markers_pub = rospy.Publisher('/room/markers', MarkerArray,
                                           queue_size=1, latch=True)
        self.obstacles_pub = rospy.Publisher('/room/obstacles', Float32MultiArray,
                                             queue_size=1, latch=True)

        # ------------------------- Построение --------------------------------
        random.seed(self.seed)   # фиксируем ГСЧ -> результат воспроизводим

        self.n_cols = max(1, int(round(self.room_w / self.cell_size)))
        self.n_rows = max(1, int(round(self.room_h / self.cell_size)))
        # Фактический размер ячейки (комната делится ровно).
        self.cw = self.room_w / self.n_cols
        self.ch = self.room_h / self.n_rows

        rospy.loginfo("Комната %.2f x %.2f м, сетка %d x %d (ячейка %.2f x %.2f м), seed=%d",
                      self.room_w, self.room_h, self.n_cols, self.n_rows,
                      self.cw, self.ch, self.seed)

        self.points = self._choose_abc_cells()      # {'A': (col,row), ...}
        self.point_xy = {k: self._cell_center(*v)   # реальные координаты A/B/C
                         for k, v in self.points.items()}

        self.wall_obstacles = self._build_walls()   # список (cx,cy,r) для стенок
        self.virtual_obstacles = self._build_virtual_obstacles()

        # Всё вместе — то, что увидит планировщик как круговые препятствия.
        self.all_obstacles = self.wall_obstacles + self.virtual_obstacles

        self._publish_params()
        self._publish_obstacles()
        self._publish_markers()

        rospy.loginfo("Точки: A(park)=%s  B(pick)=%s  C(place)=%s",
                      self._fmt(self.point_xy['A']),
                      self._fmt(self.point_xy['B']),
                      self._fmt(self.point_xy['C']))
        rospy.loginfo("Стенок-цилиндров: %d, виртуальных препятствий: %d",
                      len(self.wall_obstacles), len(self.virtual_obstacles))

    # ========================================================================
    # Геометрия сетки
    # ========================================================================
    def _cell_center(self, col, row):
        x = self.origin_x + (col + 0.5) * self.cw
        y = self.origin_y + (row + 0.5) * self.ch
        return (x, y)

    def _cell_bounds(self, col, row):
        x0 = self.origin_x + col * self.cw
        y0 = self.origin_y + row * self.ch
        return (x0, x0 + self.cw, y0, y0 + self.ch)

    def _parse_cell(self, s):
        try:
            c, r = s.split(',')
            return (int(c), int(r))
        except Exception:
            return None

    def _choose_abc_cells(self):
        """Назначаем 3 ячейки под A, B, C. Берём параметры, иначе — разумные
        умолчания: A в углу у начала координат, C в противоположном углу,
        B где-то посередине. Все три ОБЯЗАТЕЛЬНО различны."""
        a = self._parse_cell(self.cell_A)
        b = self._parse_cell(self.cell_B)
        c = self._parse_cell(self.cell_C)

        if a is None:
            a = (0, 0)                                   # парковка у (0,0)
        if c is None:
            c = (self.n_cols - 1, self.n_rows - 1)       # склад в дальнем углу
        if b is None:
            b = (self.n_cols // 2, self.n_rows // 2)     # кубик в центре
            if b in (a, c):                              # не совпасть с A/C
                b = (max(0, self.n_cols - 1), 0)

        # Гарантируем различие точек (на маленькой сетке).
        used = []
        result = {}
        for name, cell in (('A', a), ('B', b), ('C', c)):
            cell = self._clamp_cell(cell)
            while cell in used:
                cell = self._next_free_cell(used)
            used.append(cell)
            result[name] = cell
        return result

    def _clamp_cell(self, cell):
        c = min(max(0, cell[0]), self.n_cols - 1)
        r = min(max(0, cell[1]), self.n_rows - 1)
        return (c, r)

    def _next_free_cell(self, used):
        for r in range(self.n_rows):
            for c in range(self.n_cols):
                if (c, r) not in used:
                    return (c, r)
        return (0, 0)

    # ========================================================================
    # Стенки комнаты
    # ========================================================================
    def _build_walls(self):
        """Стенки представляем набором перекрывающихся цилиндров вдоль 4 границ.
        Это даёт круговому RRT-планировщику непроходимый барьер по периметру,
        чтобы робот не выехал из комнаты."""
        obs = []
        r = self.wall_thickness / 2.0 + 0.001
        step = r * 1.5            # перекрытие цилиндров, чтобы не было щелей
        x_min = self.origin_x
        x_max = self.origin_x + self.room_w
        y_min = self.origin_y
        y_max = self.origin_y + self.room_h

        # Нижняя и верхняя стенки (вдоль X).
        x = x_min
        while x <= x_max + 1e-6:
            obs.append((x, y_min, r))
            obs.append((x, y_max, r))
            x += step
        # Левая и правая стенки (вдоль Y).
        y = y_min
        while y <= y_max + 1e-6:
            obs.append((x_min, y, r))
            obs.append((x_max, y, r))
            y += step
        return obs

    # ========================================================================
    # Виртуальные препятствия
    # ========================================================================
    def _build_virtual_obstacles(self):
        """В каждую ячейку, КРОМЕ A/B/C, кладём 0..max_per_cell цилиндров.
        Положение — со случайным отклонением внутри ячейки (но цилиндр целиком
        внутри границ ячейки). Радиус слегка варьируется.

        Проходимость гарантируется так:
          - цилиндр не выходит за границы своей ячейки (между ячейками всегда
            остаётся технологический зазор, не меньше робота);
          - точки A/B/C и их ближайшие окрестности свободны;
          - суммарно в ячейке не более 2 препятствий, и они разносятся, чтобы
            между ними оставался проезд шириной robot_radius*2+clearance."""
        reserved = set(self.points.values())
        # Требуемый свободный коридор для робота.
        passage = 2.0 * (self.robot_radius + self.clearance)

        obstacles = []
        for row in range(self.n_rows):
            for col in range(self.n_cols):
                if (col, row) in reserved:
                    continue
                x0, x1, y0, y1 = self._cell_bounds(col, row)

                # Сколько препятствий в этой ячейке (0..max_per_cell).
                k = random.randint(0, self.max_per_cell)
                if k == 0:
                    continue

                placed = []  # уже размещённые в этой ячейке (cx,cy,r)
                attempts = 0
                while len(placed) < k and attempts < 30:
                    attempts += 1
                    r = random.uniform(self.obst_r_min, self.obst_r_max)

                    # Цилиндр целиком внутри ячейки: центр не ближе r+зазор к
                    # границам ячейки, чтобы оставить коридор между ячейками.
                    margin = r + self.clearance
                    if (x1 - x0) <= 2 * margin or (y1 - y0) <= 2 * margin:
                        # Ячейка слишком мала для такого радиуса — уменьшим r.
                        r = max(self.obst_r_min,
                                min(r, (min(x1 - x0, y1 - y0) / 2.0) - self.clearance))
                        margin = r + self.clearance
                        if margin <= 0 or (x1 - x0) <= 2 * margin or (y1 - y0) <= 2 * margin:
                            continue

                    cx = random.uniform(x0 + margin, x1 - margin)
                    cy = random.uniform(y0 + margin, y1 - margin)

                    # Не сливаться с уже размещёнными в ячейке: между центрами
                    # должно хватать места роботу проехать.
                    ok = True
                    for (px, py, pr) in placed:
                        if math.hypot(cx - px, cy - py) < (r + pr + passage):
                            ok = False
                            break
                    # Держим препятствия подальше от точек A/B/C (на всякий
                    # случай, даже если ячейка не зарезервирована — сосед).
                    for (ax, ay) in self.point_xy.values():
                        if math.hypot(cx - ax, cy - ay) < (r + passage):
                            ok = False
                            break
                    if ok:
                        placed.append((cx, cy, r))

                obstacles.extend(placed)
        return obstacles

    # ========================================================================
    # Публикация
    # ========================================================================
    def _publish_params(self):
        rospy.set_param('/room/width', self.room_w)
        rospy.set_param('/room/height', self.room_h)
        rospy.set_param('/room/origin_x', self.origin_x)
        rospy.set_param('/room/origin_y', self.origin_y)
        rospy.set_param('/room/n_cols', self.n_cols)
        rospy.set_param('/room/n_rows', self.n_rows)
        rospy.set_param('/room/cell_w', self.cw)
        rospy.set_param('/room/cell_h', self.ch)
        rospy.set_param('/room/frame_id', self.frame)
        rospy.set_param('/room/seed', self.seed)

        # Границы движения для планировщика (с запасом на радиус робота, чтобы
        # центр робота не утыкался в стенку).
        m = self.robot_radius + self.clearance
        rospy.set_param('/room/bounds', [
            self.origin_x + m, self.origin_x + self.room_w - m,
            self.origin_y + m, self.origin_y + self.room_h - m
        ])

        # Точки A/B/C — и в ячейках, и в координатах.
        rospy.set_param('/room/point_A', list(self.point_xy['A']))  # парковка/home
        rospy.set_param('/room/point_B', list(self.point_xy['B']))  # кубик/pick
        rospy.set_param('/room/point_C', list(self.point_xy['C']))  # склад/place
        rospy.set_param('/room/cell_A', list(self.points['A']))
        rospy.set_param('/room/cell_B', list(self.points['B']))
        rospy.set_param('/room/cell_C', list(self.points['C']))

        # Полный список препятствий (стенки + виртуальные) как плоский список.
        flat = []
        for (cx, cy, r) in self.all_obstacles:
            flat.extend([round(cx, 4), round(cy, 4), round(r, 4)])
        rospy.set_param('/room/obstacles_flat', flat)
        # Только виртуальные (без стенок) — могут пригодиться отдельно.
        vflat = []
        for (cx, cy, r) in self.virtual_obstacles:
            vflat.extend([round(cx, 4), round(cy, 4), round(r, 4)])
        rospy.set_param('/room/virtual_obstacles_flat', vflat)

    def _publish_obstacles(self):
        msg = Float32MultiArray()
        data = []
        for (cx, cy, r) in self.all_obstacles:
            data.extend([float(cx), float(cy), float(r)])
        msg.data = data
        dim = MultiArrayDimension()
        dim.label = "obstacle_xyr"
        dim.size = len(self.all_obstacles)
        dim.stride = len(data)
        msg.layout.dim = [dim]
        self.obstacles_pub.publish(msg)

    def _publish_markers(self):
        arr = MarkerArray()
        mid = 0

        # Пол комнаты (полупрозрачный прямоугольник).
        floor = Marker()
        floor.header.frame_id = self.frame
        floor.header.stamp = rospy.Time.now()
        floor.ns = "room_floor"
        floor.id = mid; mid += 1
        floor.type = Marker.CUBE
        floor.action = Marker.ADD
        floor.pose.position.x = self.origin_x + self.room_w / 2.0
        floor.pose.position.y = self.origin_y + self.room_h / 2.0
        floor.pose.position.z = -0.01
        floor.pose.orientation.w = 1.0
        floor.scale.x = self.room_w
        floor.scale.y = self.room_h
        floor.scale.z = 0.02
        floor.color.r, floor.color.g, floor.color.b, floor.color.a = (0.85, 0.85, 0.85, 0.25)
        arr.markers.append(floor)

        # Стенки — 4 куба по периметру (наглядно для RViz/Gazebo-обзора).
        arr.markers.extend(self._wall_cube_markers(mid)); mid += 4

        # Линии сетки.
        arr.markers.append(self._grid_marker(mid)); mid += 1

        # Виртуальные препятствия — оранжевые полупрозрачные цилиндры
        # (подпись "VIRTUAL", т.к. в реальности их нет).
        for (cx, cy, r) in self.virtual_obstacles:
            arr.markers.append(self._cylinder_marker(mid, cx, cy, r,
                                                     (1.0, 0.55, 0.0, 0.55))); mid += 1

        # Точки A / B / C.
        colors = {'A': (1.0, 0.9, 0.1, 0.95),   # парковка — жёлтый
                  'B': (0.2, 0.4, 1.0, 0.95),   # кубик — синий
                  'C': (0.2, 1.0, 0.3, 0.95)}   # склад — зелёный
        labels = {'A': "A: PARK/HOME", 'B': "B: PICK", 'C': "C: PLACE"}
        for name in ('A', 'B', 'C'):
            x, y = self.point_xy[name]
            arr.markers.append(self._sphere_marker(mid, x, y, colors[name])); mid += 1
            arr.markers.append(self._text_marker(mid, x, y, labels[name])); mid += 1

        self.markers_pub.publish(arr)

    def _wall_cube_markers(self, start_id):
        out = []
        t = self.wall_thickness
        cx = self.origin_x + self.room_w / 2.0
        cy = self.origin_y + self.room_h / 2.0
        x_min = self.origin_x
        x_max = self.origin_x + self.room_w
        y_min = self.origin_y
        y_max = self.origin_y + self.room_h
        specs = [
            (cx, y_min, self.room_w + t, t),   # низ
            (cx, y_max, self.room_w + t, t),   # верх
            (x_min, cy, t, self.room_h + t),   # лево
            (x_max, cy, t, self.room_h + t),   # право
        ]
        for i, (px, py, sx, sy) in enumerate(specs):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = rospy.Time.now()
            m.ns = "room_walls"
            m.id = start_id + i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = self.wall_height / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = self.wall_height
            m.color.r, m.color.g, m.color.b, m.color.a = (0.4, 0.4, 0.45, 0.9)
            out.append(m)
        return out

    def _grid_marker(self, mid):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "room_grid"
        m.id = mid
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = (0.3, 0.3, 0.3, 0.5)
        x_min = self.origin_x
        x_max = self.origin_x + self.room_w
        y_min = self.origin_y
        y_max = self.origin_y + self.room_h
        for c in range(self.n_cols + 1):
            x = self.origin_x + c * self.cw
            m.points.append(Point(x, y_min, 0.01))
            m.points.append(Point(x, y_max, 0.01))
        for r in range(self.n_rows + 1):
            y = self.origin_y + r * self.ch
            m.points.append(Point(x_min, y, 0.01))
            m.points.append(Point(x_max, y, 0.01))
        return m

    def _cylinder_marker(self, mid, cx, cy, r, rgba):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "virtual_obstacles"
        m.id = mid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = cx
        m.pose.position.y = cy
        m.pose.position.z = 0.25
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = r * 2.0
        m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _sphere_marker(self, mid, x, y, rgba):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "abc_points"
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _text_marker(self, mid, x, y, text):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "abc_labels"
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.6
        m.pose.orientation.w = 1.0
        m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 1.0, 1.0, 1.0)
        m.text = text
        return m

    # ========================================================================
    @staticmethod
    def _fmt(p):
        return "(%.2f, %.2f)" % (p[0], p[1])


if __name__ == '__main__':
    try:
        node = RoomGenerator()
        rospy.loginfo("room_generator готов. Параметры в /room/*, "
                      "препятствия в /room/obstacles, маркеры в /room/markers.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
