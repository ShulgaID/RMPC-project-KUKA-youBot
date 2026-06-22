#!/usr/bin/env python3
import random
import math

#Простой RRT-планировщик для 2D-пространства с круговыми препятствиями
class RRTPlanner:

    def __init__(self, start, goal, obstacles, bounds, max_iter=500, step_size=0.2):
        self.start = start # начальная точка
        self.goal = goal # целевая точка
        self.obstacles = obstacles # список (cx, cy, radius)
        self.bounds = bounds # (xmin, xmax, ymin, ymax) границы
        self.max_iter = max_iter # максимальное число итераций
        self.step_size = step_size # шаг расширения дерева
        self.nodes = [start]
        self.parent = {start: None}

    # Запускает RRT и возвращает путь в виде списка точек (от start до goal), или пустой список
    def plan(self):
        for _ in range(self.max_iter):
            rand_point = self._random_point()
            nearest = self._nearest(rand_point)
            new_point = self._steer(nearest, rand_point)
            if self._is_collision_free(nearest, new_point):
                self.nodes.append(new_point)
                self.parent[new_point] = nearest
                if self._distance(new_point, self.goal) < self.step_size:
                    return self._build_path(new_point)
        return []

    def _random_point(self):
        x = random.uniform(self.bounds[0], self.bounds[1])
        y = random.uniform(self.bounds[2], self.bounds[3])
        return (x, y)

    def _nearest(self, point):
        return min(self.nodes, key=lambda p: self._distance(p, point))

    def _steer(self, from_point, to_point):
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        dist = math.hypot(dx, dy)
        if dist < self.step_size:
            return to_point
        ratio = self.step_size / dist
        return (from_point[0] + dx * ratio, from_point[1] + dy * ratio)

    def _distance(self, p1, p2):
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    # Проверяет, свободен ли отрезок от столкновений с препятствиями
    def _is_collision_free(self, p1, p2):
        for (cx, cy, r) in self.obstacles:
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            if dx == 0 and dy == 0:
                if self._distance(p1, (cx, cy)) < r:
                    return False
                continue
            t = ((cx - p1[0])*dx + (cy - p1[1])*dy) / (dx*dx + dy*dy)
            t = max(0, min(1, t))
            px = p1[0] + t*dx
            py = p1[1] + t*dy
            if self._distance((px, py), (cx, cy)) < r:
                return False
        return True

    def _build_path(self, goal_node):
        path = []
        current = goal_node
        while current is not None:
            path.append(current)
            current = self.parent[current]
        path.reverse()
        return path