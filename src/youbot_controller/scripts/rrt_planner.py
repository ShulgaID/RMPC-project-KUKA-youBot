#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# rrt_planner.py — RRT для 2D с круговыми препятствиями.
#
# Что исправлено по сравнению с прежней версией:
#   1. УЧЁТ ГАБАРИТА РОБОТА. Препятствия «раздуваются» на robot_radius
#      (инфляция конфигурационного пространства). Раньше проверка шла по
#      «голому» радиусу препятствия, и центр робота проходил впритык —
#      корпус задевал препятствие. Теперь между корпусом и препятствием
#      гарантирован зазор.
#   2. GOAL BIAS. С вероятностью goal_bias выборка тянется прямо к цели —
#      дерево быстрее и прямее доходит до цели, путь чище.
#   3. БЕЗОПАСНОЕ СГЛАЖИВАНИЕ. Метод shortcut() убирает лишние зигзаги RRT,
#      проверяя КАЖДЫЙ срезанный отрезок на коллизию. Это заменяет слепой
#      кубический сплайн, который раньше выгибался ВНУТРЬ препятствий.
#   4. Плотная дискретизация проверки отрезка (по шагу), а не только концов.
# =============================================================================

import random
import math


class RRTPlanner:
    def __init__(self, start, goal, obstacles, bounds,
                 max_iter=4000, step_size=0.25,
                 robot_radius=0.30, goal_bias=0.15, safety_margin=0.05):
        self.start = tuple(start)
        self.goal = tuple(goal)
        # Инфлируем препятствия на габарит робота + запас.
        infl = robot_radius + safety_margin
        self.obstacles = [(cx, cy, r + infl) for (cx, cy, r) in obstacles]
        self.bounds = bounds                       # (xmin, xmax, ymin, ymax)
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.check_res = max(0.05, step_size / 4.0)  # шаг проверки отрезка

        self.nodes = [self.start]
        self.parent = {self.start: None}

    # ------------------------------------------------------------------ plan
    def plan(self):
        # Если прямой отрезок start->goal свободен — сразу его и возвращаем.
        if self._segment_free(self.start, self.goal):
            return self.shortcut([self.start, self.goal])

        for _ in range(self.max_iter):
            rand_point = (self.goal if random.random() < self.goal_bias
                          else self._random_point())
            nearest = self._nearest(rand_point)
            new_point = self._steer(nearest, rand_point)
            if new_point in self.parent:
                continue
            if self._segment_free(nearest, new_point):
                self.nodes.append(new_point)
                self.parent[new_point] = nearest
                if self._distance(new_point, self.goal) < self.step_size and \
                        self._segment_free(new_point, self.goal):
                    self.parent[self.goal] = new_point
                    path = self._build_path(self.goal)
                    return self.shortcut(path)
        return []   # путь не найден

    # --------------------------------------------------------------- helpers
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
        if dist < 1e-9:
            return from_point
        if dist < self.step_size:
            return to_point
        ratio = self.step_size / dist
        return (from_point[0] + dx * ratio, from_point[1] + dy * ratio)

    @staticmethod
    def _distance(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def _point_free(self, p):
        for (cx, cy, r) in self.obstacles:
            if self._distance(p, (cx, cy)) < r:
                return False
        return True

    def _segment_free(self, p1, p2):
        # Плотная проверка отрезка с шагом check_res (не только концы).
        d = self._distance(p1, p2)
        n = max(1, int(math.ceil(d / self.check_res)))
        for i in range(n + 1):
            t = i / n
            px = p1[0] + t * (p2[0] - p1[0])
            py = p1[1] + t * (p2[1] - p1[1])
            if not self._point_free((px, py)):
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

    # ------------------------------------------------------------- shortcut
    def shortcut(self, path, iterations=200):
        """Убирает лишние промежуточные точки: если отрезок между двумя
        несоседними точками свободен — выкидываем промежуточные. Гарантирует,
        что итоговый путь НЕ задевает препятствия (в отличие от сплайна)."""
        if len(path) < 3:
            return path[:]
        path = path[:]
        for _ in range(iterations):
            if len(path) < 3:
                break
            i = random.randint(0, len(path) - 3)
            j = random.randint(i + 2, len(path) - 1)
            if self._segment_free(path[i], path[j]):
                path = path[:i + 1] + path[j:]
        return path
