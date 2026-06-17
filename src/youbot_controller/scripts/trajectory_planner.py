#!/usr/bin/env python3
import math
import rospy

# Планировщик траектории, возвращающий желаемое состояние в зависимости от времени.
#   Состояние включает: положение базы (x, y, theta), углы руки (5 значений),
#   раскрытие схвата.
class TrajectoryPlanner:
    def __init__(self):
        # Параметры миссии (можно переопределить через rosparam)
        self.object_x = rospy.get_param('~object_x', 1.5)
        self.object_y = rospy.get_param('~object_y', 1.5)
        self.approach_distance = rospy.get_param('~approach_distance', 0.5)

        # Целевые конфигурации руки (углы в радианах)
        # Все углы заданы в порядке: [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5]
        self.home_angles = [0.01, 0.01, -0.1, 0.01, 0.01] # сложенное положение
        self.grasp_angles = [0.5, 0.8, -1.2, 0.3, 0.0]    # положение для захвата
        self.lift_angles = [0.5, 0.8, -0.5, 0.8, 0.0]     # поднятое положение
        self.place_angles = [0.5, 0.8, -0.2, -0.5, 0.0]   # положение для постановки

        # Параметры схвата
        self.gripper_opened = rospy.get_param('~gripper_open', 0.02)           # открыт (м)
        self.gripper_closed = rospy.get_param('~gripper_closed', 0.0)          # закрыт (м)
        self.gripper_close_time = rospy.get_param('~gripper_close_time', 1.5)  # время закрытия (сек)
        self.gripper_open_time = rospy.get_param('~gripper_open_time', 1.0)    # время открытия (сек)

        # Временные метки для фаз (относительные)
        self.t_approach_start = 0.0
        self.t_reach_start = None # устанавливается динамически
        self.t_grasp_start = None
        self.t_lift_start = None
        self.t_retreat_start = None
        self.t_return_start = None
        self.t_place_start = None

        # Параметры движения
        self.amplitude = rospy.get_param('~amplitude', 0.2) # амплитуда синусоидального бокового смещения
        self.zigzag_amplitude = rospy.get_param('~zigzag_amplitude', 0.15) # амплитуда зигзага при отъезде

        # Инициализация времени начала
        self.start_time = rospy.Time.now().to_sec()

        rospy.loginfo("Trajectory Planner initialized with object at (%.2f, %.2f)", self.object_x, self.object_y)

    # Вспомогательный метод
    # Линейная интерполяция между двумя наборами углов
    def _interpolate_angles(self, start_angles, end_angles, progress):
        return [start_angles[i] + progress * (end_angles[i] - start_angles[i]) for i in range(5)]

    # Основной метод
    #Возвращает желаемое состояние в момент времени t (сек от начала миссии).
    def get_desired_state(self, t):
        # Если не заданы времена начала фаз, устанавливаем их
        if self.t_reach_start is None:
            self.t_reach_start = self.t_approach_start + 8.0
            self.t_grasp_start = self.t_reach_start + 3.0
            self.t_lift_start = self.t_grasp_start + 1.5
            self.t_retreat_start = self.t_lift_start + 2.0
            self.t_return_start = self.t_retreat_start + 5.0
            self.t_place_start = self.t_return_start + 8.0

        # Определяем текущую фазу по времени
        if t < self.t_reach_start:
            self.current_phase = 'APPROACH'
            return self._approach_phase(t)
        elif t < self.t_grasp_start:
            self.current_phase = 'REACH'
            return self._reach_phase(t)
        elif t < self.t_lift_start:
            self.current_phase = 'GRASP'
            return self._grasp_phase(t)
        elif t < self.t_retreat_start:
            self.current_phase = 'LIFT'
            return self._lift_phase(t)
        elif t < self.t_return_start:
            self.current_phase = 'RETREAT'
            return self._retreat_phase(t)
        elif t < self.t_place_start:
            self.current_phase = 'RETURN'
            return self._return_phase(t)
        else:
            self.current_phase = 'PLACE'
            return self._place_phase(t)

    # Фаза 1: Подъезд к объекту
    # Траектория с синусоидальным смещением, манипулятор постепенно переходит из home в положение захвата)
    def _approach_phase(self, t):
        # Время нормируем от начала фазы до её конца
        t_local = t - self.t_approach_start
        duration = self.t_reach_start - self.t_approach_start
        if duration <= 0:
            duration = 1.0
        progress = min(t_local / duration, 1.0)

        # Расчёт позиции платформы: прямолинейное движение к объекту + боковое смещение
        # Центр дуги - на полпути к объекту
        mid_x = self.object_x / 2.0
        mid_y = self.object_y / 2.0

        # Добавляем синусоидальное боковое смещение (в перпендикулярном направлении)
        # Смещение зависит от progress и времени
        lateral = self.amplitude * math.sin(2.0 * math.pi * progress * 1.5) * (1 - progress)

        # Прямолинейная интерполяция с добавлением синусоидального отклонения
        x_base = mid_x * progress  # линейно растем до середины, потом к объекту
        y_base = mid_y * progress
        
        # Корректируем x,y: добавляем смещение перпендикулярно направлению движения
        # Для простоты добавим смещение по Y (если цель по диагонали, это будет не совсем перпендикулярно, но для демонстрации сойдёт)
        x_des = x_base + lateral * 0.5
        y_des = y_base + lateral * 0.5

        # Желаемая ориентация: смотреть на объект (если мы не на месте)
        if self.object_x - x_des != 0 or self.object_y - y_des != 0:
            theta_des = math.atan2(self.object_y - y_des, self.object_x - x_des)
        else:
            theta_des = math.atan2(self.object_y, self.object_x)

        # Плавный переход манипулятора из home в положение захвата
        arm_angles = self._interpolate_angles(self.home_angles, self.grasp_angles, progress)
        gripper = self.gripper_opened  # открыт

        return {'x': x_des, 'y': y_des, 'theta': theta_des,
                'arm_angles': arm_angles, 'gripper': gripper}

    # Фаза 2: Выдвижение манипулятора к объекту
    # Платформа стоит на месте, манипулятор фиксируется в положении захвата
    def _reach_phase(self, t):
        # Продолжаем интерполяцию, чтобы манипулятор достиг точных углов захвата
        t_local = t - self.t_reach_start
        duration = self.t_grasp_start - self.t_reach_start
        if duration <= 0:
            duration = 1.0
        progress = min(t_local / duration, 1.0)

        arm_angles = self._interpolate_angles(self.grasp_angles, self.grasp_angles, progress)  # фиксируем

        # Платформа остаётся на месте (держим позицию, где остановились)
        # Возвращаем ту же позицию, что была в конце APPROACH
        # Для простоты зафиксируем позицию на объекте с небольшим отступом
        x_des = self.object_x - 0.3 * math.cos(math.atan2(self.object_y, self.object_x))
        y_des = self.object_y - 0.3 * math.sin(math.atan2(self.object_y, self.object_x))
        theta_des = math.atan2(self.object_y, self.object_x)
        gripper = self.gripper_opened

        return {'x': x_des, 'y': y_des, 'theta': theta_des,
                'arm_angles': arm_angles, 'gripper': gripper}

    # Фаза 3: Захват объект
    # Схват закрывается, манипуялтор и платформа остаются на месте
    def _grasp_phase(self, t):
        t_local = t - self.t_grasp_start
        progress = min(t_local / self.gripper_close_time, 1.0)

        # Манипулятор остаётся в положении захвата объекта, схват закрывается
        gripper = self.gripper_opened * (1.0 - progress)
        arm_angles = self.grasp_angles[:]
        
        # Позиция платформы и ориентация такие же, как в REACH
        x_des = self.object_x - 0.3 * math.cos(math.atan2(self.object_y, self.object_x))
        y_des = self.object_y - 0.3 * math.sin(math.atan2(self.object_y, self.object_x))
        theta_des = math.atan2(self.object_y, self.object_x)

        return {'x': x_des, 'y': y_des, 'theta': theta_des,
                'arm_angles': arm_angles, 'gripper': gripper}

    # Фаза 4: Вывод манипулятора со схваченным объектом
    # Манипулятор переходит из положения захвата в поднятое положение
    def _lift_phase(self, t):
        t_local = t - self.t_lift_start
        duration = self.t_retreat_start - self.t_lift_start
        if duration <= 0:
            duration = 1.0
        progress = min(t_local / duration, 1.0)

        # Интерполяция от углов захвата к углам подъёма
        arm_angles = self._interpolate_angles(self.grasp_angles, self.lift_angles, progress)
        gripper = self.gripper_closed  # схват закрыт (держим объект)

        # Платформа на месте
        x_des = self.object_x - 0.3 * math.cos(math.atan2(self.object_y, self.object_x))
        y_des = self.object_y - 0.3 * math.sin(math.atan2(self.object_y, self.object_x))
        theta_des = math.atan2(self.object_y, self.object_x)

        return {'x': x_des, 'y': y_des, 'theta': theta_des,
                'arm_angles': arm_angles, 'gripper': gripper}

    # Фаза 5: Отъезд от объекта (с зигзагами)
    # Движение назад с зигзагообразной траекторией
    def _retreat_phase(self, t):
        t_local = t - self.t_retreat_start
        duration = self.t_return_start - self.t_retreat_start
        if duration <= 0:
            duration = 1.0
        progress = min(t_local / duration, 1.0)

        # Начальная точка (где мы стояли)
        start_x = self.object_x - 0.3 * math.cos(math.atan2(self.object_y, self.object_x))
        start_y = self.object_y - 0.3 * math.sin(math.atan2(self.object_y, self.object_x))

        # Конечная точка: отъезжаем назад (в направлении, обратном начальному)
        angle_to_object = math.atan2(self.object_y, self.object_x)
        retreat_direction = angle_to_object + math.pi  # противоположное направление
        retreat_distance = 0.8
        end_x = start_x + retreat_distance * math.cos(retreat_direction) + 0.3 * math.sin(retreat_direction)
        end_y = start_y + retreat_distance * math.sin(retreat_direction) - 0.3 * math.cos(retreat_direction)

        # Зигзаг: добавляем синусоидальное смещение перпендикулярно направлению движения
        # Направление движения (вектор)
        dx = end_x - start_x
        dy = end_y - start_y
        length = math.hypot(dx, dy)
        # Перпендикулярный вектор
        if length > 0:
            perp_x = -dy / length
            perp_y = dx / length
        else:
            perp_x, perp_y = 0.0, 0.0
        zigzag = self.zigzag_amplitude * math.sin(2.0 * math.pi * progress * 3.0)

        # Интерполяция с добавлением зигзага (боковое смещение)
        x_des = start_x + progress * (end_x - start_x) + zigzag * perp_x
        y_des = start_y + progress * (end_y - start_y) + zigzag * perp_y

        # Желаемая ориентация: смотреть в направлении движения
        theta_des = math.atan2(dy, dx)
        arm_angles = self.lift_angles[:]
        gripper = self.gripper_closed

        return {'x': x_des, 'y': y_des, 'theta': theta_des,
                'arm_angles': arm_angles, 'gripper': gripper}

    # Фаза 6: Возврат в исходную точку (0,0,0) с коррекцией ориентации
    # Движение в (0,0,0) с синусоидальным отклонением
    def _return_phase(self, t):
        t_local = t - self.t_return_start
        duration = self.t_place_start - self.t_return_start
        if duration <= 0:
            duration = 1.0
        progress = min(t_local / duration, 1.0)

        # Прямолинейная интерполяция
        # Начальная точка: где мы оказались в конце RETREAT
        start_x = self.object_x
        start_y = self.object_y
        end_x, end_y = 0.0, 0.0

        # Интерполяция с синусоидальным отклонением для сложности
        x_des = start_x + progress * (end_x - start_x) + 0.1 * math.sin(2.0 * math.pi * progress * 2.0)
        y_des = start_y + progress * (end_y - start_y) + 0.1 * math.cos(2.0 * math.pi * progress * 2.0)

        # Желаемая ориентация: 0 (смотреть вдоль оси X)
        theta_des = 0.0
        arm_angles = self.lift_angles[:]
        gripper = self.gripper_closed

        return {'x': x_des, 'y': y_des, 'theta': theta_des,
                'arm_angles': arm_angles, 'gripper': gripper}

    # Фаза 7: Постановка объекта на пол
    def _place_phase(self, t):
        t_local = t - self.t_place_start
        progress = min(t_local / 2.0, 1.0)

        # Фиксируем желаемые углы для постановки
        arm_angles = self._interpolate_angles(self.lift_angles, self.place_angles, progress)

        # Через некоторое время открываем схват
        if t_local < self.gripper_open_time:
            gripper = self.gripper_closed  # закрыт (ещё не отпускаем)
        else:
            gripper = self.gripper_opened  # открываем

        # База остаётся на месте (0,0)
        x_des, y_des, theta_des = 0.0, 0.0, 0.0

        # После открытия схвата можно завершить миссию, но мы просто останемся в этом состоянии.
        return {'x': x_des, 'y': y_des, 'theta': theta_des,
                'arm_angles': arm_angles, 'gripper': gripper}