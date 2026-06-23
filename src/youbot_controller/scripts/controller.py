#!/usr/bin/env python3
import math

class PID:
    def __init__(self, kp, ki, kd, dt, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limits = output_limits
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if output > self.limits[1]:
            output = self.limits[1]
            self.integral -= error * self.dt
        elif output < self.limits[0]:
            output = self.limits[0]
            self.integral -= error * self.dt
        self.prev_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class Controller:
    def __init__(self, linear_gains=(0.8, 0.0, 0.1),
                 angular_gains=(1.5, 0.0, 0.2),
                 arm_gains=(1.0, 0.0, 0.0),
                 dt=0.02):
        self.pid_linear = PID(*linear_gains, dt=dt, output_limits=(0.0, 0.5))
        self.pid_angular = PID(*angular_gains, dt=dt, output_limits=(-1.5, 1.5))
        self.arm_gains = arm_gains
        self.dt = dt

    def compute_base_control(self, current_pose, desired_pose):
        dx = desired_pose['x'] - current_pose['x']
        dy = desired_pose['y'] - current_pose['y']
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        theta_error = angle_to_target - current_pose['theta']
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

        # ВАЖНО: setpoint = цель, measurement = 0 -> ошибка положительна,
        # робот едет К цели, скорость падает по мере приближения.
        linear_vel = self.pid_linear.compute(distance, 0.0)
        angular_vel = self.pid_angular.compute(theta_error, 0.0)

        # Пока робот сильно отвёрнут от цели — почти не едем вперёд, доворачиваемся
        if abs(theta_error) > 0.6:
            linear_vel *= 0.15

        # Приехали
        if distance < 0.05:
            linear_vel = 0.0
            angular_vel = 0.0

        return linear_vel, 0.0, angular_vel

    def compute_arm_control(self, current_arm_angles, desired_arm_angles):
        return desired_arm_angles[:]

    def compute_gripper_control(self, current_gripper, desired_gripper):
        return desired_gripper