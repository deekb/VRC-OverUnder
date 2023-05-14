import math
from vex import *


class XDriveDrivetrainOdometry:
    def __init__(self, track_width_cm: float, wheel_diameter_cm: float, starting_location: tuple = (0, 0, 0)):
        """
        A class for tracking the robot's position and rotation based on an initial position and a constant stream of motor velocities
        :param starting_location: The initial position for the robot (x_position, y_position, heading_in_radians) defaults to (0, 0, 0)
        :param track_width_cm: The parallel distance in centimeters between the centers of the wheels (Your drivetrain is a square? RIGHT!)
        :param wheel_diameter_cm: The diameter of the wheels, used to calculate the distance each wheel has travelled
        """
        # Define the distance between wheels, in centimeters
        self._track_width = track_width_cm
        self._cm_per_wheel_revolution = wheel_diameter_cm * math.pi
        self._cm_per_wheel_degree = self._cm_per_wheel_revolution / 360

        # Define the initial conditions of the robot
        self._x_position, self._y_position, self._current_rotation_rad = starting_location
        self._wheel_1_speed = 0
        self._wheel_2_speed = 0
        self._wheel_3_speed = 0
        self._wheel_4_speed = 0
        self._previousTime = time.time()

    def set_velocities(self, motor_1_speed: float = None, motor_2_speed: float = None, motor_3_speed: float = None,
                       motor_4_speed: float = None):
        """
        Drivetrain motors:
        //---------------\\
        ||1             2||
        ||       ^       ||
        ||    Forward    ||
        ||               ||
        ||4             3||
        \\---------------//
        Updates the algorithm with the current wheel speeds
        :param motor_1_speed: Motor 1's speed in revolutions per second
        :param motor_2_speed: Motor 2's speed in revolutions per second
        :param motor_3_speed: Motor 3's speed in revolutions per second
        :param motor_4_speed: Motor 4's speed in revolutions per second
        """
        if motor_1_speed is not None:
            self._wheel_1_speed = motor_1_speed * self._cm_per_wheel_revolution
        if motor_2_speed is not None:
            self._wheel_2_speed = motor_2_speed * self._cm_per_wheel_revolution
        if motor_3_speed is not None:
            self._wheel_3_speed = motor_3_speed * self._cm_per_wheel_revolution
        if motor_4_speed is not None:
            self._wheel_4_speed = motor_4_speed * self._cm_per_wheel_revolution

    def update_states(self, current_time):
        delta_time = current_time - self._previousTime
        self._previousTime = current_time
        self._current_rotation_rad += (-(self._wheel_1_speed + self._wheel_2_speed + self._wheel_3_speed + self._wheel_4_speed) * delta_time / self._track_width)
        forward_backward_speed = (self._wheel_3_speed - self._wheel_1_speed) / 2
        left_right_speed = (self._wheel_2_speed - self._wheel_4_speed) / 2
        self._x_position += ((forward_backward_speed * math.cos(self._current_rotation_rad) + left_right_speed * math.sin(self._current_rotation_rad)) * delta_time)
        self._y_position += ((left_right_speed * math.sin(self._current_rotation_rad + math.pi / 2) + forward_backward_speed * math.cos(self._current_rotation_rad + math.pi / 2)) * delta_time)

    def get_x(self):
        return self._x_position

    def get_y(self):
        return self._y_position

    def get_position(self):
        return self._x_position, self._y_position

    def get_rotation_deg(self):
        return math.degrees(self._current_rotation_rad)

    def set_rotation_deg(self, rotation):
        self._current_rotation_rad = math.radians(rotation)

    def set_rotation_rad(self, rotation):
        self._current_rotation_rad = rotation

    def get_rotation_rad(self):
        return self._current_rotation_rad

    def set_position(self, x_position=None, y_position=None, direction_rad=None):
        if x_position is not None:
            self._x_position = x_position
        if y_position is not None:
            self._y_position = y_position
        if direction_rad is not None:
            self._current_rotation_rad = direction_rad

    def auto_update_velocities(self, motor_1, motor_2, motor_3, motor_4):
        while True:
            self.set_velocities(motor_1.velocity(), motor_2.velocity(), motor_3.velocity, motor_4.velocity())
            wait(5)
