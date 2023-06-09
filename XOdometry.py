"""
Competition Code for VRC: Over-Under (2024-2025)
Team: 3773P (Bowbots Phosphorus) from Bow NH
Author: Derek Baier (deekb on GitHub)
Project homepage: https://github.com/deekb/VRC-OverUnder
Project archive: https://github.com/deekb/VRC-OverUnder/archive/master.zip
Contact Derek.m.baier@gmail.com for more information

This module contains part of the competition code for the VRC (VEX Robotics Competition) Over-Under game,
for the 2024-2025 season.

The code is specifically developed for Team 3773P, known as Bowbots Phosphorus, and is authored by Derek Baier.

The project homepage and archive can be found on GitHub at the provided links.

For more information about the project, you can contact Derek Baier at the given email address.


Module dependencies:
- Utilities

Classes:
- Odometry: A class for tracking the position and rotation of an X-Drive robot based on wheel speeds.
    - __init__(self, brain, motor_1, motor_2, motor_3, motor_4, track_width_cm, wheel_circumference_cm,
    starting_location=(0, 0, 0), auto_update=True, gyroscope=False, wheel_rotation_offset_rad=0): Constructor method.
    - reset(self): Reset the robot's position and rotation.
    - set_velocities(self, motor_1_rpm=None, motor_2_rpm=None, motor_3_rpm=None, motor_4_rpm=None):
    Update the wheel speeds of the robot.
    - update_states(self, current_time): Update the position and rotation of the robot based on wheel speeds.
    - x: Property to get or set the robot's current x position.
    - y: Property to get or set the robot's current y position.
    - rotation_deg: Property to get or set the robot's current rotation in degrees.
    - rotation_rad: Property to get or set the robot's current rotation in radians.
    - position: Property to get or set the robot's current (x, y) position.
    - auto_update: Property to get or set the odometry's auto-update state.
"""


from Utilities import *


class Odometry:
    def __init__(self, brain, motor_1: Motor, motor_2: Motor, motor_3: Motor, motor_4: Motor, track_width_cm: float,
                 wheel_circumference_cm: float, starting_location: tuple = (0, 0, 0), auto_update: bool = True,
                 gyroscope: Inertial = False, wheel_rotation_offset_rad: float = 0):
        """
        A class for tracking the robot's position and rotation based on an initial position and a constant stream of motor velocities
        This implementation only works on X-drive, but the concept is similar to the tank drive implementation
        :param starting_location: The initial position for the robot (x_position, y_position, heading_in_radians) defaults to (0, 0, 0)
        :param track_width_cm: The parallel distance in centimeters between the centers of the wheels (Your drivetrain is a square? RIGHT!)
        :param wheel_circumference_cm: The circumference of the wheels, used to calculate the distance each wheel has travelled
        :param motor_1: Motor 1
        :type motor_1: Motor
        :param motor_2: Motor 2
        :type motor_2: Motor
        :param motor_3: Motor 3
        :type motor_3: Motor
        :param motor_4: Motor 4
        :type motor_4: Motor
        """
        self.timer = brain.timer
        self.terminal = Terminal(brain)
        self.print = self.terminal.print
        self.clear = self.terminal.clear

        # Define the distance between wheels, in centimeters
        self._track_width = track_width_cm
        self._wheel_circumference_cm = wheel_circumference_cm

        # Define the initial conditions of the robot
        self._x_position, self._y_position, self._current_rotation_rad = starting_location
        self._motor_1 = motor_1
        self._motor_2 = motor_2
        self._motor_3 = motor_3
        self._motor_4 = motor_4
        self._wheel_1_speed_distance_per_second = 0
        self._wheel_2_speed_distance_per_second = 0
        self._wheel_3_speed_distance_per_second = 0
        self._wheel_4_speed_distance_per_second = 0
        self._wheel_rotation_offset_rad = wheel_rotation_offset_rad
        self._previousTime = self.timer.time(SECONDS)
        self._auto_update = auto_update
        self._gyroscope = gyroscope
        if self._auto_update:
            self._auto_update_thread = Thread(self._auto_update_velocities)

    def reset(self) -> None:
        self._x_position = 0
        self._y_position = 0
        if self._gyroscope is not None:
            self._gyroscope.set_rotation(0)
        self._current_rotation_rad = 0
        self._previousTime = self.timer.time(SECONDS)
        self._wheel_1_speed_distance_per_second = 0
        self._wheel_2_speed_distance_per_second = 0
        self._wheel_3_speed_distance_per_second = 0
        self._wheel_4_speed_distance_per_second = 0

    def set_velocities(self, motor_1_rpm: float = None, motor_2_rpm: float = None, motor_3_rpm: float = None,
                       motor_4_rpm: float = None) -> None:
        """
        Drivetrain motors:
        //---------------\\
        ||1             2||
        ||               ||
        || Forward  >    ||
        ||               ||
        ||4             3||
        \\---------------//
        Updates the algorithm with the current wheel speeds (speeds are in RPM)
        :param motor_1_rpm: Motor 1's speed
        :param motor_2_rpm: Motor 2's speed
        :param motor_3_rpm: Motor 3's speed
        :param motor_4_rpm: Motor 4's speed
        """
        if motor_1_rpm is not None:
            self._wheel_1_speed_distance_per_second = motor_1_rpm / 60 * self._wheel_circumference_cm
        if motor_2_rpm is not None:
            self._wheel_2_speed_distance_per_second = motor_2_rpm / 60 * self._wheel_circumference_cm
        if motor_3_rpm is not None:
            self._wheel_3_speed_distance_per_second = motor_3_rpm / 60 * self._wheel_circumference_cm
        if motor_4_rpm is not None:
            self._wheel_4_speed_distance_per_second = motor_4_rpm / 60 * self._wheel_circumference_cm

    def update_states(self, current_time) -> None:
        delta_time = current_time - self._previousTime
        self._previousTime = current_time
        if self._gyroscope:
            self._current_rotation_rad = -math.radians(self._gyroscope.rotation(DEGREES))
        else:
            self._current_rotation_rad -= (((self._wheel_1_speed_distance_per_second +
                                             self._wheel_2_speed_distance_per_second +
                                             self._wheel_3_speed_distance_per_second +
                                             self._wheel_4_speed_distance_per_second) / self._track_width) / 2) * delta_time

        # wheel_pair_1_and_3_speed_delta_y = math.cos(math.radians(45) + self._current_rotation_rad) * (
        #         self._wheel_1_speed_distance_per_second - self._wheel_3_speed_distance_per_second) / 2
        # wheel_pair_1_and_3_speed_delta_x = math.sin(math.radians(45) + self._current_rotation_rad) * (
        #         self._wheel_1_speed_distance_per_second - self._wheel_3_speed_distance_per_second) / 2
        #
        # wheel_pair_2_and_4_speed_delta_y = math.cos(-math.pi / 4 + self._current_rotation_rad) * (
        #         self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second) / 2
        # wheel_pair_2_and_4_speed_delta_x = math.sin(-math.pi / 4 + self._current_rotation_rad) * (
        #         self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second) / 2

        x_speed = 0
        y_speed = 0

        x_speed += math.cos(self._current_rotation_rad + self._wheel_rotation_offset_rad) * (self._wheel_1_speed_distance_per_second - self._wheel_3_speed_distance_per_second)
        x_speed += math.sin(self._current_rotation_rad + self._wheel_rotation_offset_rad) * (self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second)

        y_speed += math.sin(self._current_rotation_rad + self._wheel_rotation_offset_rad) * (self._wheel_1_speed_distance_per_second - self._wheel_3_speed_distance_per_second)
        y_speed += math.cos(self._current_rotation_rad + self._wheel_rotation_offset_rad) * (self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second)

        self._x_position += x_speed * delta_time
        self._y_position += y_speed * delta_time

        # self._x_position -=
        # (wheel_pair_1_and_3_speed_delta_x + wheel_pair_2_and_4_speed_delta_x) * delta_time
        # self._y_position +=
        # (wheel_pair_1_and_3_speed_delta_y + wheel_pair_2_and_4_speed_delta_y) * delta_time

    @property
    def x(self) -> float:
        """
        Get the robot's current x position
        """
        return self._x_position

    @x.setter
    def x(self, x_position) -> None:
        """
        Set the robot's current x position
        :param x_position: The new x position
        """
        self._x_position = x_position

    @property
    def y(self) -> float:
        """
        Get the robot's current y position
        """
        return self._y_position

    @y.setter
    def y(self, y_position) -> None:
        """
        Set the robot's current y position
        :param y_position: The new y position
        """
        self._y_position = y_position

    @property
    def rotation_deg(self) -> float:
        """
        Get the robot's current rotation in degrees
        """
        return math.degrees(self._current_rotation_rad)

    @rotation_deg.setter
    def rotation_deg(self, rotation_degrees) -> None:
        """
        Set the robot's current rotation in degrees
        :param rotation_degrees: The new rotation
        """
        self._current_rotation_rad = math.radians(rotation_degrees)

    @property
    def rotation_rad(self) -> float:
        """
        Get the robot's current rotation in radians
        """
        return self._current_rotation_rad

    @rotation_rad.setter
    def rotation_rad(self, rotation_radians) -> None:
        """
        Set the robot's current rotation in radians
        :param rotation_radians: The new rotation
        """
        self._current_rotation_rad = rotation_radians

    @property
    def position(self) -> tuple:
        """
        Get the robot's current (x, y) position
        :rtype: tuple[float, float]
        """
        return self._x_position, self._y_position

    @position.setter
    def position(self, coordinates) -> None:
        """
        Set the robot's current (x, y) position
        :param coordinates: The new position
        """
        self._x_position, self._y_position = coordinates

    @property
    def auto_update(self) -> bool:
        """
        Get the odometry's auto-update state
        """
        return self._auto_update

    @auto_update.setter
    def auto_update(self, value) -> None:
        """
        Set the odometry's auto-update state
        :param value: The new state
        """
        self._auto_update = value
        if self._auto_update:
            if not self._auto_update_thread.isrunning():
                self._previousTime = self.timer.time(MSEC) / 1000  # Set the last update time to now
                # avoids situations where delta_time is extremely high
                # after pausing auto_update_velocities for long periods of time
                self._auto_update_thread = Thread(self._auto_update)
        else:
            self._auto_update_thread.stop()

    def _auto_update_velocities(self) -> None:
        """
        Used internally to constantly update the wheel states, do not call from outside this class
        """
        while True:
            if self._auto_update:
                self.update_states(self.timer.time(MSEC) / 1000)
                self.set_velocities(self._motor_1.velocity(), self._motor_2.velocity(), self._motor_3.velocity(),
                                    self._motor_4.velocity())
            else:
                self.set_velocities(0, 0, 0, 0)
                self._previousTime = self.timer.time(MSEC) / 1000
            wait(5)
