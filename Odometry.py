"""
A class for tracking an X-Drive robot using the wheel speeds
"""

from vex import *
from Utilities import *


class XDriveDrivetrainOdometry:
    # noinspection GrazieInspection
    def __init__(self, brain, motor_1: Motor, motor_2: Motor, motor_3: Motor, motor_4: Motor, track_width_cm: float,
                 wheel_circumference_cm: float, starting_location: tuple = (0, 0, 0), auto_update: bool = True,
                 gyroscope: Inertial = False):
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
        self._previousTime = brain.timer.time(SECONDS)
        self._auto_update = auto_update
        self._gyroscope = gyroscope
        if self._auto_update:
            self._auto_update_thread = Thread(self.auto_update_velocities)

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
        Updates the algorithm with the current wheel speeds
        :param motor_1_rpm: Motor 1's speed in revolutions per second
        :param motor_2_rpm: Motor 2's speed in revolutions per second
        :param motor_3_rpm: Motor 3's speed in revolutions per second
        :param motor_4_rpm: Motor 4's speed in revolutions per second
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

        wheel_pair_1_and_3_speed_delta_y = math.cos(math.pi / 4 + self._current_rotation_rad) * (
                self._wheel_1_speed_distance_per_second - self._wheel_3_speed_distance_per_second) / 2
        wheel_pair_1_and_3_speed_delta_x = math.sin(math.pi / 4 + self._current_rotation_rad) * (
                self._wheel_1_speed_distance_per_second - self._wheel_3_speed_distance_per_second) / 2
        wheel_pair_2_and_4_speed_delta_y = math.cos(-math.pi / 4 + self._current_rotation_rad) * (
                self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second) / 2
        wheel_pair_2_and_4_speed_delta_x = math.sin(-math.pi / 4 + self._current_rotation_rad) * (
                self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second) / 2

        self._x_position -= (wheel_pair_1_and_3_speed_delta_x + wheel_pair_2_and_4_speed_delta_x) * delta_time
        self._y_position += (wheel_pair_1_and_3_speed_delta_y + wheel_pair_2_and_4_speed_delta_y) * delta_time

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
                self._auto_update_thread = Thread(self._auto_update)
        else:
            self._auto_update_thread.stop()

    def auto_update_velocities(self) -> None:
        """
        Used internally to constantly update the wheel states, do not call from outside this class
        """
        while True:
            if self.auto_update:
                self.update_states(self.timer.time(MSEC) / 1000)
                self.set_velocities(self._motor_1.velocity(), self._motor_2.velocity(), self._motor_3.velocity(),
                                    self._motor_4.velocity())
            else:
                self.set_velocities(0, 0, 0, 0)
                self._previousTime = self.timer.time(MSEC) / 1000
            wait(5)


class TankDriveDrivetrainOdometry:
    # noinspection GrazieInspection
    def __init__(self, brain, motor_1: Motor, motor_2: Motor, motor_3: Motor, motor_4: Motor, track_width_cm: float,
                 wheel_circumference_cm: float, starting_location: tuple = (0, 0, 0), auto_update: bool = True,
                 gyroscope: Inertial = False):
        """
        A class for tracking the robot's position and rotation based on an initial position and a constant stream of motor velocities
        This implementation only works on Tank-drive
        :param starting_location: The initial position for the robot (x_position, y_position, heading_in_radians) defaults to (0, 0, 0)
        :param track_width_cm: The parallel distance in centimeters between the centers of the wheels
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
        self._previousTime = brain.timer.time(SECONDS)
        self._auto_update = auto_update
        self._gyroscope = gyroscope
        if self._auto_update:
            self._auto_update_thread = Thread(self.auto_update_velocities)

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
        ||1             3||
        ||       ^       ||
        ||    Forward    ||
        ||               ||
        ||2             4||
        \\---------------//
        Updates the algorithm with the current wheel speeds
        :param motor_1_rpm: Motor 1's speed in revolutions per second
        :param motor_2_rpm: Motor 2's speed in revolutions per second
        :param motor_3_rpm: Motor 3's speed in revolutions per second
        :param motor_4_rpm: Motor 4's speed in revolutions per second
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

        wheel_pair_1_and_3_speed_delta_y = math.cos(math.pi / 4 + self._current_rotation_rad) * (
                self._wheel_1_speed_distance_per_second - self._wheel_3_speed_distance_per_second) / 2
        wheel_pair_1_and_3_speed_delta_x = math.sin(math.pi / 4 + self._current_rotation_rad) * (
                self._wheel_1_speed_distance_per_second - self._wheel_3_speed_distance_per_second) / 2
        wheel_pair_2_and_4_speed_delta_y = math.cos(-math.pi / 4 + self._current_rotation_rad) * (
                self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second) / 2
        wheel_pair_2_and_4_speed_delta_x = math.sin(-math.pi / 4 + self._current_rotation_rad) * (
                self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second) / 2

        self._x_position -= (wheel_pair_1_and_3_speed_delta_x + wheel_pair_2_and_4_speed_delta_x) * delta_time
        self._y_position += (wheel_pair_1_and_3_speed_delta_y + wheel_pair_2_and_4_speed_delta_y) * delta_time

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
                self._auto_update_thread = Thread(self._auto_update)
        else:
            self._auto_update_thread.stop()

    def auto_update_velocities(self) -> None:
        """
        Used internally to constantly update the wheel states, do not call from outside this class
        """
        while True:
            if self.auto_update:
                self.update_states(self.timer.time(MSEC) / 1000)
                self.set_velocities(self._motor_1.velocity(), self._motor_2.velocity(), self._motor_3.velocity(),
                                    self._motor_4.velocity())
            else:
                self.set_velocities(0, 0, 0, 0)
                self._previousTime = self.timer.time(MSEC) / 1000
            wait(5)
