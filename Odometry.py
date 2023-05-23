import math
from vex import *

brain = Brain()


# noinspection GrazieInspection
class XDriveDrivetrainOdometry:
    # noinspection GrazieInspection
    def __init__(self, motor_1: Motor, motor_2: Motor, motor_3: Motor, motor_4: Motor, track_width_cm: float, wheel_circumference_cm: float, starting_location: tuple = (0, 0, 0)):
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
        self._auto_update = True
        self._auto_update_thread = Thread(self._auto_update)

        self._motor_3.set_position()

    def reset(self):
        self._x_position = 0
        self._y_position = 0
        self._current_rotation_rad = 0
        self._previousTime = brain.timer.time(SECONDS)
        self._wheel_1_speed_distance_per_second = 0
        self._wheel_2_speed_distance_per_second = 0
        self._wheel_3_speed_distance_per_second = 0
        self._wheel_4_speed_distance_per_second = 0

    def set_velocities(self, motor_1_rpm: float = None, motor_2_rpm: float = None, motor_3_rpm: float = None,
                       motor_4_rpm: float = None):
        """
        Drivetrain motors:
        //---------------\\
        ||3             4||
        ||               ||
        || Forward  >    ||
        ||               ||
        ||2             1||
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
            self._wheel_4_speed_distance_per_second = motor_3_rpm / 60 * self._wheel_circumference_cm

    def update_states(self, current_time):
        delta_time = current_time - self._previousTime
        self._previousTime = current_time
        self._current_rotation_rad += (-(self._wheel_1_speed_distance_per_second + self._wheel_2_speed_distance_per_second + self._wheel_3_speed_distance_per_second + self._wheel_4_speed_distance_per_second) * delta_time / self._track_width)
        forward_backward_speed = (self._wheel_3_speed_distance_per_second - self._wheel_1_speed_distance_per_second) / 2
        left_right_speed = (self._wheel_2_speed_distance_per_second - self._wheel_4_speed_distance_per_second) / 2
        self._x_position += ((forward_backward_speed * math.cos(self._current_rotation_rad) + left_right_speed * math.sin(self._current_rotation_rad)) * delta_time)
        self._y_position += ((left_right_speed * math.cos(self._current_rotation_rad + math.pi / 4) + forward_backward_speed * math.cos(self._current_rotation_rad + math.pi / 2)) * delta_time)

    @property
    def x(self):
        return self._x_position
    
    @x.setter
    def x(self, x_position):
        self._x_position = x_position
    
    @property
    def y(self):
        return self._y_position
    
    @y.setter
    def y(self, y_position):
        self._y_position = y_position

    @property
    def rotation_deg(self):
        return math.degrees(self._current_rotation_rad)

    @rotation_deg.setter
    def rotation_deg(self, rotation_degrees):
        self._current_rotation_rad = math.radians(rotation_degrees)
    
    @property
    def rotation_rad(self):
        return self._current_rotation_rad
    
    @rotation_rad.setter
    def rotation_rad(self, rotation_radians):
        self._current_rotation_rad = rotation_radians
    
    @property
    def position(self):
        return self._x_position, self._y_position
    
    @position.setter
    def position(self, coordinates):
        self._x_position, self._y_position = coordinates
        
    @property
    def auto_update(self):
        return self._auto_update

    @auto_update.setter
    def auto_update(self, value):
        self._auto_update = value

    def auto_update_velocities(self):
        while True:
            if self.auto_update:
                self.update_states(brain.timer.time(SECONDS))
                self.set_velocities(self._motor_1.velocity(), self._motor_2.velocity(), self._motor_3.velocity(), self._motor_4.velocity())
            else:
                self.set_velocities(0, 0, 0, 0)
                self._previousTime = brain.timer.time(SECONDS)
            wait(5)
