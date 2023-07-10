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
- Odometry

Classes:
- Drivetrain: A drivetrain controller for an X drive base.
    Methods:
    - __init__(self, brain, inertial: Inertial, motor_1: Motor, motor_2: Motor, motor_3: Motor, motor_4: Motor, movement_allowed_error_cm: float, wheel_radius_cm: float, track_width_cm: float, motor_lowest_speed: int = 1, driver_control_deadzone: float = 0.1, driver_control_turn_speed: float = 3, direction_correction_kp: float = 1, direction_correction_ki: float = 0, direction_correction_kd: float = 0):
    Initialize a new drivetrain with the specified properties.
    - move_to_position(self, target_position, maximum_speed: float = 0.35): Move to the specified position.
    - follow_path(self, point_list): Move the drivetrain along a path consisting of a list of points.
    - move(self, direction, speed, spin) -> None: Move the drivetrain towards a vector relative to its current rotation.
    - move_headless(self, direction, magnitude, spin): Move the drivetrain in headless mode (see the comment block in Constants.py titled "A note on headless mode" for more info)
    - move_with_controller(self, controller: Controller, headless: bool = False) -> None: Move the drivetrain using controller input.
    - reset(self) -> None: Reset the drivetrain to its initial state.
    - target_position: Property to get or set the target position of the robot.
    - target_heading_rad: Property to get or set the current target heading in radians.
    - target_heading_deg: Property to get or set the current target heading in degrees.
    - calculate_wheel_power(movement_angle_rad, movement_speed, wheel_angle_rad):
    Calculate the necessary wheel power for a wheel pointing in the specified angle
     in order to move the robot along the desired vector.

Author: derek.m.baier@gmail.com
"""

from Utilities import *
from XOdometry import Odometry
from Constants import ControllerAxis

X_AXIS = ControllerAxis.x_axis
Y_AXIS = ControllerAxis.y_axis


class Drivetrain(object):
    """
    A drivetrain controller for an X drive base
    """

    # noinspection GrazieInspection
    def __init__(self, brain, inertial: Inertial, motor_1: Motor, motor_2: Motor, motor_3: Motor, motor_4: Motor,
                 movement_allowed_error_cm: float, wheel_radius_cm: float,
                 track_width_cm: float, wheel_rotation_offset_rad: float = 0, driver_control_deadzone: float = 0.1,
                 driver_control_turn_speed: float = 3, direction_correction_kp: float = 1,
                 direction_correction_ki: float = 0, direction_correction_kd: float = 0) -> None:
        """
        Initialize a new drivetrain with the specified properties
        :param inertial: The inertial sensor to use for the drivetrain
        :type inertial: Inertial
        :param motor_1: Motor 1
        :type motor_1: Motor
        :param motor_2: Motor 2
        :type motor_2: Motor
        :param motor_3: Motor 3
        :type motor_3: Motor
        :param motor_4: Motor 4
        :type motor_4: Motor
        :param track_width_cm: The parallel distance between the wheels in centimeters
        :type track_width_cm: float
        :param wheel_rotation_offset_rad: The wheel rotational offset, illustrated in the robot_diagram.drawio and "robot_diagram.pdf files
        :type wheel_rotation_offset_rad: float
        :param movement_allowed_error_cm: The distance from the target that is acceptable or close enough
        :type movement_allowed_error_cm: float
        :param wheel_radius_cm: The radius of the wheels
        :type wheel_radius_cm: float
        :param driver_control_deadzone: The minimum value from the controller that should be treated as alive (Nonzero), 0.0-1.0, with 0.0 being no deadzone
        :type driver_control_deadzone: float
        """
        self.timer = brain.timer
        self.terminal = Terminal(brain)
        self.print = self.terminal.print
        self.clear = self.terminal.clear

        self._inertial = inertial
        self._motor_1 = motor_1
        self._motor_2 = motor_2
        self._motor_3 = motor_3
        self._motor_4 = motor_4
        self._movement_allowed_error = movement_allowed_error_cm
        self._wheel_radius_mm = wheel_radius_cm
        self._wheel_diameter_cm = self._wheel_radius_mm * 2
        self._wheel_circumference_cm = self._wheel_diameter_cm * math.pi
        self._track_width = track_width_cm
        self._driver_control_deadzone = driver_control_deadzone
        self.driver_control_turn_speed = driver_control_turn_speed
        self.rotation_PID = PIDController(brain.timer, direction_correction_kp, direction_correction_ki,
                                          direction_correction_kd)
        self._current_target_heading = 0
        self._current_target_x_cm = 0
        self._current_target_y_cm = 0
        self.last_move_with_controller_execution_time = None
        self.current_move_with_controller_execution_time = None
        self._motor_1.set_velocity(0, PERCENT)
        self._motor_2.set_velocity(0, PERCENT)
        self._motor_3.set_velocity(0, PERCENT)
        self._motor_4.set_velocity(0, PERCENT)
        self._motor_1.spin(FORWARD)
        self._motor_2.spin(FORWARD)
        self._motor_3.spin(FORWARD)
        self._motor_4.spin(FORWARD)

        self._odometry = Odometry(brain, self._motor_1, self._motor_2, self._motor_3, self._motor_4,
                                  self._track_width, self._wheel_circumference_cm, gyroscope=self._inertial, wheel_rotation_offset_rad=(-math.pi/4))
        self._odometry_thread = Thread(self._odometry.auto_update_velocities)

    def move_to_position(self, target_position, maximum_speed: float = 0.35) -> None:
        """
        Move to the specified position
        :param target_position: The position to mave to
        :type target_position: tuple[float, float]bot to move while trying to reach the target_position
        :param maximum_speed: The maximum speed between 0 and 1 for the ro
        """
        self._current_target_x_cm, self._current_target_y_cm = target_position
        start_direction_radians = self._odometry.rotation_rad
        self.rotation_PID._target_value = start_direction_radians
        while not check_position_within_distance(self._odometry.position, target_position,
                                                 self._movement_allowed_error):
            direction_rad = math.atan2(self._current_target_y_cm - self._odometry.y,
                                       self._current_target_x_cm - self._odometry.x)
            distance_cm = math.sqrt(((self._current_target_x_cm - self._odometry.x) ** 2 + (
                        self._current_target_y_cm - self._odometry.y) ** 2))
            spin = -self.rotation_PID.update(self._odometry.rotation_rad)
            self.move_headless(direction_rad, min(distance_cm / 10, maximum_speed), spin)

    def follow_path(self, point_list):
        for point in point_list:
            self.move_to_position(point)

    def move(self, direction, speed, spin) -> None:
        """
        Move the drivetrain towards a vector
        :param: direction
        """
        self._motor_1.set_velocity(
            (self.calculate_wheel_power(direction, clamp(speed, 0, 1), math.radians(-45)) + spin) * 100)
        self._motor_2.set_velocity(
            (self.calculate_wheel_power(direction, clamp(speed, 0, 1), math.radians(45)) + spin) * 100)
        self._motor_3.set_velocity(
            (self.calculate_wheel_power(direction, clamp(speed, 0, 1), math.radians(135)) + spin) * 100)
        self._motor_4.set_velocity(
            (self.calculate_wheel_power(direction, clamp(speed, 0, 1), math.radians(225)) + spin) * 100)

    def move_headless(self, direction, magnitude, spin):
        direction -= self._odometry.rotation_rad
        self.move(direction, magnitude, spin)

    def move_with_controller(self, controller: Controller, headless: bool = False) -> None:
        """
        Move using the controller input
        :param controller: The controller to pull input from
        :type controller: Controller
        :param headless: Whether to move the robt in headless mode
        :type headless: bool
        """
        self.current_move_with_controller_execution_time = self.timer.time(SECONDS)
        if self.last_move_with_controller_execution_time is not None:
            delta_time = self.current_move_with_controller_execution_time - self.last_move_with_controller_execution_time
        else:
            delta_time = 0
        left_stick = {X_AXIS: controller.axis4.position, Y_AXIS: controller.axis3.position}
        right_stick = {X_AXIS: controller.axis1.position, Y_AXIS: controller.axis2.position}

        left_x = left_stick[X_AXIS]() / 100
        left_y = left_stick[Y_AXIS]() / 100
        right_x = right_stick[X_AXIS]() / 100

        direction = math.atan2(left_y, left_x)

        self.target_heading_rad -= right_x * delta_time * self.driver_control_turn_speed

        magnitude = math.sqrt(left_x ** 2 + left_y ** 2)

        magnitude = apply_deadzone(magnitude, self._driver_control_deadzone, 1)

        self.rotation_PID._target_value = self.target_heading_rad
        spin = -self.rotation_PID.update(self._odometry.rotation_rad)

        if headless:
            self.move_headless(direction, magnitude, spin)
        else:
            self.move(direction, magnitude, spin)

        self.last_move_with_controller_execution_time = self.current_move_with_controller_execution_time

    def reset(self) -> None:
        """
        Reset all the drivetrain to its newly-instantiated state
        """
        self._motor_1.set_velocity(0, PERCENT)
        self._motor_2.set_velocity(0, PERCENT)
        self._motor_3.set_velocity(0, PERCENT)
        self._motor_4.set_velocity(0, PERCENT)
        self._motor_1.spin(FORWARD)
        self._motor_2.spin(FORWARD)
        self._motor_3.spin(FORWARD)
        self._motor_4.spin(FORWARD)
        self._inertial.set_heading(0, DEGREES)
        self._current_target_heading = 0
        self._current_target_x_cm = 0
        self._current_target_y_cm = 0
        self._odometry.reset()

    @property
    def target_position(self) -> tuple:
        """
        Get the position of the robot, useful for displaying on the screen
        :returns: A tuple; x, y
        :rtype: tuple[float, float]
        """
        return self._current_target_x_cm, self._current_target_y_cm

    @target_position.setter
    def target_position(self, position) -> None:
        """
        Set the target position of the robot, useful after calibration to set it to a specific position without modifying heading
        :param position: The robot's new coordinate pair (x, y)
        :type position: tuple[float, float]
        """
        self._current_target_x_cm, self._current_target_y_cm = position

    @property
    def target_heading_rad(self) -> float:
        """
        Get the current target heading in radians
        :returns: The current target heading of the robot in radians (use heading_deg for degrees)
        """
        return self._current_target_heading

    @target_heading_rad.setter
    def target_heading_rad(self, heading) -> None:
        """
        Set the current target heading in radians
        :param heading: New target heading in radians
        """
        self._current_target_heading = heading

    @property
    def target_heading_deg(self) -> float:
        """
        Get the current target heading in degrees
        :returns: The current target heading of the robot in degrees (use heading_deg for radians)
        """
        return math.degrees(self._current_target_heading)

    @target_heading_deg.setter
    def target_heading_deg(self, heading) -> None:
        """
        Set the current target heading in degrees
        :param heading: New target heading in degrees
        """
        self._current_target_heading = math.radians(heading)

    @staticmethod
    def calculate_wheel_power(movement_angle_rad, movement_speed, wheel_angle_rad) -> float:
        """
        Calculate the necessary wheel power for a wheel pointing in the specified angle to move the robot toward the desired target
        This function must be run for all wheels in the drivetrain separately
        :param movement_angle_rad: The angle to move the robot
        :type movement_angle_rad: float
        :param movement_speed: The speed to move at
        :type movement_speed: float
        :param wheel_angle_rad: The angle of the wheel to calculate power for
        :type wheel_angle_rad: float
        """
        return movement_speed * math.sin(wheel_angle_rad + movement_angle_rad) * 2
