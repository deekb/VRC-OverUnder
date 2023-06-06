"""
A highly customizable drivetrain with built-in dynamic course correction
"""
import math

from vex import *
from Utilities import apply_deadzone, print, clear, clamp, check_position_within_tolerance
from Odometry import XDriveDrivetrainOdometry

brain = Brain()


class Drivetrain(object):
    """
    A drivetrain controller for an X drive base
    """

    # noinspection GrazieInspection
    def __init__(self, inertial: Inertial, motor_1: Motor, motor_2: Motor, motor_3: Motor, motor_4: Motor,
                 movement_allowed_error_cm: float, wheel_radius_cm: float,
                 track_width_cm: float, motor_lowest_speed: int = 1,
                 driver_control_linearity: float = 0.45, driver_control_deadzone: float = 0.1) -> None:
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
        :param movement_allowed_error_cm: The distance from the target that is acceptable or close enough
        :type movement_allowed_error_cm: float
        :param wheel_radius_cm: The radius of the wheels
        :type wheel_radius_cm: float
        :param motor_lowest_speed: The speed at which the motors can just barely spin (normally "1" for accuracy but can be set higher if your drivetrain has more friction)
        :type motor_lowest_speed: int
        :param driver_control_linearity: How close to linearly to map the controllers inputs to the motors outputs during the cubic normalization
        :type driver_control_linearity: float
        :param driver_control_deadzone: The minimum value from the controller that should be treated as alive (Nonzero), 0.0-1.0, with 0.0 being no deadzone
        :type driver_control_deadzone: float
        :param DEBUG: Whether to print debugging information (slows down the program)
        :type DEBUG: bool
        """
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
        self._motor_lowest_speed = motor_lowest_speed
        self._driver_control_linearity = driver_control_linearity
        self._driver_control_deadzone = driver_control_deadzone
        self._current_target_heading = 0
        self._current_target_x_cm = 0
        self._current_target_y_cm = 0
        self._motor_1.set_velocity(0, PERCENT)
        self._motor_2.set_velocity(0, PERCENT)
        self._motor_3.set_velocity(0, PERCENT)
        self._motor_4.set_velocity(0, PERCENT)
        self._motor_1.spin(FORWARD)
        self._motor_2.spin(FORWARD)
        self._motor_3.spin(FORWARD)
        self._motor_4.spin(FORWARD)

        self._odometry = XDriveDrivetrainOdometry(self._motor_1, self._motor_2, self._motor_3, self._motor_4,
                                                  self._track_width, self._wheel_circumference_cm)
        self._odometry_thread = Thread(self._odometry.auto_update_velocities)

    def move_to_position(self, target_position) -> None:
        """
        Move to the specified position
        :param target_position: The position to mave to
        :type target_position: tuple[float, float]
        """
        self._current_target_x_cm, self._current_target_y_cm = target_position
        while not check_position_within_tolerance(self._odometry.position, target_position, self._movement_allowed_error):
            direction_rad = math.atan2(self._current_target_y_cm - self._odometry.y, self._current_target_x_cm - self._odometry.x)
            distance_cm = math.sqrt(((self._current_target_x_cm - self._odometry.x) ** 2 + (self._current_target_y_cm - self._odometry.y) ** 2))
            self.move_headless(direction_rad, min(distance_cm / 100, 1), 0)

    def follow_path(self, point_list):
        for point in point_list:
            self.move_to_position(point)

    def move(self, direction, speed, spin) -> None:
        """
        Move the drivetrain towards a vector
        :param: direction
        """
        self._motor_1.set_velocity((self.calculate_wheel_power(direction, clamp(speed, 0, 1), math.radians(-45)) + spin) * 100)
        self._motor_2.set_velocity((self.calculate_wheel_power(direction, clamp(speed, 0, 1), math.radians(45)) + spin) * 100)
        self._motor_3.set_velocity((self.calculate_wheel_power(direction, clamp(speed, 0, 1), math.radians(135)) + spin) * 100)
        self._motor_4.set_velocity((self.calculate_wheel_power(direction, clamp(speed, 0, 1), math.radians(225)) + spin) * 100)

    def move_headless(self, direction, magnitude, spin):
        direction -= self._odometry.rotation_rad
        self.move(direction, magnitude, spin)

    def move_with_controller(self, controller: Controller, headless: bool = False) -> None:
        """
        Move using the controller input
        :param controller: The controller to get input from
        :type controller: Controller
        :param headless: Whether to move the robt in headless mode
        :type headless: bool
        """
        left_stick = {"x": controller.axis4.position, "y": controller.axis3.position}
        right_stick = {"x": controller.axis1.position, "y": controller.axis2.position}

        left_x = left_stick["x"]() / 100
        left_y = left_stick["y"]() / 100
        right_x = right_stick["x"]() / 100

        direction = math.atan2(left_y, left_x)
        magnitude = math.sqrt(left_x ** 2 + left_y ** 2)

        magnitude = apply_deadzone(magnitude, self._driver_control_deadzone, 1)

        if headless:
            self.move_headless(direction, magnitude, right_x)
        else:
            self.move(direction, magnitude, right_x)

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
    def calculate_wheel_power(movement_angle_deg, movement_speed, wheel_angle_deg) -> float:
        """
        Calculate the necessary wheel power for a wheel pointing in the specified angle to move the robot toward the desired target
        This function must be run for all wheels in the drivetrain separately
        :param movement_angle_deg: The angle to move the robot
        :type movement_angle_deg: float
        :param movement_speed: The speed to move at
        :type movement_speed: float
        :param wheel_angle_deg: The angle of the wheel to calculate power for
        :type wheel_angle_deg: float
        """
        return movement_speed * math.sin(wheel_angle_deg + movement_angle_deg)
