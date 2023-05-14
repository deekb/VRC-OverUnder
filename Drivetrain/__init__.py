"""
A highly customizable drivetrain with built-in dynamic course correction
"""
import math

from vex import *
from HelperFunctions import apply_deadzone, print, clamp
from Odometry import XDriveDrivetrainOdometry


class Drivetrain(object):
    """
    A drivetrain controller for an X drive base
    """
    def __init__(self, inertial: Inertial, motor_1: Motor, motor_2: Motor, motor_3: Motor, motor_4: Motor,
                 heading_allowed_error: float, wheel_radius_mm: float, track_width_cm: float, turn_Kp: float = 0.25,
                 correction_Kp: float = 0.1, motor_lowest_speed: int = 1,
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
        :param heading_allowed_error: The delta heading that is acceptable or close enough
        :type heading_allowed_error: float
        :param turn_Kp: How aggressive to be while turning
        :type turn_Kp: float
        :param correction_Kp: How aggressive to be while correcting movements
        :type correction_Kp: float
        :param wheel_radius_mm: The radius of the wheels
        :type wheel_radius_mm: float
        :param motor_lowest_speed: The speed at which the motors can just barely spin (normally "1" for accuracy but can be set higher if your drivetrain has more friction)
        :type motor_lowest_speed: int
        :param driver_control_linearity: How close to linearly to map the controllers inputs to the motors outputs during the cubic normalization
        :type driver_control_linearity: float
        :param driver_control_deadzone: The minimum value from the controller that should be treated as alive (Nonzero), 0.0-1.0, with 0.0 being no deadzone
        :type driver_control_deadzone: float
        """
        self._inertial = inertial
        self._motor_1 = motor_1
        self._motor_2 = motor_2
        self._motor_3 = motor_3
        self._motor_4 = motor_4
        self.all_motors = MotorGroup(motor_1, motor_2, motor_3, motor_4)
        self._heading_allowed_error = heading_allowed_error
        self._turn_aggression = turn_Kp
        self._correction_aggression = correction_Kp
        self._wheel_radius_mm = wheel_radius_mm
        self._wheel_diameter_mm = self._wheel_radius_mm * 2
        self._wheel_circumference_mm = self._wheel_diameter_mm * math.pi
        self._track_width = track_width_cm
        self._motor_lowest_speed = motor_lowest_speed
        self._driver_control_linearity = driver_control_linearity
        self._driver_control_deadzone = driver_control_deadzone
        self._current_target_heading = 0
        self._current_target_x_cm = 0
        self._current_target_y_cm = 0
        self.all_motors.spin(FORWARD)
        self.all_motors.set_velocity(0, PERCENT)
        self.odometry = XDriveDrivetrainOdometry(self._track_width, self._wheel_diameter_mm)
        self._odometry_thread = Thread(self.odometry.auto_update_velocities, args=(motor_1, motor_2, motor_3, motor_4))

    def turn_to_heading(self, desired_heading: float) -> None:
        """
        Turn to an absolute heading using the inertial sensor
        :param desired_heading: The heading to turn to
        :type desired_heading: float
        """
        desired_heading %= 360
        get_heading = self._inertial.heading  # Speeds up the process of getting the heading by storing a reference to the getter function
        current_heading = get_heading(DEGREES) % 360  # Running modulo on the heading to ensure it is between 0 and 360
        # Determine how far off the robot is and which way is a shorter turn
        left_turn_difference = (current_heading - desired_heading)
        right_turn_difference = (desired_heading - current_heading)
        if left_turn_difference < 0:
            left_turn_difference += 360
        if right_turn_difference < 0:
            right_turn_difference += 360
        if left_turn_difference < right_turn_difference:
            delta_heading = left_turn_difference
        else:
            delta_heading = right_turn_difference
        self.all_motors.set_velocity(0, PERCENT)
        self.all_motors.spin(FORWARD)
        while abs(delta_heading) > self._heading_allowed_error:
            if left_turn_difference < right_turn_difference:
                delta_heading = left_turn_difference
                self._motor_1.set_velocity(delta_heading * self._turn_aggression + self._motor_lowest_speed, PERCENT)
                self._motor_2.set_velocity(delta_heading * self._turn_aggression + self._motor_lowest_speed, PERCENT)
                self._motor_3.set_velocity(delta_heading * self._turn_aggression + self._motor_lowest_speed, PERCENT)
                self._motor_4.set_velocity(delta_heading * self._turn_aggression + self._motor_lowest_speed, PERCENT)
            else:
                delta_heading = right_turn_difference
                self._motor_1.set_velocity((delta_heading * self._turn_aggression + self._motor_lowest_speed) * -1, PERCENT)
                self._motor_2.set_velocity((delta_heading * self._turn_aggression + self._motor_lowest_speed) * -1, PERCENT)
                self._motor_3.set_velocity((delta_heading * self._turn_aggression + self._motor_lowest_speed) * -1, PERCENT)
                self._motor_4.set_velocity((delta_heading * self._turn_aggression + self._motor_lowest_speed) * -1, PERCENT)
            current_heading = get_heading(DEGREES) % 360
            left_turn_difference = current_heading - desired_heading
            right_turn_difference = desired_heading - current_heading
            if left_turn_difference < 0:
                left_turn_difference += 360
            if right_turn_difference < 0:
                right_turn_difference += 360
        self.all_motors.stop()
        self._current_target_heading = desired_heading

    # def move_to_position(self, x: float, y: float, target_speed: float) -> None:
    #     """
    #     Move to an x, y position
    #     :param x: The x position to mave to
    #     :param y: The y position to mave to
    #     :param target_speed: The speed to move at
    #     """
    #     angle = math.atan2(x - self._current_target_x_cm, y - self._current_target_y_cm) * math.pi / 180
    #     distance = math.sqrt(((x - self._current_target_x_cm) ** 2 + (y - self._current_target_y_cm) ** 2))

    def move_with_controller(self, controller: Controller, headless: bool = False) -> None:
        """
        Move using the controller input
        :param controller: The controller to get input from
        :type controller: Controller
        :param headless: Whether to move the robt in headless mode
        :type headless: bool
        """
        left_stick, right_stick = {"x": controller.axis4.position, "y": controller.axis3.position},\
                                  {"x": controller.axis1.position, "y": controller.axis2.position}
        left_x = apply_deadzone(left_stick["x"]() / 100, self._driver_control_deadzone, 1)
        left_y = apply_deadzone(left_stick["y"]() / 100, self._driver_control_deadzone, 1)
        right_x = apply_deadzone(right_stick["x"]() / 100, self._driver_control_deadzone, 1)

        if left_x or left_y or right_x:
            angle = math.atan2(left_x, left_y)
            distance = math.sqrt(left_x ** 2 + left_y ** 2)
            print("Left Stick angle: " + str(math.degrees(angle)))
            if headless:
                angle -= self._odometry.get_rotation_rad()
                angle %= 360
                print("Angle after headless offset: " + str(math.degrees(angle)))
            self._motor_1.set_velocity((self.calculate_wheel_power(angle, clamp(distance, 0, 1), 45) + right_x) * 100)
            self._motor_2.set_velocity((self.calculate_wheel_power(angle, clamp(distance, 0, 1), 135) + right_x) * 100)
            self._motor_3.set_velocity((self.calculate_wheel_power(angle, clamp(distance, 0, 1), 225) + right_x) * 100)
            self._motor_4.set_velocity((self.calculate_wheel_power(angle, clamp(distance, 0, 1), 315) + right_x) * 100)

    def reset(self) -> None:
        """
        Reset all rolling aspects of the drivetrain
        """
        self.all_motors.stop()
        self.all_motors.set_velocity(0, PERCENT)
        self._inertial.set_heading(0, DEGREES)
        self._current_target_heading = 0
        self._current_target_x_cm = 0
        self._current_target_y_cm = 0

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
    def calculate_wheel_power(movement_angle_deg, movement_speed, wheel_angle_deg):
        return movement_speed * math.sin(math.radians(wheel_angle_deg - movement_angle_deg))
