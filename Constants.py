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


Constants:
- autonomous_verbosity: An integer representing the verbosity level of autonomous mode logging and printing (0-2).
- background_image_path: The path to a bitmap image (.BMP file with dimensions 480 x 240),
on the SD card to be displayed as the background, Set to None to disable.
- headless: A boolean indicating whether the drivetrain should be operated in headless mode.
(See note below)
- track_width_cm: The parallel distance between the centers of the wheels on the robot's drivetrain

Enums:
- Color: An enum for defining color constants.
    Constants:
    - red
    - green
    - blue

- ControllerAxis: An enum for defining Controller axis constants.
    Constants:
    - x_axis
    - y_axis
"""

from enum import Enum

autonomous_verbosity = 2  # 0-2, 0: nothing, 1: logging, 2: logging & printing
background_image_path = None  # Set this to the name of a bitmap image (.BMP file with dimensions 480 x 240)
# on the sd card to display it as the background or None to disable

"""
A note on headless mode:
    In a typical control mode, the robot's movement is relative to its current heading. For example,
    if the robot is facing forward and the control input is "move forward,"
    the robot will move in the direction it is facing. However, in headless mode,
    the control inputs are based on an absolute reference frame, regardless of the robot's orientation.
    
    For instance, if the robot is in headless mode and the control input is "move forward,"
    the robot will move in the same direction regardless of its current heading.
    The control inputs are interpreted relative to an absolute reference frame,
    such as the global coordinate system or a predefined reference direction.
    
    Headless mode can be useful in certain scenarios where the robot's orientation or heading is not critical,
    or when controlling the robot based on external references or coordinates.
    It allows for simplified control inputs and decouples the control logic from the robot's orientation.
    
    It does take a little getting used to: Good luck :)
"""
driver_control_headless = True

track_width_cm = 25.5  # The parallel distance between the centers of the wheels on the robot's drivetrain

driver_control_turn_speed = 3.5

drivetrain_turn_Kp = 1
drivetrain_turn_Ki = 0
drivetrain_turn_Kd = 0


class ControllerAxis(Enum):
    """An enum for defining Controller axis constants."""
    x_axis = 0
    y_axis = 1


class Color(Enum):
    """An enum for defining color constants."""
    red = 0
    green = 1
    blue = 2
