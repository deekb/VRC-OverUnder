"""
Competition Code for VRC: Over-Under (2024-2025)
Team: 3773P (Bowbots Phosphorus) from Bow NH
Author: Derek Baier (deekb on GitHub)
Project homepage: https://github.com/deekb/VRC-OverUnder
Project archive: https://github.com/deekb/VRC-OverUnder/archive/master.zip
Contact Derek.m.baier@gmail.com for more information

This module contains the competition code for the VRC (VEX Robotics Competition) Over-Under game,
for the 2024-2025 season.

The code is specifically developed for Team 3773P, known as Bowbots Phosphorus, and is authored by Derek Baier.

The project homepage and archive can be found on GitHub at the provided links.

For more information about the project, you can contact Derek Baier at the given email address.

The module sets up the necessary components
and defines classes and functions for controlling the VEX robot during competition.
It includes the required modules and handles the initialization of motors,
sensors, controllers, and other global variables.
The module provides functions for autonomous and driver control modes,
as well as utility functions for debugging and setup.

Module dependencies:
- Constants
- XDrivetrain
- XOdometry
- Utilities
- Autonomous

Functions:
- autonomous_handler(): Coordinates when to run the autonomous function(s).
- select_autonomous(): Selects which autonomous routine to execute using the controller.
- driver_handler(): Coordinates when to run the driver function(s).
- on_driver(): This is the function designated to run when the driver control portion of the program is triggered.
- on_autonomous(): This is the function designated to run when the autonomous portion of the program is triggered.

Debug Related Functions:
- debug_thread(): A thread function for debugging purposes, displays the position and direction of the drivetrain.
- rotate_to_0_degrees(): Sets the target heading of the drivetrain to 0 degrees.
- rotate_to_90_degrees(): Sets the target heading of the drivetrain to 90 degrees.
- rotate_to_180_degrees(): Sets the target heading of the drivetrain to 180 degrees.
- rotate_to_270_degrees(): Sets the target heading of the drivetrain to 270 degrees.
"""

__title__ = "Vex V5 2024 Competition code"
__description__ = "Competition Code for VRC: Over-Under 2024-2025"
__team__ = "3773P (Bowbots Phosphorus)"
__url__ = "https://github.com/deekb/VRC-OverUnder"
__download_url__ = "https://github.com/deekb/VRC-OverUnder/archive/master.zip"
__version__ = "0.0.2_rc"
__author__ = "Derek Baier"
__author_email__ = "Derek.m.baier@gmail.com"
__license__ = "MIT"

from vex import *

brain = Brain()

"""
The purpose of the following loop is to ensure that all the required modules are successfully imported before proceeding.
If any of the modules fail to import, an error message is displayed on screen,
indicating that the library Micro-SD card needs to be inserted.
The loop continues to check for the presence of the required modules until they are successfully imported,
at which point the loop breaks, and the program execution proceeds.
"""

while True:
    try:
        import Constants  # noqa: suppress pycharm module not found message as the program will continue to try to import them until they are loaded
        from Utilities import Terminal, apply_deadzone, check_position_within_distance, MotorPID, clamp, PIDController, Logging  # noqa: reason above ^^
        from XDrivetrain import Drivetrain  # noqa: reason above ^^
        from Autonomous import Autonomous, available_autonomous_routines  # noqa: reason above ^^

        brain.screen.clear_screen()
        print("Successfully loaded 4 modules")
        wait(500)
        brain.screen.clear_screen()
        break
    except ImportError:
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("Please insert the library Micro-SD card")
        wait(250)

brain.screen.clear_screen()
brain.screen.set_cursor(1, 1)

terminal = Terminal(brain)
print = terminal.print
clear = terminal.clear

if Constants.background_image_path:
    brain.screen.draw_image_from_file(str(Constants.background_image_path), 0, 0)


class Motors:
    """
     A class representing the motors and motor groups attached to the robot.

    This class provides references to individual motors and motor groups, allowing control over the robot's movement
    and the ability to stop all motors simultaneously.

    Motor positions:
        //---------------\\
        ||1             2||
        ||               ||
        || Forward  >    ||
        ||               ||
        ||4             3||
        \\---------------//

    Available Motors:
    - motor_1: The top left motor.
    - motor_2: The top right motor.
    - motor_3: The bottom right motor.
    - motor_4: The bottom Left motor.
    # All motor point with their forward clockwise,
    # If you turned all the motors forward, the drivetrain would spin clockwise

    Motor Groups:
    - allWheels: All the motors controlling wheels on the robot.
    - allMotors: All the motors on the robot.
    """

    motor_1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
    motor_2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
    motor_3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
    motor_4 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
    # Motor groups:
    allWheels = MotorGroup(motor_1, motor_2, motor_3,
                           motor_4)  # A reference to all wheels so that we can stop the drivetrain the robot in one command
    allMotors = MotorGroup(motor_1, motor_2, motor_3,
                           motor_4)  # A reference to all motors so that we can stop everything on the robot in one command


class Sensors:
    """
    A class representing the sensors attached to the robot.

    This class provides references to various sensors, enabling data collection and integration with the robot's
    decision-making processes.

    Available Sensors:
    - inertial: 6-axis IMU: 3-axis accelerometer and 3-axis gyroscope
    """

    inertial = Inertial(Ports.PORT5)


class Controllers:
    """
    A class that contains references to the primary and secondary controller
    """
    primary = Controller(PRIMARY)
    secondary = Controller(PARTNER)


class Globals:
    """
    Stores variables that can be modified or accessed by any function in the program.
    Here you can also set default/initial values for said variables
    """
    autonomous_threads = []  # This list will be populated with any an all autonomous tasks
    # these tasks are running concurrently during autonomous mode so that they may be monitored, tracked,
    # and stopped at the appropriate time
    driver_control_threads = []  # Same thing but for driver control
    setup_complete = False  # Whether the pre-match selections on the controller have been completed
    pause_driver_control = False  # This can be used for autonomous functions during drier control, for example,
    # if we want to turn towards a goal when a button is pressed,
    # you can set this lock variable to True and keep everything nice and thread-safe
    autonomous_task = None  # The task to perform during autonomous
    # (DO NOT SET HERE: this should be set by the setup function)


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # ensure setup is complete
    if not Globals.setup_complete:
        print("[on_autonomous]: setup not complete, can't start autonomous")
        return
    Autonomous(terminal=terminal, motors=Motors, drivetrain=drivetrain, _globals=Globals,
               verbosity=Constants.autonomous_verbosity)


def debug_thread():
    while True:
        if "drivetrain" in globals():
            # For testing, ignore the protected member error!
            clear()
            print("Position: " + str(drivetrain._odometry.x) + " " + str(drivetrain._odometry.y))  # noqa
            print("Direction: " + str(drivetrain._odometry.rotation_deg))  # noqa
            wait(100)
            if Controllers.primary.buttonA.pressing():
                drivetrain.reset()
            if Controllers.primary.buttonB.pressing():
                Globals.pause_driver_control = True
                drivetrain.follow_path([(0, 0), (0, 121.92), (121.92, 121.92), (121.92, 0), (0, 0)])
                # drivetrain.move_to_position((0, 0))
                Globals.pause_driver_control = False


# region Debug_Functions


def rotate_to_0_degrees():
    drivetrain.target_heading_deg = 0


def rotate_to_90_degrees():
    drivetrain.target_heading_deg = 90


def rotate_to_180_degrees():
    drivetrain.target_heading_deg = 180


def rotate_to_270_degrees():
    drivetrain.target_heading_deg = 270


# endregion


def on_driver() -> None:
    """
    This is the function designated to run when the driver control portion of the program is triggered
    """
    # Wait for setup to finish
    print("[on_driver]: Waiting for setup...")
    while not Globals.setup_complete:
        wait(5)
    print("[on_driver]: Done")
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.pause_driver_control:
            drivetrain.move_with_controller(Controllers.primary, headless=Constants.driver_control_headless)
        else:
            wait(10)


def autonomous_handler() -> None:
    """
    Coordinate when to run the autonomous function(s)
    """
    for _function in (on_autonomous,):
        Globals.autonomous_threads.append(Thread(_function))

    print("Started autonomous")
    while competition.is_autonomous() and competition.is_enabled():
        wait(10, MSEC)
    for _thread in Globals.autonomous_threads:
        _thread.stop()


def driver_handler() -> None:
    """
    Coordinate when to run the driver function(s)
    """
    for _function in (on_driver, debug_thread):
        Globals.driver_control_threads.append(Thread(_function))
    print("Started driver control")
    while competition.is_driver_control() and competition.is_enabled():
        wait(10, MSEC)
    for _thread in Globals.driver_control_threads:
        _thread.stop()


# Register the competition handlers
competition = Competition(driver_handler, autonomous_handler)


def select_autonomous() -> None:
    """
    Selects which autonomous to execute using the controller
    """
    possible_tasks = available_autonomous_routines
    autonomous_index = 0
    Globals.autonomous_task = possible_tasks[autonomous_index]
    while not Controllers.primary.buttonA.pressing():
        # Wait until all buttons are released
        while any((Controllers.primary.buttonLeft.pressing(),
                   Controllers.primary.buttonRight.pressing(),
                   Controllers.primary.buttonA.pressing(),
                   Controllers.primary.buttonB.pressing())):
            wait(5)
        while not any((Controllers.primary.buttonLeft.pressing(),
                       Controllers.primary.buttonRight.pressing(),
                       Controllers.primary.buttonA.pressing(),
                       Controllers.primary.buttonB.pressing())):
            wait(5)
        Globals.autonomous_task = possible_tasks[autonomous_index]

        Controllers.primary.screen.clear_screen()
        Controllers.primary.screen.set_cursor(1, 1)
        Controllers.primary.screen.print(("Autonomous: " + Globals.autonomous_task[0]))
        if Controllers.primary.buttonRight.pressing() and autonomous_index < len(possible_tasks):
            autonomous_index += 1
        elif Controllers.primary.buttonLeft.pressing() and autonomous_index > 0:
            autonomous_index -= 1


if __name__ == "__main__":
    print("Program: " + __title__)
    print("Version: " + __version__)
    print("Author: " + __author__)
    print("Team: " + __team__)
    select_autonomous()
    print("Autonomous selected")
    Motors.allWheels.set_stopping(BRAKE)
    Controllers.primary.rumble("-")
    drivetrain = Drivetrain(brain=brain, inertial=Sensors.inertial, motor_1=Motors.motor_1, motor_2=Motors.motor_2,
                            motor_3=Motors.motor_3, motor_4=Motors.motor_4, movement_allowed_error_cm=2,
                            wheel_radius_cm=4.13, track_width_cm=Constants.track_width_cm,
                            driver_control_turn_speed=Constants.driver_control_turn_speed,
                            direction_correction_kp=Constants.drivetrain_turn_Kp,
                            direction_correction_ki=Constants.drivetrain_turn_Ki,
                            direction_correction_kd=Constants.drivetrain_turn_Kd)
    print("Calibrating Gyro...")
    Sensors.inertial.calibrate()
    while Sensors.inertial.is_calibrating():
        pass
    # Set up controller callbacks here to avoid triggering them by pressing buttons during setup
    # Primary controller bindings
    # Controllers.primary.buttonA.pressed(callback)
    # Secondary controller bindings
    # Controllers.secondary.buttonA.pressed(callback)
    Controllers.primary.buttonL1.pressed(rotate_to_0_degrees)
    Controllers.primary.buttonL2.pressed(rotate_to_90_degrees)
    Controllers.primary.buttonR1.pressed(rotate_to_180_degrees)
    Controllers.primary.buttonR1.pressed(rotate_to_270_degrees)
    Globals.setup_complete = True
    print("Setup complete", end="\n")
    Controllers.primary.rumble(".")
