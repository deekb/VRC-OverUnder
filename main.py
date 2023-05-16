"""
Competition Code for VRC: Over-Under (2024-2025)
Team: 3773P (Bowbots Phosphorus)
Author: Derek Baier (deekb on GitHub)
Project homepage: <Project URL>
Project archive: <Project URL>/archive/master.zip
Contact Derek.m.baier@gmail.com for more information
"""
from vex import *
import Constants

# If you get errors on any of the following imports it means that you
# didn't copy their corresponding modules to the Micro-SD card in the Micro-SD slot on your VEX brain
# if you do this, and it still doesn't want to upload click "Upload anyway"
from Constants import *
from HelperFunctions import *
from Drivetrain import Drivetrain
from Autonomous import Autonomous

__title__ = "Vex V5 2024 Competition code"
__description__ = "Competition Code for VRC: Over-Under 2024-2025"
__team__ = "3773P (Bowbots Phosphorus)"
__url__ = "<Project URL>"
__download_url__ = "<Project URL>/archive/master.zip"
__version__ = "0.0.1_rc"
__author__ = "Derek Baier"
__author_email__ = "Derek.m.baier@gmail.com"
__license__ = "MIT"

brain = Brain()
if Constants.background_image_path:
    brain.screen.draw_image_from_file(str(Constants.background_image_path), 0, 0)


class Motors:
    """
    A class containing references to all motors and motor groups attached to the robot including motors with custom PIDs
    """
    # Drivetrain motors:
    # //---------------\\
    # ||4             1||
    # ||       ^       ||
    # ||    Forward    ||
    # ||               ||
    # ||3             2||
    # \\---------------//
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
    A class that contains references to all sensors attached to the robot
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
    Stores variables that may need to be (or should be able to be) modified or accessed by any function in the program, here you can also set default/initial values for said variables
    """
    autonomous_threads = []  # This is a list of the threads running concurrently during autonomous so that they may be tracked and stopped at the appropriate time
    driver_control_threads = []  # Same thing but for driver control
    setup_complete = False  # Whether the pre-match selections on the controller have been completed
    pause_driver_control = False  # This can be used for autonomous functions during driver control, for example if we want to turn towards a goal when a button is pressed
    autonomous_task = None  # The task to perform during autonomous (should be set by the setup function)


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # ensure setup is complete
    if not Globals.setup_complete:
        print("[on_autonomous]: setup not complete, ignoring request")
        return
    Autonomous()


def print_current_position():
    print("Position: " + str(drivetrain.odometry.get_position()))
    print("Direction: " + str(drivetrain.odometry.get_rotation_deg()))


def on_driver() -> None:
    """
    This is the function designated to run when the driver control portion of the program is triggered
    """
    # Wait for setup to be complete
    print("[on_driver]: Waiting for setup")
    while not Globals.setup_complete:
        sleep(5)
    print("[on_driver]: Done")
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.pause_driver_control:
            drivetrain.move_with_controller(Controllers.primary)
        else:
            wait(10)


# <editor-fold desc="Competition State Handlers">
def autonomous_handler() -> None:
    """
    Coordinate when to run the autonomous function(s) using the vex competition library to read the game state.
    """
    for _function in (on_autonomous,):
        Globals.autonomous_threads.append(Thread(_function))
    Autonomous()

    print("Started autonomous")
    while competition.is_autonomous() and competition.is_enabled():
        wait(10, MSEC)
    for _thread in Globals.autonomous_threads:
        _thread.stop()


def driver_handler() -> None:
    """
    Coordinate when to run the driver function(s) using the vex competition library to read the game state.
    """
    for _function in (on_driver,):
        Globals.driver_control_threads.append(Thread(_function))
    print("Started driver control")
    while competition.is_driver_control() and competition.is_enabled():
        wait(10, MSEC)
    for _thread in Globals.driver_control_threads:
        _thread.stop()


# Register the competition handlers
competition = Competition(driver_handler, autonomous_handler)


# </editor-fold>


def select_autonomous() -> None:
    """
    A setup function, selects the autonomous to execute using the controller
    """
    # This line grabs all non-hidden variables in the Constants.AutonomousTask class so that we can select one
    possible_tasks = [(name, value) for name, value in vars(Autonomous).items() if not name.startswith("_")]
    autonomous_index = 0
    Globals.autonomous_task = possible_tasks[autonomous_index][1]
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
        autonomous_index = 0
        Globals.autonomous_task = possible_tasks[autonomous_index][1]

        clear(Controllers.primary)
        print(text=("Autonomous: " + Globals.autonomous_task), console=Controllers.primary)
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
    clear(Controllers.primary)
    drivetrain = Drivetrain(inertial=Sensors.inertial, motor_1=Motors.motor_1, motor_2=Motors.motor_2,
                            motor_3=Motors.motor_3, motor_4=Motors.motor_4, wheel_radius_mm=50, heading_allowed_error=1, track_width_cm=Constants.track_width_cm)
    print("Calibrating Gyro...")
    Sensors.inertial.calibrate()
    while Sensors.inertial.is_calibrating():
        pass
    clear(Controllers.primary)
    # Set up controller callbacks here to avoid triggering them by pressing buttons during setup
    # Primary controller bindings
    # Controllers.primary.buttonA.pressed(callback)
    # Secondary controller bindings
    # Controllers.secondary.buttonA.pressed(callback)
    Controllers.primary.buttonA.pressed(print_current_position)
    Globals.setup_complete = True
    print("Setup complete", (brain, Controllers.primary))
    Controllers.primary.rumble(".")
    Controllers.secondary.rumble(".")
