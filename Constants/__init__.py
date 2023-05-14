"""
All custom constants needed for our programs
"""
# speed_curve_linearity is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc
# it should be set between 0.00 and 3.00 for optimal performance
speed_curve_linearity = 0.35
autonomous_verbosity = 2  # 0-2, 0: nothing, 1: logging, 2: logging & printing
background_image_path = None  # Set this to the name of a bitmap (BMP) image (480 x 240) on the sd card to display it as the background or None to disable
headless = True


class Color(object):
    """
    Any color constants needed in the program
    """
    red = 0
    green = 1
    blue = 2


class AutonomousTask(object):
    """
    All autonomous tasks, this class contains constants for checking the name of the currently selected autonomous,
    to add an entry, create a new class variable of it's name and set it's content to a human-readable version of it's name
    """
    skills = "Skills"
    do_nothing = "Nothing"
    drivetrain_test = "Drivetrain Test"
