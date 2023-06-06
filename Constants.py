"""
All custom constants needed for our programs
"""
# speed_curve_linearity is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc
# it should be set between 0.00 and 3.00 for optimal performance
speed_curve_linearity = 0.35
autonomous_verbosity = 2  # 0-2, 0: nothing, 1: logging, 2: logging & printing
background_image_path = None  # Set this to the name of a bitmap (BMP) image (480 x 240) on the sd card to display it as the background or None to disable
headless = True
track_width_cm = 26


class Color(object):
    """
    Any color constants needed in the program
    """
    red = 0
    green = 1
    blue = 2
