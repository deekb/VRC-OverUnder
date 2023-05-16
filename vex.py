"""
This file is for Pycharm autocomplete only and has no functionality,
it does NOT need to be uploaded to the robot or placed on the SD card
"""

SECONDS = "SECONDS"
PERCENT = "PERCENT"
DEGREES = "DEGREES"
MSEC = "MSEC"
PRIMARY = "PRIMARY"
PARTNER = "PARTNER"
COAST = "COAST"
BRAKE = "BRAKE"
HOLD = "HOLD"
FORWARD = "FORWARD"
REVERSE = "REVERSE"


class Competition:
    @staticmethod
    def is_enabled():
        return False

    @staticmethod
    def is_autonomous():
        return False

    @staticmethod
    def is_driver_control():
        return False


class Inertial:
    @staticmethod
    def calibrate():
        pass

    @staticmethod
    def heading(*args):
        pass

    @staticmethod
    def is_calibrating():
        return False

    @staticmethod
    def set_heading(*args):
        pass


class Controller:
    def __init__(self, port):
        self.port = port
        pass

    class screen:
        @staticmethod
        def print(text):
            pass

        @staticmethod
        def set_cursor(ROW, COLUMN):
            pass

        @staticmethod
        def next_row():
            pass

        @staticmethod
        def clear_screen():
            pass

        @staticmethod
        def clear_row(ROW=-1):
            pass

        @staticmethod
        def draw_pixel(X, Y):
            pass

        @staticmethod
        def draw_line(START_X, START_Y, END_X, END_Y):
            pass

        @staticmethod
        def draw_rectangle(X, Y, WIDTH, HEIGHT):
            pass

        @staticmethod
        def draw_circle(X, Y, RADIUS):
            pass

        @staticmethod
        def set_font(FONT_TYPE):
            pass

        @staticmethod
        def set_pen_width(PEN_WIDTH):
            pass

        @staticmethod
        def set_pen_color(COLOR):
            pass

        @staticmethod
        def set_fill_color(COLOR):
            pass

        @staticmethod
        def pressed(callback):
            pass

        @staticmethod
        def released(callback):
            pass

        @staticmethod
        def row():
            return 20

        @staticmethod
        def column():
            return 80

        @staticmethod
        def pressing():
            return False

        @staticmethod
        def x_position():
            return 0

        @staticmethod
        def y_position():
            return 0

        @classmethod
        def draw_image_from_file(cls, *args):
            pass

    class buttonLeft:
        @staticmethod
        def pressing():
            return False

    class buttonRight:
        @staticmethod
        def pressing():
            return False

    class buttonA:
        @staticmethod
        def pressing():
            return False

    class buttonB:
        @staticmethod
        def pressing():
            return False

    @staticmethod
    def rumble(*args):
        pass

    class axis1:
        @staticmethod
        def position():
            return 0

    class axis2:
        @staticmethod
        def position():
            return 0

    class axis3:
        @staticmethod
        def position():
            return 0

    class axis4:
        @staticmethod
        def position():
            return 0


class Motor:
    def __init__(self, Port, GearRatio, Inverted):
        pass

    @staticmethod
    def spin(direction):
        pass

    @staticmethod
    def set_velocity(*args):
        pass

    @staticmethod
    def velocity():
        return 0


class MotorGroup:
    def __init__(self, *motors):
        pass

    def spin(self, direction):
        pass

    def set_stopping(self, *args):
        pass

    def set_velocity(self, *args):
        pass

    @staticmethod
    def position(*args):
        return 0

    def stop(self):
        pass


class Ports:
    PORT1 = "Port 1"
    PORT2 = "Port 2"
    PORT3 = "Port 3"
    PORT4 = "Port 4"
    PORT5 = "Port 5"
    PORT6 = "Port 6"
    PORT7 = "Port 7"
    PORT8 = "Port 8"
    PORT9 = "Port 9"
    PORT10 = "Port 10"
    PORT11 = "Port 11"
    PORT12 = "Port 12"
    PORT13 = "Port 13"
    PORT14 = "Port 14"
    PORT15 = "Port 15"
    PORT16 = "Port 16"
    PORT17 = "Port 17"
    PORT18 = "Port 18"
    PORT19 = "Port 19"
    PORT20 = "Port 20"
    PORT21 = "Port 21"


class GearSetting:
    RATIO_18_1 = "18 to 1"


class FontType:
    @staticmethod
    def MONO12():
        pass

    @staticmethod
    def MONO15():
        pass

    @staticmethod
    def MONO20():
        pass

    @staticmethod
    def MONO30():
        pass

    @staticmethod
    def MONO40():
        pass

    @staticmethod
    def MONO60():
        pass

    @staticmethod
    def PROP20():
        pass

    @staticmethod
    def PROP30():
        pass

    @staticmethod
    def PROP40():
        pass

    @staticmethod
    def PROP60():
        pass


class Color:
    @staticmethod
    def BLACK():
        pass

    @staticmethod
    def WHITE():
        pass

    @staticmethod
    def RED():
        pass

    @staticmethod
    def GREEN():
        pass

    @staticmethod
    def BLUE():
        pass

    @staticmethod
    def YELLOW():
        pass

    @staticmethod
    def ORANGE():
        pass

    @staticmethod
    def PURPLE():
        pass

    @staticmethod
    def CYAN():
        pass

    @staticmethod
    def TRANSPARENT():
        pass


class Brain:
    class screen:
        @staticmethod
        def print(text):
            pass

        @staticmethod
        def set_cursor(ROW, COLUMN):
            pass

        @staticmethod
        def next_row():
            print("\n")

        @staticmethod
        def clear_screen():
            pass

        @staticmethod
        def clear_row(ROW=-1):
            pass

        @staticmethod
        def draw_pixel(X, Y):
            pass

        @staticmethod
        def draw_line(START_X, START_Y, END_X, END_Y):
            pass

        @staticmethod
        def draw_rectangle(X, Y, WIDTH, HEIGHT):
            pass

        @staticmethod
        def draw_circle(X, Y, RADIUS):
            pass

        @staticmethod
        def set_font(FONT_TYPE):
            pass

        @staticmethod
        def set_pen_width(PEN_WIDTH):
            pass

        @staticmethod
        def set_pen_color(COLOR):
            pass

        @staticmethod
        def set_fill_color(COLOR):
            pass

        @staticmethod
        def pressed(callback):
            callback()

        @staticmethod
        def released(*args):
            pass

        @staticmethod
        def row():
            return 20

        @staticmethod
        def column():
            return 80

        @staticmethod
        def pressing():
            return False

        @staticmethod
        def x_position():
            return 0

        @staticmethod
        def y_position():
            return 0

        @classmethod
        def draw_image_from_file(cls, param, param1, param2):
            pass

    class battery:
        @staticmethod
        def voltage():
            return 12.0

        @staticmethod
        def current():
            return 16.0

        @staticmethod
        def capacity():
            return 100.0

    class timer:
        @staticmethod
        def event(callback, timee):
            pass

        @staticmethod
        def clear():
            pass

        @staticmethod
        def time(UNITS):
            pass


def wait(*args):
    pass
