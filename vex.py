from __future__ import print_function
import builtins as __builtin__
import time
import random
from threading import Thread
from pynput import keyboard
from pynput.keyboard import Key

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
sleep = time.sleep
pressed_keys = []


class Competition:
    def __init__(self, driver_control, autonomous):
        self.competition_state = input('"d": driver control or "a": autonomous, add e/d to the end for enabled/disabled')
        self.previous_competition_state = None
        self.driver_control_function = driver_control
        self.autonomous_function = autonomous
        self.current_thread = None
        self.current_thread_is_started = False
        Thread(target=self.check_state_loop).start()

    def check_state_loop(self):
        while True:
            wait(10, MSEC)
            if is_pressed("e"):
                self.competition_state = self.competition_state[0] + "e"
            if is_pressed("d"):
                self.competition_state = self.competition_state[0] + "d"
            if is_pressed("m"):
                self.competition_state = "d" + self.competition_state[1]
            if is_pressed("c"):
                self.competition_state = "a" + self.competition_state[1]
            if self.competition_state != self.previous_competition_state:
                if self.competition_state.endswith("e"):
                    if self.competition_state.startswith("d"):
                        self.current_thread = Thread(target=self.driver_control_function)
                    elif self.competition_state.startswith("a"):
                        self.current_thread = Thread(target=self.autonomous_function)
                    if not self.current_thread_is_started:
                        self.current_thread.start()
                        self.current_thread_is_started = True
                    print("Enabled")
                else:
                    if self.current_thread_is_started:
                        del self.current_thread
                        self.current_thread_is_started = False
                    print("Disabled")
                self.previous_competition_state = self.competition_state

    def is_enabled(self):
        return self.competition_state.endswith("e")

    def is_autonomous(self):
        return self.competition_state.startswith("a")

    def is_driver_control(self):
        return self.competition_state.startswith("d")


class Inertial:
    def __init__(self, Port):
        print(f"[VEX_EM]:Gyro initialized on {Port}")

    @staticmethod
    def calibrate():
        print("[VEX_EM]:Inertial calibrating")
        time.sleep(1)

    @staticmethod
    def heading(self, *args):
        return random.randint(-360, 360)

    @staticmethod
    def is_calibrating():
        return False

    def set_heading(self, *args):
        pass


class Controller:
    def __init__(self, port):
        self.port = port
        pass

    class screen:
        @staticmethod
        def print(text):
            __builtin__.print("[VEX_EM]:CONSOLE:CONTROLLER: " + text, end="")

        @staticmethod
        def set_cursor(ROW, COLUMN):
            pass

        @staticmethod
        def next_row():
            print("\n")

        @staticmethod
        def clear_screen():
            # command = 'clear'
            # if os.name in ('nt', 'dos'):
            #     command = 'cls'
            # os.system(command)
            __builtin__.print("[VEX_EM]:CONSOLE:CONTROLLER:CLEAR")

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
        def released(callback):
            callback()

        @staticmethod
        def row():
            return 20

        @staticmethod
        def column():
            return 80

        @staticmethod
        def pressing():
            if random.randint(0, 1) == 1:
                return True
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
            return is_pressed(Key.left)

    class buttonRight:
        @staticmethod
        def pressing():
            return is_pressed(Key.right)

    class buttonA:
        @staticmethod
        def pressing():
            return is_pressed("a")

    class buttonB:
        @staticmethod
        def pressing():
            return is_pressed("b")

    def rumble(self, param):
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

    def spin(self, direction):
        pass

    def set_velocity(self, *args):
        pass


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
    def position(self, *args):
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
            __builtin__.print("[VEX_EM]:CONSOLE:BRAIN: " + text, end="")

        @staticmethod
        def set_cursor(ROW, COLUMN):
            pass

        @staticmethod
        def next_row():
            print("\n")

        @staticmethod
        def clear_screen():
            # command = 'clear'
            # if os.name in ('nt', 'dos'):
            #     command = 'cls'
            # os.system(command)
            __builtin__.print("[VEX_EM]:CONSOLE:BrAIN:CLEAR")

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
        def released(callback):
            callback()

        @staticmethod
        def row():
            return 20

        @staticmethod
        def column():
            return 80

        @staticmethod
        def pressing():
            if random.randint(0, 1) == 1:
                return True
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
            time.sleep(timee * 1000)
            callback()

        @staticmethod
        def clear():
            pass

        @staticmethod
        def time(UNITS):
            pass

    class Event:
        @staticmethod
        def broadcast():
            pass

        @staticmethod
        def broadcast_and_wait():
            pass

        @staticmethod
        def __call__(callback):
            callback()

        @staticmethod
        def wait(_time):
            time.sleep(_time)


def wait(_time, _type=MSEC):
    if _type == SECONDS:
        time.sleep(_time)
    elif _type == MSEC:
        time.sleep(_time / 1000)


def is_pressed(key):
    global pressed_keys
    return key in pressed_keys


def on_press(key):
    global pressed_keys
    try:
        if key.char in ("a", "b", "e", "d"):
            pressed_keys.append(key.char)
    except AttributeError:
        if key == Key.left:
            pressed_keys.append(key)
        elif key == Key.right:
            pressed_keys.append(key)


def on_release(key):
    global pressed_keys
    try:
        if key.char in pressed_keys:
            pressed_keys.remove(key.char)
    except BaseException:
        if key in pressed_keys:
            pressed_keys.remove(key)


Brain.screen.print("Starting Up")
Brain.screen.next_row()
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()
Brain.screen.print("Started keyboard listener")
Brain.screen.next_row()
Brain.screen.clear_screen()
