import math
from vex import *

brain = Brain()


def clear(console=(brain,)):
    """
    Clears a console
    :param console: The console to clear (brain, controller1, controller2, or tuple with any combination)
    """
    if type(console) is tuple:
        for device in console:
            device.screen.clear_screen()
            device.screen.set_cursor(1, 1)
    else:
        console.screen.clear_screen()
        console.screen.set_cursor(1, 1)


def print(text: str, console=(brain,), end: str = "\n"):
    """
    Prints a string to a console
    :param console: The console to print to (brain, controller1, controller2, or tuple with any combination)
    :param text: the text to print to the screen
    :param end: The string to print at the end (defaults to new line)
    """
    if type(console) is tuple:
        for device in console:
            device.screen.set_font(FontType.MONO12)
            device.screen.print(str(text))
            device.screen.print(str(end).replace("\n", ""))
            if "\n" in str(end):
                device.screen.next_row()
    else:
        console.screen.set_font(FontType.MONO12)
        console.screen.print(str(text))
        if end == "\n":
            console.screen.next_row()
        else:
            console.screen.print(str(end))


def apply_deadzone(value: float, dead_zone: float, maximum: float) -> float:
    """
    Apply a dead_zone to the passed value
    :param maximum: The maximum value for the input, helps to smooth out the returned values when the value is outside the dead zone
    :param value: The value to apply a deadzone to
    :param dead_zone: The lowest value for the function to consider "alive"
    :returns: The value with a dead_zone applied
    """
    if abs(value) < dead_zone:
        return 0
    else:
        return (value - math.copysign(dead_zone, value)) * maximum / (
                    maximum - dead_zone)  # Preserve a "live" zone of -maximum to maximum


def check_position_within_tolerance(current_coordinates: tuple, target_coordinates: tuple, max_distance: float):
    """
    Checks whether two sets of coordinates are within a distance from each other
    :param current_coordinates: The set of coordinates corresponding to the robot's position
    :type current_coordinates: tuple[float | float]
    :param target_coordinates: The set of coordinates corresponding to the target's position
    :type target_coordinates: tuple[float | float]
    :param max_distance: the maximum distance apart that should return True
    """
    current_x, current_y = current_coordinates
    target_x, target_y = target_coordinates
    distance = math.sqrt((current_y - target_y) ** 2 + (current_x - target_x) ** 2)
    return distance <= max_distance


class SlewLimit(object):
    """
    Limit the acceleration of a value with a slew limiter
    """

    def __init__(self, max_slew_rate_per_second):
        """
        Initialize a new slew limiter with the specified maximum rate
        :param max_slew_rate_per_second: The maximum rate of acceleration in one second, every time the "update" method is called the value is allowed to increase or decrease by a maximum of this value times the delta time
        :type max_slew_rate_per_second: float
        """
        self.max_slew_rate = max_slew_rate_per_second
        self.previous_value = 0
        self.previous_time = brain.timer.time(SECONDS)

    def update(self, new_value):
        """
        Update the slew function with the most recent speed value
        :param new_value:
        :return:
        """
        delta_value = new_value - self.previous_value
        current_time = brain.timer.time(SECONDS)
        delta_time = current_time - self.previous_time
        self.previous_time = current_time

        if abs(delta_value) > self.max_slew_rate * delta_time:
            self.previous_value += self.max_slew_rate * delta_time * (delta_value / abs(delta_value))
        else:
            self.previous_value = new_value
        return self.previous_value

    def set(self, new_value):
        self.previous_value = new_value
        self.previous_time = brain.timer.time(SECONDS)


def apply_cubic(value: float, linearity: float) -> float:
    """
    Normalize a value across a cubic curve with a linearity
    :param linearity: How close to a linear function the normalizer should use
    :param value: The value to normalize
    :returns: The passed value normalized across a cubic curve
    """
    return value ** 3 + linearity * value / (1 + linearity)


class PIDController(object):
    """
    Wrap a motor definition in this class to use a custom PID to control its movements ie: my_motor = PIDMotor(Motor(...), kp, kd, t)
    Waring, this class disables all motor functionality except the following functions:[set_velocity, set_stopping, stop, spin, velocity]
    Don't worry about calling pid_update, it is done automatically in a new thread unless you specify auto_pid_update as False
    :param motor_object: The motor to apply the PID to
    :param kp: Kp value for the PID: How quickly to modify the speed if it has not yet reached the desired speed
    :param kd: Kd value for the PID: Higher values reduce the speed of response and limit overshoot
    :param t: Time between PID updates
    :param auto_pid_update: Whether to automatically update the PID and flush motor values
    """

    def __init__(self, motor_object, kp: float = 0.4, kd: float = 0.05, t: float = 0.01, auto_pid_update=True):
        self.motor_object = motor_object
        self.kp = kp
        self.kd = kd
        self.delay = t
        self.actual_velocity = 0
        self.error = 0
        self.derivative = 0
        self.output = 0
        self.previous_error = 0
        self.target_velocity = 0
        if auto_pid_update:
            self.pid_thread = Thread(target=self._pid_loop)

    def pid_update(self) -> None:
        """
        Update the PID state with the most recent motor and target velocities and send the normalized value to the motor
        """
        self.actual_velocity = self.motor_object.velocity(PERCENT)
        self.error = self.target_velocity - self.actual_velocity
        self.derivative = (self.error - self.previous_error) / self.delay
        self.output = self.kp * self.error + self.kd * self.derivative
        self.previous_error = self.error
        self.motor_object.set_velocities(self.actual_velocity + self.output, PERCENT)

    def _pid_loop(self) -> None:
        """
        Used to run the PID in a new thread: updates the values the PID uses and handles applying those updated speed values to the motor
        """
        while True:
            self.pid_update()
            wait(self.delay, SECONDS)

    def set_velocity(self, velocity: float) -> None:
        """
        Set the motors target velocity using the PID
        :param velocity: The new target velocity of the motor
        :type velocity: float
        """
        self.target_velocity = velocity


class Logging(object):
    """
    A class that can run multiple logs for different events and store their outputs to the SD card
    """

    def __init__(self, log_name, log_format: str, mode: str = "w"):
        """
        Create a new instance of the class
        :param log_name: The name to use for the log, a number preceded by a hyphen "-" will be appended to this name to avoid overwriting old logs
        :type log_name: str
        :param log_format: The format for the log, %s for the passed string, %m for time in milliseconds, %t for time in seconds %n for the passed function name (if supplied, otherwise "None" will be used)
        :type log_format: str
        :param mode: The mode you want to open the file in
        :type mode: str
        """
        self.log_format = log_format
        index_dict = {}
        log_number = 0
        try:
            index_dict = eval(open("/Logs/index.json", "r").read())
            if str(log_name) in index_dict:
                log_number = index_dict[str(log_name)]
                index_dict[str(log_name)] += 1
            else:
                index_dict[str(log_name)] = 0
                log_number = 0
            with open("/Logs/index.json", "wt") as file:
                file.write(str(index_dict))
        except (OSError, AttributeError):
            try:
                index_dict[str(log_name)] = 0
                with open("/Logs/index.json", "wt") as file:
                    file.write(str(index_dict))
            except (OSError, AttributeError):
                raise RuntimeError("No SD card, can't initialize log: " + str(log_name))
        self.file_name = "/Logs/" + str(log_name) + "-" + str(log_number) + ".log"
        self.file_object = open(self.file_name, mode)
        self.log("Starting log at " + self.file_name, function_name="logging.__init__")

    def log(self, string, function_name=None):
        """
        Send a string to the file, using the log format
        :param string:
        :param function_name:
        """
        temp = self.log_format
        temp = temp.replace("%s", str(string))
        temp = temp.replace("%t", str(brain.timer.time(SECONDS)))
        temp = temp.replace("%m", str(brain.timer.time(MSEC)))
        temp = temp.replace("%n", str(function_name))
        self.file_object.write(temp)

    def exit(self):
        """
        Close the log object
        """
        self.log("Ending log at " + self.file_name, function_name="logging.exit")
        self.file_object.close()


def clamp(value: float, lower_limit=None, upper_limit=None):
    if lower_limit is None or upper_limit is None:
        return value
    if lower_limit is not None:
        if value < lower_limit:
            return lower_limit
    if upper_limit is not None:
        if value > upper_limit:
            return upper_limit
    return value
