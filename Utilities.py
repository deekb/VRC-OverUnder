import math
from vex import *


class Terminal(object):
    def __init__(self, brain):
        self.brain = brain
        self.log = Logging("console", "%s")

    def clear(self):
        """
        Clears the brain screen
        """
        self.brain.screen.clear_screen()
        self.brain.screen.set_cursor(1, 1)

    def print(self, text: str, end: str = "\n"):
        """
        Prints a string to a console
        :param text: the text to print to the screen
        :param end: The string to print at the end (defaults to new line)
        """
        self.brain.screen.print(str(text))
        self.brain.screen.print(str(end).replace("\n", ""))
        if str(end).startswith("\n"):
            self.brain.screen.next_row()


def apply_deadzone(value: float, dead_zone: float, maximum: float) -> float:
    """
    Apply a dead_zone to the passed value
    :param maximum: The maximum value for the input, helps to smooth out the returned values when the value is outside the dead zone
    :param value: The value to apply a deadzone to
    :param dead_zone: The lowest value that should have a nonzero output
    """
    if abs(value) < dead_zone:
        return 0
    else:
        return maximum / (maximum - dead_zone) * (value - dead_zone)


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


def apply_cubic(value: float, linearity: float) -> float:
    """
    Normalize a value across a cubic curve with a linearity
    :param linearity: How close to a linear function the normalizer should use
    :param value: The value to normalize
    :returns: The passed value normalized across a cubic curve
    """
    return value ** 3 + linearity * value / (1 + linearity)


class MotorPID(object):
    """
    Wrap a motor definition in this class to use a custom PID to control its movements ie: my_motor = MotorPID(Motor(...), kp, kd, t)
    Waring, this class disables all motor functionality except the following functions:[set_velocity, set_stopping, stop, spin, velocity]
    :param motor_object: The motor to apply the PID to
    :param kp: Kp value for the PID: How quickly to modify the target value if it has not yet reached the desired value
    :param ki: Ki value for the PID: Integral gain to reduce steady-state error
    :param kd: Kd value for the PID: Higher values reduce the response time and limit overshoot
    :param t: Time between PID updates
    """

    def __init__(self, timer: Brain.timer, motor_object, kp: float = 0.4, ki: float = 0.01, kd: float = 0.05, t: float = 0.1) -> None:
        self.motor_object = motor_object
        self.motor_PID = PIDController(timer, kp, ki, kd)
        self.pid_thread = Thread(self.loop)
        self.t = t

    def update(self) -> None:
        """
        Update the PID state with the most recent motor and target velocities and send the normalized value to the motor
        """
        self.motor_object.set_velocity(self.motor_PID.update(self.velocity(PERCENT)), PERCENT)

    def loop(self) -> None:
        """
        Used to run the PID in a new thread:
         updates the values the PID uses and handles
          applying those updated unsigned_target_speed values to the motor
        """
        while True:
            self.update()
            wait(self.t, SECONDS)

    def set_velocity(self, velocity: float) -> None:
        """
        Set the motors target velocity using the PID, make sure you run PID_loop in a new thread or this
        will have no effect
        :param velocity: The new target velocity of the motor
        :type velocity: float
        """
        self.motor_PID.target_value = velocity

    def spin(self, DIRECTION):
        self.motor_object.spin(DIRECTION)

    def stop(self):
        self.motor_object.stop()

    def velocity(self, _type):
        return self.motor_object.velocity(_type)


class PIDController(object):
    """
    A generalized PID controller implementation.
    """

    def __init__(self, timer: Brain.timer, kp: float = 1, ki: float = 0, kd: float = 0):
        """
        Initializes a PIDController instance.
        :param timer: The timer object used to measure time.
        :param kp: Kp value for the PID.
        :param ki: Ki value for the PID.
        :param kd: Kd value for the PID.
        """

        self._kp = kp
        self._ki = ki
        self._kd = kd
        self.timer = timer
        self.previous_time = timer.time(MSEC) / 1000
        self.current_value = 0
        self._target_value = 0
        self.error_integral = 0
        self.previous_error = 0
        self.control_output = 0

    @property
    def kp(self) -> float:
        """
        Getter for the Kp value of the PID.
        :return: The Kp value.
        """
        return self._kp

    @kp.setter
    def kp(self, value: float):
        """
        Setter for the Kp value of the PID.
        :param value: The new Kp value.
        """
        self._kp = value

    @property
    def ki(self) -> float:
        """
        Getter for the Ki value of the PID.
        :return: The Ki value.
        """
        return self._ki

    @ki.setter
    def ki(self, value: float):
        """
        Setter for the Ki value of the PID.
        :param value: The new Ki value.
        """
        self._ki = value

    @property
    def kd(self) -> float:
        """
        Getter for the Kd value of the PID.
        :return: The Kd value.
        """
        return self._kd

    @kd.setter
    def kd(self, value: float):
        """
        Setter for the Kd value of the PID.
        :param value: The new Kd value.
        """
        self._kd = value

    @property
    def target_value(self) -> float:
        """
        Getter for the target value of the PID.
        :return: The target value.
        """
        return self._target_value

    @target_value.setter
    def target_value(self, value: float):
        """
        Setter for the target value of the PID.
        :param value: The new target value.
        """
        self._target_value = value
        self.error_integral = 0
        self.previous_error = self._target_value - self.current_value
        self.control_output = 0

    def update(self, current_value: float) -> float:
        """
        Update the PID state with the most recent current value and calculate the control output.
        :param current_value: The current measurement or feedback value.
        :return: The calculated control output.
        """

        current_time = self.timer.time(MSEC) / 1000
        delta_time = current_time - self.previous_time
        self.previous_time = current_time

        delta_time = clamp(delta_time, 0.00001, 1)   # Clamp delta_time within the range [0.00001, 1]

        current_error = self.target_value - current_value
        self.error_integral += current_error * delta_time
        error_derivative = (current_error - self.previous_error) / delta_time
        self.control_output = (
            self.kp * current_error + self.ki * self.error_integral + self.kd * error_derivative
        )
        self.previous_error = current_error
        return self.control_output


class Logging(object):
    """
    A class that can run multiple logs for different events and store their outputs to the SD card
    """

    def __init__(self, log_name, log_format: str):
        """
        Create a new instance of the class
        :param log_name: The name to use for the log, a number preceded by a hyphen "-" will be appended to this name to avoid overwriting old logs
        :type log_name: str
        :param log_format: The format for the log, %s for the passed string, %m for time in milliseconds, %t for time in seconds %n for the passed function name (if supplied, otherwise "None" will be used)
        :type log_format: str
        """
        self.log_format = str(log_format)
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
        except (OSError, AttributeError, SyntaxError):
            try:
                index_dict[str(log_name)] = 0
                with open("/Logs/index.json", "wt") as file:
                    file.write(str(index_dict))
            except (OSError, AttributeError):
                raise RuntimeError("No SD card, can't initialize log: " + str(log_name))
        self.file_name = "/Logs/" + str(log_name) + "-" + str(log_number) + ".log"
        self.file_object = open(self.file_name, "w")
        self.log("Starting log at " + self.file_name, function_name="logging.__init__")

    def log(self, string: str, function_name: str = ""):
        """
        Send a string to the file, using the log format
        :param string:
        :param function_name:
        """
        temp = self.log_format
        temp = temp.replace("%s", str(string))
        temp = temp.replace("%n", str(function_name))
        self.file_object.write(temp)

    def exit(self):
        """
        Close the log object
        """
        self.log("Ending log at " + self.file_name, function_name="logging.exit")
        self.file_object.close()


def clamp(value: float, lower_limit: float = None, upper_limit: float = None) -> float:
    """
    Restricts a value within a specified range.
    :param value: The value to be clamped.
    :param lower_limit: The lower limit of the range.
        If None, no lower limit is applied.
    :param upper_limit: The upper limit of the range.
        If None, no upper limit is applied.
    :return: The clamped value.
    """

    if lower_limit is None or upper_limit is None:
        return value
    if lower_limit is not None:
        if value < lower_limit:
            return lower_limit
    if upper_limit is not None:
        if value > upper_limit:
            return upper_limit
    return value
