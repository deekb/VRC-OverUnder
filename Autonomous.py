from Utilities import *


class Autonomous(object):
    """
    This class is used to manage the execution of an autonomous routine
    """
    def __init__(self, terminal, motors, drivetrain, _globals, verbosity) -> None:
        self.terminal = terminal
        self.print = self.terminal.print
        self.clear = self.terminal.clear

        self.verbosity = verbosity
        self.motors = motors
        self.drivetrain = drivetrain
        self.globals = _globals
        # Start a new log for this autonomous instance, places it in /Logs/ (if available), on the SD card
        self.autonomous_log_object = Logging(log_format="[%n]:%m:%s\n", log_name="Autonomous")

        self.log = self.autonomous_log_object.log
        self.motors.allWheels.set_stopping(BRAKE)
        self.drivetrain.reset()
        if self.globals.autonomous_task[0] not in [x[0] for x in available_autonomous_routines]:
            self.log("Can't find autonomous routine " + _globals.autonomous_task[0])
            raise RuntimeError("Can't find autonomous")
        self.log("Autonomous:STATUS: Start")
        self.log("Autonomous:STATUS: Running predefined autonomous routine \"" + _globals.autonomous_task[0] + "\"")
        self.print(self.globals.autonomous_task[1])
        self.globals.autonomous_task[1](self)
        self.autonomous_log_object.exit()
        print("Autonomous Complete")

    def log(self, string):
        if self.verbosity:
            print(string)
            if self.verbosity > 1:
                self.autonomous_log_object.log(string)

    def skills(_self):  # noqa
        _self.log("Starting skills")
        _self.drivetrain.reset()
        _self.drivetrain.follow_path([(0, 0), (0, 121.92), (121.92, 121.92), (121.92, 0), (0, 0)])
        _self.drivetrain.move(0, 0, 0)
        _self.log("Done")

    def nothing(_self):  # noqa
        _self.log("Starting nothing")
        _self.log("Done")
        _self.log("That was easy")


available_autonomous_routines = [("Skills", Autonomous.skills), ("Nothing", Autonomous.nothing)]
