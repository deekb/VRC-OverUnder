from Utilities import *


class Autonomous(object):
    def __init__(self, motors, drivetrain, _globals, verbosity) -> None:
        self.verbosity = verbosity
        self.motors = motors
        self.drivetrain = drivetrain
        self.globals = _globals
        # Start a new log for this autonomous instance, places it in /Logs/ (if available), on the SD card
        self.autonomous_log_object = Logging(log_format="[%n]:%m:%s\n", mode="wt", log_name="Autonomous")

        self.log = self.autonomous_log_object.log
        self.motors.allWheels.set_stopping(BRAKE)
        self.drivetrain.reset()
        if self.globals.autonomous_task[0] not in available_autonomous_routines:
            self.log("Can't find autonomous routine " + _globals.autonomous_task[0])
            raise RuntimeError("Can't find autonomous")
        self.log("Autonomous:STATUS: Start")
        self.log("Autonomous:STATUS: Running predefined autonomous routine \"" + _globals.autonomous_task[0] + "\"")
        self.globals.autonomous_task[1]()
        self.autonomous_log_object.exit()
        print("Autonomous Complete")

    def log(self, string):
        if self.verbosity:
            print(string)
            if self.verbosity > 1:
                self.autonomous_log_object.log(string)

    def skills(self):
        def __init__(self) -> None:
            self.log("Starting skills")
            wait(1000)
            self.log("Done")

    def nothing(self):
        def __init__(self) -> None:
            self.log("Starting nothing")
            self.log("Done")
            self.log("That was easy")


available_autonomous_routines = [("Skills", Autonomous.skills), ("Nothing", Autonomous.nothing)]
