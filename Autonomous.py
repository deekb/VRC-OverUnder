from main import Motors, drivetrain, Globals
from Utilities import *


class Autonomous(object):
    def __init__(self):
        print("Created Autonomous object")
        # Start a new log for this autonomous instance, places it in /Logs (if available), on the SD card
        self.autonomous_log_object = Logging(log_format="[%n]:%m:%s\n", mode="wt", log_name="Autonomous")
        self.log = self.autonomous_log_object.log
        Motors.allWheels.set_stopping(BRAKE)
        drivetrain.reset()
        possible_tasks = [value for name, value in vars(self).items() if not name.startswith("_")]
        if Globals.autonomous_task not in possible_tasks:
            self.log(f"Can't find autonomous class {Globals.autonomous_task}")
            raise RuntimeError("Can't find autonomous")
        self.log("Autonomous:STATUS: Start")
        self.log("Autonomous:STATUS: Running predefined autonomous routine \"" + Globals.autonomous_task + "\"")
        Globals.autonomous_task()
        print("Autonomous Complete")

    class Skills(object):
        def __init__(self):
            print("Skills running")
            wait(1000)
            print("done")
