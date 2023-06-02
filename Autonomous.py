from Utilities import *


class Autonomous(object):
    def __init__(self, motors, drivetrain, _globals):
        print("Created Autonomous object")
        # Start a new log for this autonomous instance, places it in /Logs (if available), on the SD card
        self.autonomous_log_object = Logging(log_format="[%n]:%m:%s\n", mode="wt", log_name="Autonomous")
        self.log = self.autonomous_log_object.log
        motors.allWheels.set_stopping(BRAKE)
        drivetrain.reset()
        if _globals.autonomous_task[0] not in available_autonomous_routines:
            self.log("Can't find autonomous routine " + _globals.autonomous_task[0])
            raise RuntimeError("Can't find autonomous")
        self.log("Autonomous:STATUS: Start")
        self.log("Autonomous:STATUS: Running predefined autonomous routine \"" + _globals.autonomous_task[0] + "\"")
        _globals.autonomous_task[1]()
        print("Autonomous Complete")

    class Skills(object):
        def __init__(self):
            print("Skills running")
            wait(1000)
            print("done")


available_autonomous_routines = [("Skills", Autonomous.Skills)]
