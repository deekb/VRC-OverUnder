# Main
**The main module represents the majority of the high-level instructions for the robot. It includes classes for motors, sensors, and controllers, along with a Globals class for storing global variables. The module defines functions for autonomous and driver control, as well as utility functions for debugging. Additionally, it handles the selection of autonomous routines using the controller. The module serves as the entry point for the program execution. It also coordinates the execution of autonomous and driver control based on the competition mode.**

Module dependencies
-------------------

* Constants
* XDrivetrain
* XOdometry
* Utilities
* Autonomous


Classes
-------

* `Motors`: A class representing the motors and motor groups attached to the robot.
  > Descriptions of motor placement are relative to looking down at the robot with the power button on the right of the brain screen. 
  - `motor_1`: The top left motor.
  - `motor_2`: The top right motor.
  - `motor_3`: The bottom right motor.
  - `motor_4`: The bottom Left motor.
  - `allWheels`: All the motors controlling wheels on the robot.
  - `allMotors`: All the motors on the robot.


* `Sensors`: A class representing the sensors attached to the robot.
  - `inertial`: 6-axis IMU: 3-axis accelerometer and 3-axis gyroscope.


* `Controllers`: A class that contains references to the primary and secondary controller.
  - `primary`: The primary controller
  - `secondary`: The secondary controller


* `Globals`: Stores variables that can be modified or accessed by any function in the program. Here you can also set default/initial values for said variables.
  - `autonomous_threads`: This is a list of the threads that will run concurrently during autonomous so that they may be tracked and stopped at the appropriate time.
  - `driver_control_threads`: Same as `autonomous_threads`, but for driver control.
  - `setup_complete`: Whether the pre-match selections on the controller have been completed.
  - `pause_driver_control`: This can be used to temporarily forfeit drivetrain control to an autonomous functions during driver control. For example, if we want to turn towards a goal when a button is pressed.
  - `autonomous_task`: The task to perform during autonomous (should be set by the setup function).


Functions
---------

* `autonomous_handler()`: Coordinate when to run the autonomous function(s).


* `select_autonomous()`: Selects which autonomous routine to execute using the controller.


* `driver_handler()`: Coordinate when to run the driver function(s).


* `on_driver()`: This is the function designated to run when the driver control portion of the program is triggered.


* `on_autonomous()`: This is the function designated to run when the autonomous portion of the program is triggered.


> The following functions are for debugging only and will likely be removed soon
> * `debug_thread()`: This function is for testing purposes and displays the position and direction of the drivetrain. It also allows resetting the drivetrain when button A is pressed and follows a predefined path when button B is pressed.
> 
> 
> * `rotate_to_0_degrees()`: Rotates the drivetrain to 0 degrees.
> 
> 
> * `rotate_to_90_degrees()`: Rotates the drivetrain to 90 degrees.
> 
> 
> * `rotate_to_180_degrees()`: Rotates the drivetrain to 180 degrees.
>
> 
> * `rotate_to_270_degrees()`: Rotates the drivetrain to 270 degrees.
