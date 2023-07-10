XDrivetrain
====================

**This module provides a drivetrain controller for an X drive base. It allows you to control the drivetrain's movements, move to specific positions, follow paths, and perform other related operations.**

Classes
-------


* `Drivetrain`: Represents a drivetrain object.
  - `__init__()`: Initializes a new drivetrain with the specified properties.
    > The following are arguments for the drivetrain initializer
    - `brain`: The brain object, whichever variable has the value `Brain()`.
    - `inertial`: The inertial sensor to use for the drivetrain.
    - `motor_1`: Motor 1.
    - `motor_2`: Motor 2.
    - `motor_3`: Motor 3.
    - `motor_4`: Motor 4.
    - `movement_allowed_error_cm`: The distance from the target that is acceptable or close enough.
    - `wheel_radius_cm`: The radius of the wheels.
    - `track_width_cm`: The parallel distance between the wheels in centimeters.
    - `motor_lowest_speed`: The speed at which the motors can just barely spin (defaults to 1 for accuracy but can be set higher if your drivetrain has more friction).
    - `driver_control_deadzone`: The minimum value from the controller that should be treated as alive (non-zero) (defaults to 0.1, range: 0.0-1.0).
    - `driver_control_turn_speed`: The turn speed for driver control (defaults to 3).
    - `direction_correction_kp`: Kp value for the direction correction PID (defaults to 1).
    - `direction_correction_ki`: Ki value for the direction correction PID (defaults to 0).
    - `direction_correction_kd`: Kd value for the direction correction PID (defaults to 0).

  - `move_to_position(self, target_position, maximum_speed: float = 0.35) -> None`: Move to the specified position.
    - `target_position`: The position to move to (tuple[float, float]).
    - `maximum_speed`: The maximum speed between 0 and 1 for the robot to move while trying to reach the target_position.

  - `follow_path(self, point_list)`: Move the robot along a given path.
    - `point_list`: The list of points (positions) to follow.

  - `move(self, direction, speed, spin) -> None`: Move the drivetrain towards a vector.
    - `direction`: The direction to move in.
    - `speed`: The speed of movement.
    - `spin`: The spin value.

  - `move_headless(self, direction, magnitude, spin)`: Move the drivetrain in a headless manner (without considering the current heading).
    - `direction`: The direction to move in.
    - `magnitude`: The magnitude of movement.
    - `spin`: The spin value.

  - `move_with_controller(self, controller: Controller, headless: bool = False) -> None`: Move the drivetrain using controller input.
    - `controller`: The controller to get input from.
    - `headless`: Whether to move the robot in headless mode (defaults to False).

  - `reset(self) -> None`: Reset the drivetrain to its newly-instantiated state.

Properties
----------

* `target_position`: The position of the robot (tuple[float, float]).
* `target_heading_rad`: The current target heading in radians.
* `target_heading_deg`: The current target heading in degrees.

Static Methods
--------------

* `calculate_wheel_power(movement_angle_rad, movement_speed, wheel_angle_rad) -> float`: Calculate the necessary wheel power for a wheel pointing in the specified angle to move the robot toward the desired target.
