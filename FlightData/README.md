# Flight Data

This folder acts as a potential location for the data from the drone's flights. The data from each of the flights are saved as separate `JSON`-files.

## Flight Data Format

Each flight data file contains the following structure:

- `startDateTime`: The start time of the flight data collection (ISO 8601 format).
- `endDateTime`: The end time of the flight data collection (ISO 8601 format).
- `data`: An array of data points collected during the flight. Each data point includes:
  - `time`: The elapsed time in seconds since the start of the flight.
  - `acceleration`: The acceleration values along the x, y, and z axes (in m/s²):
    - `x`: Acceleration along the x-axis.
    - `y`: Acceleration along the y-axis.
    - `z`: Acceleration along the z-axis.
  - `velocity`: The velocity values along the x, y, and z axes (in m/s):
    - `x`: Velocity along the x-axis.
    - `y`: Velocity along the y-axis.
    - `z`: Velocity along the z-axis.
  - `angularVelocity`: The angular velocity values along the x, y, and z axes (in degrees per second):
    - `x`: Angular velocity around the x-axis.
    - `y`: Angular velocity around the y-axis.
    - `z`: Angular velocity around the z-axis.
  - `rotation`: The rotation values (in degrees):
    - `pitch`: Rotation around the x-axis.
    - `roll`: Rotation around the y-axis.
    - `yaw`: Rotation around the z-axis.
  - `controllerPower`: The user-inputted power value (as a percentage).
  - `joystick`: The joystick position values along the x and y axes (normalized between -1 and 1). The combined magnitude of `x` and `y` is always less than or equal to 1:
    - `x`: Joystick position along the x-axis.
    - `y`: Joystick position along the y-axis.
  - `fps`: The frames-per-second values for different components:
    - `app`: The application update rate.
    - `drone`: The drone update rate.
    - `receiver`: The receiver update rate.
  - `motorPowers`: The motor power values for each motor as a percentage between 0% and 100%:
    - `frontLeft`: The power of the front-left motor.
    - `frontRight`: The power of the front-right motor.
    - `backLeft`: The power of the back-left motor.
    - `backRight`: The power of the back-right motor.

