# Flight Data

This folder acts as a potential location for the data from the drone's fligths. The data form each of the flights are saved as separate `JSON`-files.

## Flight Data Format

Each flight data file contains the following structure:

- `startDateTime`: The start time of the flight data collection (ISO 8601 format).
- `endDateTime`: The end time of the flight data collection (ISO 8601 format).
- `data`: An array of data points collected during the flight. Each data point includes:
  - `time`: The elapsed time in seconds since the start of the flight.
  - `acceleration`: The acceleration values along the x, y, and z axes.
  - `velocity`: The velocity values along the x, y, and z axes.
  - `angularVelocity`: The angular velocity values along the x, y, and z axes.
  - `rotation`: The rotation values (pitch, roll, yaw).
  - `power`: The power information.
  - `joystick`: The joystick position values along the x and y axes.
  - `fps`: The frames-per-second values for:
    - `app`: The application update rate.
    - `drone`: The drone update rate.
    - `receiver`: The receiver update rate.

