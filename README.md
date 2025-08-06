# Go1 GPS Navigation 

What this project does is simply read through the GPS waypoints from a txt file in the terminal and it commands the Unitree Go1 robot to navigate through these waypoints by using the Arduino GPS data. I am using the Ublox NEO-M8N GPS Module and connected it to the Arduino.

### Setup

- Upload Arduino GPS code to Arduino Uno.
- Connect Arduino to Go1 onboard computer.
- Build and run `main.cpp` on the Go1 onboard computer.
- Place `waypoints.txt` in the same folder.

#### Waypoints Format

Each line in `waypoints.txt` contains two floating point numbers separated by a comma, representing relative offsets in meters (dx, dy) from the robot’s starting GPS position.

Example:
