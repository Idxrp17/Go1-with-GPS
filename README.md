# Go1 GPS Navigation 

What this project does is simply read through the GPS waypoints from a txt file in the terminal and it commands the Unitree Go1 robot to navigate through these waypoints by using the Arduino GPS data. I am using the Ublox NEO-M8N GPS Module and connected it to the Arduino.

## Setup

- Upload Arduino GPS code to Arduino Uno.
- Connect Arduino to Go1 onboard computer.
- Build and run `main.cpp` on the Go1 onboard computer.
- Place `waypoints.txt` in the same folder.

## Waypoints Format

Each line in the waypoints.txt file has numbers separated by a comma. These represent the offsets in meters (dx, dy) based on the starting GPS position.

Example:

-3.0,5.0

0.5,2.0

2.0,-4.0

## Tester

To make sure the GPS data gets received on the Go1's onboard pi, you can run a txt file to see the x,y (lat, lon) coordinates, sat #, speed, etc. 

## Requirements

- Unitree Legged SDK v3.8.0
- Linux with the /dev/ttyACM0 serial port and make sure its connected to Arduino GPS
