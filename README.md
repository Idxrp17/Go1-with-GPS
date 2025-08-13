# Go1 GPS Navigation 

What this project does is simply read through the GPS waypoints from a txt file in the terminal and it commands the Unitree Go1 robot to navigate through these waypoints by using the Arduino GPS data. I am using the Ublox NEO-M8N GPS Module and connected it to the Arduino.


## Equipment
- UBLOX NEO-M8N GPS Module with antenna
- Arduino Uno R3 board
- Portable Monitor - USB-C ~(5V, 3A)
- Portable Power Bank (USB-C)
- Female to Male Dupont Wires
- Anker Ultra Slim 4-Port USB 3.0 Data Hub (or any usb to either 2-3 usb hub)


## Setup
- Connect Neo-M8N to Arduino board
  1. 



## Waypoints Format

Each line in the waypoints.txt file has numbers separated by a comma. These represent the offsets in meters (dx, dy) based on the starting GPS position.

Example:

-3.0,5.0

0.5,2.0

2.0,-4.0

## Arduino txt file

You can read the GPS data from the Arduino on the pi by creating a txt file in the terminal such as `arduinogps.py` and execute it (python3)
