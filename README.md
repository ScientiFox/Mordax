

# Mordax
Mordax is a small mobile robot platform intended to develop and test rigorously defined and robust algorithms for applications in behavior-based robotics systems. In support of these studies, the package contains software which tracks the motion of the robot through a camera, as well as a wireless bluetooth module for issuing direct commands to the robot, and reporting system state variables from the robot to a supervisor system.

### Software
There are four pieces of software included herein:

`Mordax_v14.ino` The main firmware for the Mordax robot, implementing all behaviors, communication, and sensing tasks

`Mordax_tune.ino` A simplistic tuning module for the robot, looping all simple behaviors, intended for checking wheel, sensor, and timing parameters

`field_of_view_tracking.py` A tracking script which analyzes video of the robot in operation, using blob-tracking to follow the progress of the robot consistently

`robot_control_tracker.py` An extension of the tracking software which also interfaces the state-reporting of the robot to track its position on grid-like coordinates

### Demos

# Hardware

# Behaviors

