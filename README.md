<img width="600" alt="image" src="https://github.com/user-attachments/assets/d344b167-6ac3-4273-ac5d-b8faf87f817a" />

# Mordax
Mordax is a small mobile robot platform intended to develop and test rigorously defined and robust algorithms for applications in behavior-based robotics systems. In support of these studies, the package contains software which tracks the motion of the robot through a camera, as well as a wireless bluetooth module for issuing direct commands to the robot, and reporting system state variables from the robot to a supervisor system.

### Software
There are four pieces of software included herein:

`Mordax_v14.ino` The main firmware for the Mordax robot, implementing all behaviors, communication, and sensing tasks

`Mordax_tune.ino` A simplistic tuning module for the robot, looping all simple behaviors, intended for checking wheel, sensor, and timing parameters

`field_of_view_tracking.py` A tracking script which analyzes video of the robot in operation, using blob-tracking to follow the progress of the robot consistently

`robot_control_tracker.py` An extension of the tracking software which also interfaces the state-reporting of the robot to track its position on grid-like coordinates

### Demos

Additionally, there are four exemplar videos highlighting the execution of the robot on line mazes with color-coded patches for triggering changes to behavior. 

`everything1.mp4`, `everything2.mp4`, and `everything_track.mp4` illustrate, from the fore view, reverse view, and the tracking system's perspective a full run of the robot engaging in all core behaviors- line following, maze navigation, wall following, obstacle avoidance, and object collection.

`pickup.mp4` illustrates a case in which, guided by a line maze, the robot interacts with another mechanical system, dropping off a sample object for placement

## Hardware

The Modrax hardware specification is designed to provide allowances for three principle general actions- line following, wall following, and object interaction. These three basic actions give rise to the higher order behaviors- maze folloing, onstacle avoidance, and object maneuvering. To accomplish these actions, there are a suit of components that are placed to facilitate- Ground IR sensors, forward and side IR sensors, Drive motors, and the cage actuator. The robot is supported by two wheels and a castor, which is constructed of a cusped plastic fitting with a strong magnet embedded in it, which holds in place a steel ball-bearing.

The robot's power supply is a pair of Li-Fe batteries, which is regulated by a bick-boost level converter. An Arduino Nano MCU is the primary control hardware, with an HC-05 bluetooth to serial link for communication with the supervisor PC. An LCS color sensor is mounted on the underside of the chassis, for detecting color flags on the ground.

Line following is enabled by the ground IR sensors. The control algorithm which provides the behavior will be discussed in the Software section. Here, the physical parameters are discussed. The two sensors are places so that they straddle the guiding line, telling the MCU when it has strayed so that it is drifting away from center orientation. An interesting concern with these sensors is placement relative to the turn axis of the robot. The sensors are placed so that they 'lead' the turn axis of the robot by some portion:

<img width="473" height="228" alt="image" src="https://github.com/user-attachments/assets/23831f11-2cd5-426b-adcf-429d71f161c2" />

_Detail of Ground IR sensors, showing rotation axis (orange dot) and sensor placement_
 
This leading allows the robot to have simple turns which correct for off-center movement when line following. If the sensors do not lead like this (for example, if the robot is moving backwards), then it is possible for the robot to align itself so that in say, a corner, no single turn movement can correct the alignment in a measurable way. That is to say, control is still possible, but it requires knowledge of the robot's direction of travel to reverse the path and attempt a turn correction before encountering the misalignment:

<img width="455" height="224" alt="image" src="https://github.com/user-attachments/assets/7aa709e5-08da-4063-9ff5-9d549523d044" />

_Illustrating the point- no simple turn fixes the alignment measurably (sensor read is orange, blue is the path of the sensor read point in a turn)_

Additionally, because the positive tracking of the robot is not exact,it is intractable to use path reversal in this corrective way, as the exact path cannot be reliably retraced. FOr this reason, the leading alignement of the sensors is used, to avoid this complexity.

Wall following is enabled by the use of a forward IR sensor and a left hand side sensor. In general, the forward sensor is used to detect the wall by proximity, and the left side sensor is used to perform the following. The placement of the forward sensor is straight-forward, and just requires that the sensor be aimed straight-ahead and far enough forward that the readings allow early-enough detection of the wall.

The side sensor, though, requires similar consideration to the ground sensors. We place it ahead of the turn axis as well, so the the turn movement of the robot is correlated with changes in wall distance measurements:

<img width="186" height="272" alt="image" src="https://github.com/user-attachments/assets/04e3fa47-ae61-4f60-9194-ff3f21a5184c" />

_Illustrating left side wall sensor leading the turn axis_

If the sensor is not placed as such, then we run into the same problem, where the readings on the sensor change correlation to movement effects depending on the relative position of the robot to the wall. This can be controlled in a similar way to the line following non-lead mode, but just like that approach, is much less robust than the leading placement implementation.

The object detector, which is used to detect objects that the robot can maneuver, not obstacles (which are detected by the forward wall sensor), must only be placed low enough that it can detect objects, and that the forward obstacle sensor will not also detect the object.

The cage actuator and cage itself must be placed so that a few constraints are met. Firstly, the cage must not obstruct the obstacle sensor when either up or down, when up, it should be sufficiently above the obstacle sensor, and when down, below it. Additionally, the link between the cage armature and the actuator must not interfere with the left side wall sensor. This can be accomplished by either making the wall sensor connections above the 'up' height of the link, or below the 'down' height. Finally, the cage must be able to prevent motion of a 'collected' object when 'down'.

The last geometrically determined hardware design component is the placement of the drive wheels and the castor. The position of the drive wheels should, while maintaning the constraints imposed by the sensor placement, also preserve these relationships: the motion planes of the wheels should be parallel; the line connecting the centers of rotation of the wheels should be normal to the wheel rotation planes; and the castor should be placed so that the robot is statically stable.

## World of the Robot
In describing the Mordax robot, we will be adhering to a certain world design for the robot to be placed in. This world is fairly flexible, and easy to implement, requiring very little special attention that could not be adapted to a real-world application.

The world of the robot is broken down into a few parts, which correspond to the robot's perceptual interpretation of the physical system. These parts are: Obstacles, objects, walls, lines, intersections, dead-ends, and mazes.

<img width="500" alt="basic color course (labeled)" src="https://github.com/user-attachments/assets/34b5686d-3883-4b22-9023-d19c40ec5f3f" />

_An example of the world for the robot_

Obstacles are things which obstruct forward motion of the robot. The robot detects obstacles using its forward IR distance sensor. To be an obstacle, the item must be tall enough that the forward sensor can detect it. By contrast, objects are items which are small enough that the forward object sensor detects them. Objects must be small enough that the robot can drop the 'cage' and maneuver them. If an item cannot be maneuvered, it must be tall enough that it registers and an obstacle.

A wall is a portion of an obstacle when detected by the left hand side IR distance sensor, and is the item of interest during wall following. Typically, the robot detects an obstacle, and turns to navigate around it, during which the obstacle's side is a 'wall'.

Lines are detectable by the robot's ground IR sensors. They provide a means to control where in space the robot can go. The robot, when line following, has control software to position the line between the two ground sensors. The 'floor' of the world is defined as surfaces which can be split into two categories- lines and non-line spaces, which are categories which can be differentiated between reliably by the ground sensors. The reccomended implementation is white surfaces for non-line space (printer paper works well), and black electrical tape for lines.

A maze is a contiguous connection of either lines or obstacles and walls. The standard maze navigation implementation is left-hand-rule path following, which is simple to implemnent for both wall-mazes and line-mazes. 

A line maze is composed of line areas, intersections, and dead ends. Intersections are sections where line regions cross. The robot can detect these areas, and is set to take the leftmost path. A dead end is a hook-shaped line section, which allows the robot to return the direction it came from. Dead-ends allow for termination of a path. Note that because of the left-going standard, the hook must be clockwise chiral.

<img width="531" height="338" alt="image" src="https://github.com/user-attachments/assets/214e850a-e236-42fc-aca4-7c4b6dab5f5a" />
Line maze elements: A- Intersection; B- Dead-end; C- example section of maze

A wall maze is a contiguous section of Obstacles which the robot can navigate around using wall following. Currently, only concave mazes are supported. When implemented, intersections will be convex corners, again keeping with the left-hand rule standard.

Object interactions are simple, with the robot dropping the cage to collect and manipulate objects when detecting one directly in front of the object sensor.

## Behaviors

The Mordax specification also includes a suit of behaviors which enable easy high-level design of useful actions. The basis set of behaviors is comprised of line following, wall following, and obstacle avoidance routines. It also uses a state-machine control structure to conditionally determine when to transition between behaviors.

The first building blocks are the action primitives. For Mordax, these include: reading the IR sensors (ground, forward, side, and object), driving the motors (left and right), actuating the cage (up or down), and any additional convenience functions (beeping a buzzer for indication, reading a globalization sensor, etc.) Use of these action primitives is what defines a 'behavior'. The basic primitives are:

`read_QTR(Y); //Read IR sensor`
`myservoX.write(DSX+Y); //Set Motor or Cage servo`
`beep_(); //Beep piezo`

The servo write function is fully self contained, and beep_() is the simplest single function in the software (see the source code to see its contents). The read_QTR() function is a little complicated, owing to the nature of  the QTR IR distance sensors.

These sensors work by charging up a capacitor, whose discharge rate is controlled by the ir phototransistor which is the receiver for the IR emissions from the sensor. The greater the IR incident on the transistor, the faster the discharge. This means the sensor can measure either distance or reflectance. In either case, the MCU watches counts the number of microseconds for the input pin to switch from high to low. This time length is then proportional to the distance away or inversely proportional to the reflectance (if distanc eis fixed). The function charges up the capacitor by making the pin an output and setting the output high. It then sets the pin to input (after a short charging delay), and counts microseconds until the pin level drops. This time length is then divided down, and returned as the output of the function. 1000 indicates a timeout, which is rare, but possible.

All higher-order behaviors are built based on these primitives. Recall that the 'pin' specified for any function will depend on the particular wiring of any robot.

#### Line Following

The most basic behaviors which Mordax includes are line following, and wall following. The core line following behavior simply tells the robot to go 'straight' until one sensor sees the black line, and to turn back away from the line when it does:

<img width="1033" height="205" alt="image" src="https://github.com/user-attachments/assets/8e787434-ac00-4eaa-b78a-6d96ff7496f4" />

_Illustrating the behavior: A- Drive forward normally; B- Detecting the line in the right side and turning; C- returning to forward motion_

#### Wall Following

The basic wall following routine is just as simple as the line following code. In this routine, sp is the 'set point', or the desired distance from the wall. When the measured distance is greater than this sp + 1, the robot drifts back towards the wall, and when less than sp - 1, drifts away. This drift action has both wheels rotating in the forward direction, but with one wheel moving faster than the other, so as to bias the path of the robot towards or away from the wall.

Note that this is a very robust, if simple, algorithm if the initial orientation of the robot is within certain ranges, which depend on the exact orientation of the sensor, the sensor itself, etc. The edge of the appropriate range is defined as orientation such that the line of sight from the sensor to the wall is perpendicular to the wall, and the intersection of the line-of-sight with the wall must 'lead' the center of rotation of the robot with respect to the position of the center of rotation of the robot, projected on the wall:

<img width="409" height="249" alt="image" src="https://github.com/user-attachments/assets/af3b14cc-a7b8-42be-9ca2-9aa5f47c498c" />

_Orientations: A- stable orientation, sensor line (red) leads center of rotation (CoR) line (yellow) B- unstable orientation, CoR leads sensor line._

Thus, if the wall follow mode activates while the sensor is aimed too far left, reading a distance greater than the set point, the robot will move in the leftward direction- turning the side sensor further away from the wall:

<img width="251" height="247" alt="image" src="https://github.com/user-attachments/assets/d62a588c-9d35-40f5-984c-f4ab150172df" />

_Illustration of 'unsafe' region- D > Sp, so the robot thinks it is too far from the wall, the left going turn makes the situation worse._

In general, as long as you hit within the relatively wide 'safe' range, it is stable. The obstacle detection behavior incorporates checks to ensure that the starting point is within this safe range.

#### Line Follow with Corner Turn

The next step up in behavior complexity is the expanded line follow, which accomodates line maze navigation by adding support for intersections and dead ends. This takes the form of an added case- for right ground and left ground sensors both reading the line:

`  if ((RG>5)&(LG>5)){`
`     corner_turn();}`

The corner_turn() behavior causes the robot to take a left handed turn and re-align itself with the line. As a result, when the robot encounters a line when not straddling one, it will merge onto the line, with a leftward heading. Likewise, when encountering an intersection, it will take the leftmost path. One of the subtle properties of the line following behavior is that the robot will line itself up perpendicular to the intersection or line before executing this corner turn behavior, because both ground sensors must be over the line to trigger it. This alignment helps stabilize the robot's orientation.

This routine has the robot to pivot about the left wheel, with a small backward motion on the left wheel. This arcs the robot around the line it started perpendicular to, making a smooth left turn, and immediatly realigning it with the new line:

<img width="927" height="323" alt="image" src="https://github.com/user-attachments/assets/78bb8582-4b37-4ffc-8d1a-41aa3d598621" />

_Corner turn behavior: A- encountering the intersection, both sensors reading the line; B- pivoting, keeping the left side in the lower left quadrant, moving right side to the upper left quadrant; C- returning to basic line following, having taken the left turn._

Within this routine, it is noticeable that there are two copies of the shorter process. This improves the smoothness when at an intersection. Since the routine transitions when seeing that the right side ground sensor is out of the line, the robot will switch back to line following after the first line is crossed. At an intersection, the robot would then align and turn again, but this is not really necessary. The second cycle left the robot move directly to the next cross section. When encountering a line freely, the section is simply triggered and rapidly cleared.

#### Obstacle Avoidance

The final core behavior is the obstacle avoidance routine, which uses both line following and wall following to accomodate the placement of obstacles which obstruct a path in a line maze. The behavior runs as line following until an obstacle is detected, at which point it turns so as to aim the left side wall sensor at the obstacle. It then performs wall following until it observes the line again. Once it sees the line, it initiates a corner_turn, aligning it facing the obstacle. It then does a short line follow to align itself straddling the line, and finally pivots about the right wheel until the left ground sensor is over the line. This has the robot positioned facing away from the obstacle, and correctly for line following to align it properly to continue traversing the line maze.

Note the trend of implementing higher order behaviors as state machines of lower order behaviors- nested state machines, essentially. Also, some of the state transitions include a timing condition as well, this is to prevent accidental transitions, such as seeing the line the robot just left upon detecting the obstacle as the line it should rejoin.






