# CDE2310 Group 8 Rescue Robot: The ZelephantBot
Hyunjin Kim, Thia Yang Han, Mayukh Ghosh, Justin Matthew Tunaldi <br/>

CDE2310 - Fundamentals of Systems Design, AY 24/25, <br/>
Innovation and Design Programme, National University of Singapore (NUS). <br/>
![alt text](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/blob/main/Documentations/Mechanical_Files/image.png)



## Description
The ZelephantBot, developed by NUS IDP students from CDE2310, is a modified Turtlebot 3 originally provided by Robotis Co. Our modifications consist of adding a heat-detection and a flywheel ball launching system to accomplish the mission objectives which are: Autonomous navigation within a closed maze, locating three heat sources (humans), and launching ping-pong balls vertically upwards in a pre-programmed sequence. 

This repository contains all the code that we have used for our ZelephantBot in Ubuntu 22.04.5, ROS2 Humble and Python3 to explore a closed maze using a Naive Global Wavefront Frontier approach with Breadth-First Search. Do check out our [project report](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/blob/main/Documentations/Final%20Project%20Report.pdf) under the [Documentations](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/tree/main/Documentations) folder for more details on how we have developed our ZelephantBot. There are 2 unresolved issues which you can check under Issues if you plan on developing our code for your application.

## File Organisation
- [Documentations](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/tree/main/Documentations) folder contains all the documentation we have made for the ZelephantBot. This includes the project report (which contains the assembly manual and the troubleshooting guide), full software setup guide, the end user documentation, CG derivation, and other relevant mechanical files and drawing.
- [laptop_programs](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/tree/main/laptop_programs) contains all the relevant files that need to be installed on the remote laptop for navigation and coordination. This includes the following files:
  - [coordinator_node.py](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/blob/main/laptop_programs/coordinator_node.py) is the main program enabling coordination between all the other Remote Laptop and RPi programs. It also contains the code for seeking a heat source using Navigation 2. 
  - [auto_mapper.cpp](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/tree/main/laptop_programs/frontier_exploration/auto_mapper/src) is our frontier-based exploration algorithm using Navigation 2.
  - [r2auto_nav_modified.py](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/blob/main/laptop_programs/r2auto_nav_modified.py) is our backup navigation code when frontier exploration fails.

- [rpi_programs](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/tree/main/rpi_programs) contains all the code that needs to be installed on the RPi for heat detection and for launching the ping-pong balls:
  - [heat_detection_node.py](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/blob/main/rpi_programs/heat_detection_node.py) is the code for detecting and turning the ZelephantBot towards the heat source
  - [motor_driver.py](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/blob/main/rpi_programs/motor_driver.py) is the code for turning on our flywheels and servos in a sequence for launching the ping-pong balls. 

- [Original_Files](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/tree/main/Original_Files) folder is an archive of the forked repository from [NickInSynchronicity’s r2auto_nav_CDE2310](https://github.com/NickInSynchronicity/r2auto_nav_CDE2310) and is not necessary for the ZelephantBot’s operations.
- [Miscellaneous_Files](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/tree/main/Miscellaneous_Files) folder is an archive of the development and testing versions of our code, so feel free to check that out to see how we progressed through our software development, although it is not consistent.



## Calibration and configuration
### Frontier-Based Navigation Code
You may experiment with different parameters to calibrate the navigation algorithm to suit your needs.
| Variable name| Description| Recommended value  |
| ------------- |:-------------| ---:|
| MIN_FRONTIER_DENSITY | Minimum number of neighbouring frontiers required to be a valid frontier | 0.10 |
| MIN_DISTANCE_TO_FRONTIER_SQUARED| Minimum squared distance from robot to be a valid frontier| 0.4225 |
| MIN_FREE_THRESHOLD | Minimum number of free cells adjacent required to be a valid frontier | 2 |
| MAX_OBSTACLE_THRESHOLD | Maximum number of occupied cells adjacent allowed to be a valid frontier | 1 |

### Coordinator Program Code
| Variable name| Description| Recommended value  |
| ------------- |:-------------| ---:|
| radius_of_search | Radius around flared heat sources in order to prevent the robot from seeking the same heat sources | 0.4 |
| heat_distance_threshold | Distance to publish a heat seeking Nav2 Goal | 0.4 |

### R2AutoNav Backup Navigation Code
| Variable name| Description| Recommended value  |
| ------------- |:-------------| ---:|
| rotatechange | Angular speed | 0.5 |
| speedchange | Linear speed | 0.1 |
| stop_distance | Distance away from the wall to stop the robot before turning | 0.4 |
| max_runtime | How long to keep backup program running | 25 seconds |

### Heat Detection Code
| Variable name| Description| Recommended value  |
| ------------- |:-------------| ---:|
| rotatechange | Angular speed | 0.3 |
| temperature_capture_angle | Angle interval to record temperature | >= 6 |
| heat_detected_threshold | Temperature at which a heat source will be considered detected | ~25.5 (depending on room temperature) |



### Software Installation and Operating Instructions
Instructions on how to install, setup, and operate the necessary software used in our ZelephantBot can be found in our [Software Setup Manual](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/blob/main/Documentations/Software%20Setup%20Manual.pdf). Please note: These instructions are specifically intended for users running Ubuntu, either natively or through the Windows Subsystem for Linux (WSL), as our environment and tools are optimized for this setup. For any troubleshooting, please refer to the relevant section in our [report](https://github.com/hyunjinkim1112/r2auto_nav_CDE2310/blob/main/Documentations/Final%20Project%20Report.pdf) for more details. 




