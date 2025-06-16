# mazeSolver
Maze Solver TurtleBot 

## About
The problem or exercise requires the Turtlebot to successfully complete or solve a maze. 

This is to be both in a simulation as well as a physical model. The physical model of the maze is to be created using cardboard with the Turtlebot moving through the cardboard 
environment (where most of the material is utilized represents the maze walls). The simulation is modelled with a combination of programs including:
- RViz
- Gazebo
- ROS Kinetic & Melodic
- Ubuntu 18.04 & 16.04 

The main method of completing the maze would involve the use of wall following. The method uses only detection of a wall to guide the robot through the maze. The robot will 
continue this motion until it reaches the maze exit. The simulation will involve further modification of a turtle bot drive file with the addition of a tested algorithm of turtle bot left-wall follower.  

The physical simulation that requires autonomous solving of the turtlebot employs the use of a remote PC program to monitor the progress of completion of the robot in the maze. All decisions, movement and overall solution is to occur autonomously on the robot. There are three levels of independence that are an objective: 
1. Remote Run:
   - Remote computer runs the program.
   - Starts immediately on request.
   - Roscore and decision-making node run on turtlebot.
   - Program runs autonomously without need of remote PC.
     
2. Remote Prime:
   - Remote connection to run the program.
   - Main node sits idle until pushbutton is pressed on the robot.
   - Sits idle for two seconds then runs (if button is pressed).
   - Runs independently from the remote PC.

3. No Remote PC:
   - PC runs automatically on boot.
   - Edit .bashrc on the robot to make your program launch automatically.
   - Waits for a pushbutton to be pressed.
   - Sits idle for two seconds and runs. 

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Operational Modes](#operational-modes)
- [Resources](#resources)
- [License](#license)

## Overview

The MazeSolver project focuses on developing an autonomous TurtleBot capable of solving mazes without human intervention. The system is designed to operate in both simulated environments using ROS tools and in physical mazes constructed from cardboard. The primary navigation strategy employed is the left-wall-following algorithm, allowing the robot to traverse the maze by consistently keeping its left side adjacent to a wall.

## Features

- **Dual Environment Operation**: Supports both simulation and real-world maze navigation.
- **Wall-Following Algorithm**: Implements a left-wall-following strategy for maze traversal.
- **Autonomous Decision Making**: The robot makes navigation decisions without external input.
- **ROS Integration**: Utilizes ROS Kinetic & Melodic for simulation and control.
- **Visualization Tools**: Employs RViz and Gazebo for simulation visualization and testing.

## System Architecture

The MazeSolver system comprises several components:

- **Simulation Environment**:
  - **ROS Versions**: Compatible with ROS Kinetic and Melodic.
  - **Operating Systems**: Tested on Ubuntu 16.04 and 18.04.
  - **Simulation Tools**: Utilizes RViz and Gazebo for creating and visualizing the maze environment.
  - **Algorithm Implementation**: The left-wall-following algorithm is integrated into the TurtleBot's control system.

- **Physical Implementation**:
  - **Maze Construction**: Physical mazes are built using cardboard to represent walls.
  - **Robot Hardware**: The TurtleBot is equipped with necessary sensors to detect walls and navigate the maze.
  - **Autonomous Operation**: The robot operates independently, making real-time decisions based on sensor input.

## Operational Modes

The system supports three levels of operational independence:

1. **Remote Run**:
   - The program is initiated from a remote computer.
   - The TurtleBot runs `roscore` and the decision-making node autonomously.
   - After initiation, the robot operates without further remote assistance.

2. **Remote Prime**:
   - The program is launched via a remote connection.
   - The main node remains idle until a physical pushbutton on the TurtleBot is pressed.
   - After a 2-second delay post-button press, the robot begins autonomous operation.

3. **No Remote PC**:
   - The TurtleBot is configured to start the program automatically on boot by editing the `.bashrc` file.
   - The robot waits for a pushbutton press to commence operation.
   - After a 2-second delay, the robot starts navigating the maze autonomously.

## Resources

- **Project Report**:
  - [`Maze Solver TurtleBot Project Report.pdf`](Maze%20Solver%20TurtleBot%20Project%20Report.pdf): Comprehensive documentation detailing the project's objectives, methodology, and findings.

- **Source Code**:
  - [`SamplePackage/`](SamplePackage/): Contains the implementation of the wall-following algorithm and related control scripts.
  - [`CMakeLists.txt`](CMakeLists.txt): Build configuration for the project.

## License

This project is licensed under the [MIT License](LICENSE), allowing for open collaboration and modification.
