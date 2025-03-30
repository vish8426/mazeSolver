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
