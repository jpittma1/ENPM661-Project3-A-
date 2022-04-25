# ENPM661-Project3-A*
A* implementation for a mobile robot for Spring 2022 ENPM661 course at UMD-CP
# Phase 1: Non-holonomic constraints
# Phase 2: Add holonomic constraints (differential Drive of Turtlebot)

UMD-CP PMRO. Spring 2022
#Section 0101

#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 


# Project 3: Phase 1 (non-holonomic constraints only)

#********************************************
# Requires the following in same folder to run:
#1) Python code: "functions.py", "Node.py", and "obstacles.py" and "jerry_pittman.py"

# Generates the following outputs:
#1) Initial Map Image : "initial_map.png"
#2) Final Map Image (with path): "Final_map.png"
#3) Video for phase 1: "project3_jerry_pittman.avi"
#********************************************

# Phase 2 Part 1 (non-holonomic constraints only)
#********************************************
# Requires the following in same folder to run:
#1) Python code: "functions.py", "Node.py", and "obstacles.py" and "jerry_pittman.py"

to run: "python3 jerry_pittman.py"

# Outputs:
path_nodes.csv
vel_points.csv
path_nodes.csv

# Phase 2 Part 2 Gazebo Simulation
#********************************************
#1) ROS files: turtleastar_ws and its subfolders built as a catkin workspace

to run, commands in terminal: or Gazebo: Build the astar_ws using catkin build (may require catkin clean - b)

	Source the workspace 
		source ~/astar_ws/devel/setup.bash

	Launch the file:

	roslaunch astar_turtlebot3 enviroment.launch x_init:=5 y_init:=3 theta_init:=0 x_final:=9 y_final:=9 rpm1:=15 rpm2:=10
	roslaunch astar_turtlebot3 enviroment.launch x_init:=6 y_init:=8 theta_init:=0 x_final:=9 y_final:=9 rpm1:=15 rpm2:=10
	roslaunch astar_turtlebot3 enviroment.launch x_init:=8 y_init:=5 theta_init:=0 x_final:=7 y_final:=7 rpm1:=15 rpm2:=10


Function modules used: numpy, cv2, scipy, matplotlib.pyplot, sys, math, copy, timeit, queue

Github Repo: https://github.com/jpittma1/ENPM661-Project3-Astar.git

Video1 GDrive link: https://drive.google.com/file/d/1mVRywEdmyglWI77xN5PO4NGf_e3jIXUo/view?usp=sharing

Video2 (phase 2 part 2) GDrive link: https://drive.google.com/file/d/1zTlrJZids36evF3Qnpi_c_KSNqX0G8Cq/view?usp=sharing

Video3 (phase 2 part 2) GDrive Link: 
