# ENPM661-Project3-A*
A* implementation for a mobile robot for Spring 2022 ENPM661 course at UMD-CP
# Phase 1: holonomic constraints
# Phase 2: Add non-holonomic constraints (differential Drive of Turtlebot) and simulate in Gazebo (Part 2)

UMD-CP PMRO. Spring 2022
#Section 0101

#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 


 Phase 2 Part 2 Gazebo Simulation
#********************************************
<p align="center">
<img src="data/path_1.gif"/>
</p>
<p align="center">
<img src="data/path_2.gif"/>
</p>

#1) ROS files: turtleastar_ws and its subfolders built as a catkin workspace

to run, commands in terminal:
or Gazebo:
		Build the astar_ws using catkin build (may require catkin clean - b)

		Source the workspace 
			source ~/astar_ws/devel/setup.bash

		Launch the file:

		roslaunch astar_turtlebot3 enviroment.launch x_init:=5 y_init:=3 theta_init:=0 x_final:=9 y_final:=9 rpm1:=15 rpm2:=10
		roslaunch astar_turtlebot3 enviroment.launch x_init:=6 y_init:=8 theta_init:=0 x_final:=9 y_final:=9 rpm1:=15 rpm2:=10
		roslaunch astar_turtlebot3 enviroment.launch x_init:=8 y_init:=5 theta_init:=0 x_final:=7 y_final:=7 rpm1:=15 rpm2:=10


Function modules used: numpy, cv2, scipy, matplotlib.pyplot, sys, math

Github Repo: https://github.com/jpittma1/ENPM661-Project3-Astar.git

Video (Phase 2 part2) GDrive link: https://drive.google.com/file/d/1vQHhmdbhlDbMmHER62JuFUi4YsTiEPOO/view?usp=sharing

