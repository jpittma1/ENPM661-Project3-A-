#!/usr/bin/env python3

#ENPM673 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Phase 2
# Functions

import copy
import timeit
import queue
from queue import PriorityQueue
import numpy as np
import cv2
import scipy
from numpy import linalg as LA
import matplotlib.pyplot as plt
import sys
import math
from obstacles import *
from Node import *

def GetInitialStates():
    print("Enter initial node (Xs, Ys, theta_s), separated by spaces: ")
    initial=[int(x) for x in input().split()]
    print("Enter goal node (Xg, Yg), separated by spaces: ")
    final=[int(x) for x in input().split()]
    print("Enter clearance and robot radius Default=(5, 10), separated by spaces: ")
    robot=[int(x) for x in input().split()]
    print("Enter Step size of movement in units (1-10) Default=5: ")
    step_size=int(input())
    print("Enter Theta (angle between movements) Default=30: ")
    theta=int(input())
    
    return initial, final, robot, step_size, theta

###########################################
'''OpenCV/ Visualization Functions'''
#To fix the origin from top left to bottom right
#Transform into pixel coordinates
def pointTransformation(node, chart):
    X, Y, _ = chart.shape
    trans_y = node[0]
    trans_x = X - node[1] -1
    return [int(trans_x), int(trans_y)]


def updateNodesOnMap(map, node_state, color):
    #Draw vector lines connecting parent to child
    if node_state.getParent() is not None:
        parent= node_state.getParent().getState()
        # parent= node_state.getParent()
        child = node_state.getState()
        parent_transformed = pointTransformation(parent, map)
        child_transformed = pointTransformation(child, map)

        #Send in (y,x) for cv2.line
        map = cv2.line(map, (parent_transformed[1], parent_transformed[0]), (child_transformed[1], child_transformed[0]), color, 1)
    else: #No parent => just plot the point
        child = node_state.getState()
        child_transformed = pointTransformation(child, map)
        map[child_transformed[0], child_transformed[1], :] = color
    
    return map

'''Function from Project 2 (Dijkstra for plotting)'''
def plotPointOnMap(map, node_state, color):
    x,y, _ = map.shape
    trans_y = node_state[0]  
    trans_x = x - node_state[1] - 1
    map[trans_x,trans_y, :] = color
    
    return map

# '''To plot a circle equal to goal threshold'''
# def plotGoalThresholdOnMap(map, goal, thresh,color):
#     x,y, _ = map.shape
#     trans_y = goal[0]  
#     trans_x = x - goal[1] - 1
    
#     cv2.circle(map,(trans_x, trans_y),thresh, color, 2) 
    
#     return map

#Equation of line for Hexagon and Boomerang
def lineEquation(p1,p2,x,y):
    func = ((p2[1] - p1[1]) * (x - p1[0])) / ( p2[0] - p1[0]) + p1[1] - y
    
    return func

def addObstacles2Map(map):
    #########---------PLOT Circle----------------#########
    for i in range(circle_offset_x - circle_radius, circle_offset_x + circle_radius):
        for j in range(circle_offset_y - circle_radius, circle_offset_y + circle_radius):
            if (i - circle_offset_x) **2 + (j - circle_offset_y)**2 <= circle_radius**2:
                plotPointOnMap(map, [i, j], [0,255,255])
    

    for i in range(map.shape[1]):
       for j in range(map.shape[0]):
            #-----HEXAGON--------------------------
            if (i<hexagon_right_x and i>hexagon_left_x and lineEquation((hexagon_left_x,hexagon_upper_y),(hexagon_top_x,hexagon_top_y),i,j) > 0 and lineEquation((hexagon_top_x,hexagon_top_y),(hexagon_right_x,hexagon_upper_y),i,j) > 0 and lineEquation((hexagon_left_x,hexagon_lower_y),(hexagon_bottom_x,hexagon_bottom_y),i,j) < 0 and lineEquation((hexagon_bottom_x,hexagon_bottom_y),(hexagon_right_x,hexagon_lower_y),i,j) < 0):
                plotPointOnMap(map, [i, j], [0,255,255])
            
            #----Top Triangle of Boomerang--------
            if(lineEquation((left_x,left_y),(triangle_top_x,triangle_top_y),i,j) >0 and lineEquation((triangle_top_x,triangle_top_y),(right_x, right_y),i,j) <0 and lineEquation((left_x,left_y),(right_x, right_y),i,j) <0):
                plotPointOnMap(map, [i, j], [0,255,255])

            #----Bottom Triangle of Boomerang---------
            if (lineEquation((left_x,left_y),(right_x, right_y),i,j) >0 and lineEquation((right_x, right_y),(triangle_bottom_x,triangle_bottom_y),i,j) <0 and lineEquation((triangle_bottom_x,triangle_bottom_y),(left_x,left_y),i,j) <0):
                plotPointOnMap(map, [i, j], [0,255,255])
            
            
    return map

############################################################
'''Return 1 if within an obstacle or outside of map'''
def isInObstacleSpace(x,y):
    x_max=sizex-1   #399
    y_max=sizey-1   #249
   
    #-------Check if within Map-----------
    if (int(x) > x_max or int(x)<0 or int(y)<0 or int(y)>y_max):
        return 1
    
    #-----Check if within circle-------
    in_circle=(x-circle_offset_x)**2+(y-circle_offset_y)**2
    if in_circle <= (circle_radius)**2:
        return 1
    
    '''PointPolygonTest: positive (inside), negative (outside), or zero (on an edge) value,
    In the function, the third argument is measureDist. If it is True, it finds the
    shortest distance between a point in the image and a contour. If False, it finds
    whether the point is inside, outside, or on the contour. Since we don't want to
    find the distance, we set the measureDist argument to False'''
    
    #----Check if within Hexagon------
    in_hexagon=cv2.pointPolygonTest(hexagon_pts, (x,y), False)
    if in_hexagon>0:
        return 1
  
    #----Check if within boomerang--------
    in_boomerang_top=cv2.pointPolygonTest(boomerang_pts_top, (x,y), False)
    if in_boomerang_top>0:
        return 1
    
    in_boomerang_bottom=cv2.pointPolygonTest(boomerang_pts_bottom, (x,y), False)
    if in_boomerang_bottom>0:
        return 1
    
    return 0

'''Returns poss_moves which becomes X_prime consisting of array of Nodes
Updates state (post_move)
parent(current_node)
move(moves options)
cost_to_come (step_size)'''
def possibleMoves(current_node, step_size, theta):
    # i = int(current_node[0])
    # j=  int(current_node[1])
    moves=["maxPort","port", "straight", "starboard", "maxStarboard"]
    node_state=current_node.getState()
    actions=[] #list of Node objects to save "moves", parent, and cost
    actions.append(Node(ActionMoveMaxPort(node_state, step_size, theta),current_node, moves[0], current_node.getCost() + step_size))
    actions.append(Node(ActionMovePort(node_state, step_size, theta),current_node, moves[0], current_node.getCost() + step_size))
    actions.append(Node(ActionMoveStraight(node_state, step_size),current_node, moves[0], current_node.getCost() + step_size))
    actions.append(Node(ActionMoveStarboard(node_state, step_size, theta),current_node, moves[0], current_node.getCost() + step_size))
    actions.append(Node(ActionMoveMaxStarboard(node_state, step_size, theta),current_node, moves[0], current_node.getCost() + step_size))
    

    #remove None nodes (None means is in obstacleSpace)
    poss_moves = [action for action in actions if action.getState() is not None]
    # print("current_node [0]", current_node[0])
    
    
    return poss_moves

'''Compute Heuristic Euclidean Distance for Cost 2 goal'''
def heuristicEuclidean(now, goal):
    cost=0.0
    
    if now is not None:
        cost=np.sqrt((now[0]-goal[0])**2 + (now[1]-goal[1])**2)
    return cost

'''-----5 Subfunctions for actions-----
Default theta_step is 30
Action sets= Port 60, Port 30, Straight, Starboard 30, Starboard 60'''

def ActionMovePort(initial, step_size, angle):
    
    current_theta = initial[2]
    new_theta = current_theta + angle
    if new_theta >= 360:
        new_theta = new_theta - 360

    dx = step_size * np.cos(np.radians(new_theta))
    dy = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + dx, initial[1] + dy, new_theta]

    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

def ActionMoveMaxPort(initial, step_size, angle):

    current_theta = initial[2]
    new_theta = current_theta + 2*angle
    if new_theta >= 360:
        new_theta = new_theta - 360

    dx = step_size * np.cos(np.radians(new_theta))
    dy = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + dx, initial[1] + dy, new_theta]

    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

def ActionMoveStraight(initial, step_size):

    current_theta = initial[2]
    new_theta = current_theta

    dx = step_size * np.cos(np.radians(new_theta))
    dy = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + dx, initial[1] + dy, new_theta]

    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

def ActionMoveStarboard(initial, step_size, angle):
  
    current_theta = initial[2]
    new_theta = current_theta - angle
    if new_theta <= -360 :
        new_theta = new_theta + 360

    dx = step_size * np.cos(np.radians(new_theta))
    dy = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + dx, initial[1] + dy, new_theta]

    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

def ActionMoveMaxStarboard(initial, step_size, angle):
    
    current_theta = initial[2]
    new_theta = current_theta - 2*angle
    if new_theta <= -360:
        new_theta = new_theta + 360
    
    dx = step_size * np.cos(np.radians(new_theta))
    dy = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + dx, initial[1] + dy, new_theta]
    
    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

'''Check for Goal Node Function'''
def compare2Goal(now,goal, goal_thresh):
    
    distance_sq=np.square(now[0] - goal[0]) + np.square(now[1] - goal[1])
    
    if distance_sq < goal_thresh**2:
    # if np.array_equal(now, goal) or now==goal:
        return True
    else:
        return False
