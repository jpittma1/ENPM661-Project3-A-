#!/usr/bin/env python3

#ENPM673 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Functions

import copy
import timeit
import queue
from queue import PriorityQueue
import numpy as np
import cv2
import scipy
from scipy import fft, ifft
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
def updateNodesOnMap(map, node_state, color):
    x,y, _ = map.shape
    trans_y = node_state[0]  
    trans_x = x - node_state[1] - 1
    map[trans_x,trans_y, :] = color
    
    return map

#Equation of line for Hexagon and Boomerang
def lineEquation(p1,p2,x,y):
    func = ((p2[1] - p1[1]) * (x - p1[0])) / ( p2[0] - p1[0]) + p1[1] - y
    
    return func

def addObstacles2Map(map):
    
    #########---------PLOT Circle----------------#########
    for i in range(circle_offset_x - circle_radius, circle_offset_x + circle_radius):
        for j in range(circle_offset_y - circle_radius, circle_offset_y + circle_radius):
            if (i - circle_offset_x) **2 + (j - circle_offset_y)**2 <= circle_radius**2:
                updateNodesOnMap(map, [i, j], [0,255,255])
    

    for i in range(map.shape[1]):
       for j in range(map.shape[0]):
            #-----HEXAGON--------------------------
            if (i<hexagon_right_x and i>hexagon_left_x and lineEquation((hexagon_left_x,hexagon_upper_y),(hexagon_top_x,hexagon_top_y),i,j) > 0 and lineEquation((hexagon_top_x,hexagon_top_y),(hexagon_right_x,hexagon_upper_y),i,j) > 0 and lineEquation((hexagon_left_x,hexagon_lower_y),(hexagon_bottom_x,hexagon_bottom_y),i,j) < 0 and lineEquation((hexagon_bottom_x,hexagon_bottom_y),(hexagon_right_x,hexagon_lower_y),i,j) < 0):
                updateNodesOnMap(map, [i, j], [0,255,255])
            
            #----Top Triangle of Boomerang--------
            if(lineEquation((left_x,left_y),(triangle_top_x,triangle_top_y),i,j) >0 and lineEquation((triangle_top_x,triangle_top_y),(right_x, right_y),i,j) <0 and lineEquation((left_x,left_y),(right_x, right_y),i,j) <0):
                updateNodesOnMap(map, [i, j], [0,255,255])

            #----Bottom Triangle of Boomerang---------
            if(lineEquation((left_x,left_y),(triangle_bottom_x,triangle_bottom_y),i,j) <0 and lineEquation((triangle_bottom_x,triangle_bottom_y),(right_x, right_y),i,j) >0 and lineEquation((right_x, right_y),(left_x,left_y),i,j) >0):
                updateNodesOnMap(map, [i, j], [0,255,255])
            
            
    return map

############################################################
'''Return 1 if within an obstacle or outside of map'''
def isInObstacleSpace(x,y):
    x_max=400-1
    y_max=250-1
    '''positive (inside), negative (outside), or zero (on an edge) value,
    In the function, the third argument is measureDist. If it is True, it finds the
    shortest distance between a point in the image and a contour. If False, it finds
    whether the point is inside, outside, or on the contour. Since we don't want to
    find the distance, we set the measureDist argument to False'''
    
    #Check if within Map
    if (x > x_max or int(x)<0 or int(y)<0 or int(y)>y_max):
        return 1
    
    #Check if within circle
    in_circle=(x-circle_offset_x)**2+(y-circle_offset_y)**2
    if in_circle <= (circle_radius)**2:
        return 1
    
    #check if within Hexagon
    in_hexagon=cv2.pointPolygonTest(hexagon_pts, (x,y), False)
    if in_hexagon>0:
        return 1
  
    #check if within boomerang
    
    in_boomerang_top=cv2.pointPolygonTest(boomerang_pts_top, (x,y), False)
    if in_boomerang_top>0:
        return 1
    
    in_boomerang_bottom=cv2.pointPolygonTest(boomerang_pts_bottom, (x,y), False)
    if in_boomerang_bottom>0:
        return 1
    
    return 0

def possibleMoves(current_node, step_size, theta):
    # i = int(current_node[0])
    # j=  int(current_node[1])
    node_state=current_node.getState()
    actions=[]
    actions.append(ActionMoveP60(node_state, step_size, theta))
    actions.append(ActionMoveP30(node_state, step_size, theta))
    actions.append(ActionMoveStraight(node_state, step_size))
    actions.append(ActionMoveS30(node_state, step_size, theta))
    actions.append(ActionMoveS60(node_state, step_size, theta))
    

    #remove None nodes (None means is in obstacleSpace)
    poss_moves = [a for a in actions if a is not None]
    # print("current_node [0]", current_node[0])
    
    #Verify not visited by comparing to parent---
    for move in range(len(poss_moves)):
        if current_node.getParentState() == move:
            poss_moves.remove(move)
    
    
    # moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
    # poss_moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
    # move_i = [i, i+1, i+1, i+1, i, i-1, i-1, i-1]
    # move_j = [j+1, j+1, j, j-1, j-1, j-1, j, j+1]
    # for move in range(len(moves)):
    #     if (isInObstacleSpace(move_i[move], move_j[move]) or current_node.getParentState() == [move_i[move], move_j[move]]):
    #         poss_moves.remove(moves[move])
    # print(final_moves)
    
    return poss_moves

'''Compute Heuristic Euclidean Distance for Cost 2 goal'''
def heuristicEuclidean(now, goal):
    cost=0.0
    
    if now is not None:
        cost=np.sqrt((now[0]-goal[0])**2 + (now[1]-goal[1])**2)
    return cost

'''-----5 Subfunctions for actions-----
Action sets= Port 60, Port 30, Straight, Starboard 30, Starboard 60'''

def ActionMoveP30(initial, step_size, angle):
    
    current_theta = initial[2]
    new_theta = current_theta + angle
    if new_theta >= 360:
        new_theta = new_theta - 360

    step_x = step_size * np.cos(np.radians(new_theta))
    step_y = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + step_x, initial[1] + step_y, new_theta]

    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

def ActionMoveP60(initial, step_size, angle):

    current_theta = initial[2]
    new_theta = current_theta + 2*angle
    if new_theta >= 360:
        new_theta = new_theta - 360

    step_x = step_size * np.cos(np.radians(new_theta))
    step_y = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + step_x, initial[1] + step_y, new_theta]

    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

def ActionMoveStraight(initial, step_size):

    current_theta = initial[2]
    new_theta = current_theta

    step_x = step_size * np.cos(np.radians(new_theta))
    step_y = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + step_x, initial[1] + step_y, new_theta]

    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

def ActionMoveS30(initial, step_size, angle): #assuming we cat land on the borders
  
    current_theta = initial[2]
    new_theta = current_theta - angle
    if new_theta <= -360 :
        new_theta = new_theta + 360

    step_x = step_size * np.cos(np.radians(new_theta))
    step_y = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + step_x, initial[1] + step_y, new_theta]

    if isInObstacleSpace(NewNode[0], NewNode[1]):
        return None
 
    return NewNode

def ActionMoveS60(initial, step_size, angle):
    
    current_theta = initial[2]
    new_theta = current_theta - 2*angle
    if new_theta <= -360:
        new_theta = new_theta + 360
    
    step_x = step_size * np.cos(np.radians(new_theta))
    step_y = step_size * np.sin(np.radians(new_theta))
    
    NewNode = [initial[0] + step_x, initial[1] + step_y, new_theta]
    if isInObstacleSpace(initial, NewNode):
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


# '''backtracking function'''
# def generate_path(start, end, pathTaken):
#     global Parent_Node_Index_i
#     global Node_Index_i
    
#     temp_path = []
#     temp_path.append(end)
    
#     # pathTaken[j][0] is child Node
#     # pathTaken[j][1] is parent Node
    
#     c=2 #Next Node Index is 2
#     for i in range(len(pathTaken)):
#         Node_Index_i.append(c)      #increment child index
        
#         for j in range(len(pathTaken)):

#             #compare last node of temp_path to child of pathway
#             if temp_path[i] == pathTaken[j][0]: #path vs child
#                 temp_path.append(pathTaken[j][1]) #add parent to path

#                 previousParent=Parent_Node_Index_i[-1] #last parent node index value
#                 # print("Previous parent ", previousParent)

#                 if pathTaken[j][1]!=pathTaken[j-1][1]:  #increment parent index if new parent
#                     Parent_Node_Index_i.append(previousParent+1)
#                 else:
#                     Parent_Node_Index_i.append(previousParent)
#                 # print("temp_path[i] is ", temp_path[i])
#                 break
            
#         if temp_path[i]==start: #determine if parent is start
#             break
#         c+=1
    
#     # print("pre-reversed_path is ", temp_path)
#     # print("Parent index is ", Parent_Node_Index_i)
#     # print("Node index is ", Node_Index_i)
    
#     path=[]
#     #reverse path so goes start to goal
#     for i in reversed(temp_path):
#         path.append(i)
    
#     # print("reversed path is ", path)
#     # print("length of path is ", len(path))
    
#     return path
    
