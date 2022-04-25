#!/usr/bin/env python3

#ENPM661 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Phase 2

from obstacles import *
from functions import *
import numpy as np
import cv2
from Node import *
import matplotlib.pyplot as plt


import math
import queue

import csv

wheel_radius = 0.038
wheel_distance = 0.354


def halfRound(n):
    val = round(2*n)/2
    if (val == 10):
      val -= 0.5
    return val


def toRadian(angle):
    return np.pi * angle / 180
def toDegree(angle):
    return 180 * angle / np.pi


def checkGoalReached(current_node, goal_state, thresh_radius):
    current_state = current_node.getState()
    radius_sq = np.square(current_state[0] - goal_state[0]) + np.square(current_state[1] - goal_state[1])
    if radius_sq < thresh_radius**2:
        return True
    else:
        return False


def computeHeuristic(current_state, goal_state):
    cost = 0.0
    if current_state is not None:
        cost =  ((current_state[0]-goal_state[0])**2 + (current_state[1]-goal_state[1])**2)**(0.5)
    return cost

def checkVisited(node, node_array, goal_state, threshold=0.5):
    result = False
    node_state = node.getState()
    x = node_state[0]
    y = node_state[1]
    theta = node_state[2]
    x = int(halfRound(x)/threshold)
    y = int(halfRound(y)/threshold)

    if (node.getCost() + computeHeuristic(node_state, goal_state) < node_array[x, y, theta]):
        result = True
    return result

def getBranches(node, T, w1, w2, obs):
    actions=[[w1, w1], [w2, w2], [w1, 0], [0, w1], [w1, w2], [w2, w1], [0, w2], [w2, 0]]
    state = node.getState()
    branches = []

    for action in actions:
        new_state, path_array, cost = move(state, action, T, obs)
        if new_state is not None:
            branch_node = Node(new_state, node, action, node.getCost() + cost, path_array)
            branches.append(branch_node)
      
    return branches

def move(state, action, T, obs):
    t = 0
    dt = 0.1
    
    Xi, Yi, thetai = state
    thetai = toRadian(thetai)

    wL, wR = action

    Xn = Xi
    Yn = Yi
    thetan = thetai

    path_array = []
    cost = 0.0
    path_array.append([Xn, Yn])
    while t<T:
        t = t + dt
        dx = 0.5 * wheel_radius * (wL + wR) * math.cos(thetan) * dt
        dy = 0.5 * wheel_radius * (wL + wR) * math.sin(thetan) * dt
        Xn += dx
        Yn += dy
        thetan += (wheel_radius / wheel_distance) * (wL - wR) * dt
        cost += math.sqrt(math.pow(dx,2) + math.pow(dy,2))
        path_array.append([Xn, Yn])
        
        if obs.isInObstacleSpace(Xn, Yn):
            return None, None, None

    thetan = int(toDegree(thetan))
    if (thetan >= 360):
        thetan-=360
    if (thetan <= -360):
        thetan+=360
    return [Xn, Yn, thetan] , path_array, cost

def visualize(viz, traversed_nodes, node_path):
    fig, ax = plt.subplots(figsize = (10, 10))
    ax.set(xlim=(0, 10), ylim = (0,10))
    ax = viz.addObstacles2Map(ax)
    ax.set_aspect("equal")
    for node in traversed_nodes:
        xi, yi, _ = node.getState()
        points = node.getPathArray()
        if points is not None:
            for point in points:
                xn, yn = point
                ax.plot([xi, xn], [yi, yn], color="blue", linewidth = 0.5 )
                xi, yi = xn, yn
            plt.pause(0.000001)


    for node in node_path:
        xi, yi, _ = node.getState()
        points = node.getPathArray()
        if points is not None:
            for point in points:
                xn, yn = point
                ax.plot([xi, xn], [yi, yn], color="red", linewidth = 0.5 )
                xi, yi = xn, yn  
            plt.pause(0.0001)

    plt.show()
    plt.close()

def astar():

    h,w = 10,10
    threshold = 0.5
    start_point = [5,3,0]
    goal_state = [9,9]
    w1, w2 = 5, 10
    nodes = queue.PriorityQueue()
    init_node = Node(start_point, None, None, 0, None)
    nodes.put((init_node.getCost(), init_node))
    traversed_nodes = []

    obs = Obstacle(0.0)
    viz = Visualization(obs)

    fig, ax = plt.subplots(figsize = (10, 10))
    ax.set(xlim=(0, 10), ylim = (0,10))
    ax = viz.addObstacles2Map(ax)
    ax.set_aspect("equal")

    goal_reached = False
    node_array = np.array([[[math.inf for k in range(360)] for j in range(int(h/threshold))] for i in range(int(w/threshold))])

    full_path = None
    goal_reached = False
    print('Searching for optimal path.....')



    while (not nodes.empty()):
        current_node = nodes.get()[1]
        traversed_nodes.append(current_node)

        if checkGoalReached(current_node, goal_state,1):
            print('Goal reached')
            print("The cost of path: ", current_node.getCost())
            moves, node_path = current_node.getFullPath()

            visualize(viz, traversed_nodes, node_path)

            goal_reached = True

            fp = open('path_points.csv', 'w')
            fn = open('path_nodes.csv', 'w')
            fv = open('vel_points.csv', 'w')
            writer_p = csv.writer(fp)
            writer_n = csv.writer(fn)
            writer_v = csv.writer(fv)

            for move in moves:
                writer_v.writerow(move)

            for node in node_path:
                xi, yi, _ = node.getState()
                writer_n.writerow([xi, yi])

                points = node.getPathArray()
                if points is not None:
                    for point in points:
                        xn, yn = point
                        row = [xn, yn]
                        writer_p.writerow(row)
                        xi, yi = xn, yn        
            fp.close()
            fv.close()
            fn.close()
        

        else:
            branches = getBranches(current_node, 1, w1, w2, obs)
            for branch_node in branches:
                branch_state = branch_node.getState()
                if checkVisited(branch_node, node_array, goal_state, threshold=0.5):
                    node_array[int(halfRound(branch_state[0])/threshold), int(halfRound(branch_state[1])/threshold), branch_state[2]] = branch_node.getCost() + computeHeuristic(branch_state, goal_state)
                    nodes.put((branch_node.getCost() + computeHeuristic(branch_state, goal_state), branch_node))
        if (goal_reached): break


        
if __name__ == "__main__":
    astar()
