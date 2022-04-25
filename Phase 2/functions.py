#!/usr/bin/env python3

#ENPM673 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Phase 2
# Functions

import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Visualization:
    def __init__(self, obstacle):
        self.radius = obstacle.robot_radius
        self.obstacle = obstacle

    def getRadius(self):
        return self.radius

    def updateMapViz(self, space_map, state, color):
        X, Y, _ = space_map.shape
        transformed_y = state[0]
        transformed_x = X - state[1] -1
        space_map[transformed_x, transformed_y, :] = color

    def addObstacles2Map(self, ax):
        circle1_centre = (self.obstacle.circle1_x_offset, self.obstacle.circle1_y_offset)
        circle1_radius = self.obstacle.circle1_radius +  self.obstacle.clearance

        circle2_centre = (self.obstacle.circle2_x_offset, self.obstacle.circle2_y_offset)
        circle2_radius = self.obstacle.circle2_radius +  self.obstacle.clearance

        square1_corner1 = (self.obstacle.square1_corner1_x - self.obstacle.clearance, self.obstacle.square1_corner1_y - self.obstacle.clearance)
        square1_side = self.obstacle.square_side + (2 * self.obstacle.clearance)

        rect1_corner1 = (self.obstacle.rect1_corner1_x - self.obstacle.clearance, self.obstacle.rect1_corner1_y - self.obstacle.clearance)
        rect1_length = self.obstacle.rect1_length + (2 * self.obstacle.clearance)
        rect1_width = self.obstacle.rect1_width + (2 * self.obstacle.clearance)

        rect2_corner1 = (self.obstacle.rect2_corner1_x - self.obstacle.clearance, self.obstacle.rect2_corner1_y - self.obstacle.clearance)
        rect2_length = self.obstacle.rect2_length + (2 * self.obstacle.clearance)
        rect2_width = self.obstacle.rect2_width + (2 * self.obstacle.clearance)


        circle1 = patches.Circle(circle1_centre, radius=circle1_radius, linewidth=1, edgecolor='r', facecolor='r')
        circle2 = patches.Circle(circle2_centre, radius=circle2_radius, linewidth=1, edgecolor='r', facecolor='r')
        square1 = patches.Rectangle(square1_corner1, square1_side, square1_side, linewidth=1, edgecolor='r', facecolor='r')
        rect1 = patches.Rectangle(rect1_corner1, rect1_length, rect1_width, linewidth=1, edgecolor='r', facecolor='r')
        rect2 = patches.Rectangle(rect2_corner1, rect2_length, rect2_width, linewidth=1, edgecolor='r', facecolor='r')

        ax.add_patch(square1)
        ax.add_patch(rect1)
        ax.add_patch(rect2)
        ax.add_patch(circle1)
        ax.add_patch(circle2)

        return ax
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