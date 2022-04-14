#!/usr/bin/env python3

#ENPM673 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Phase 1
# Node Class Object

import numpy as np
import math

'''Class of Node to store state, parent, move made, and cost of each move'''
class Node():
    def __init__(self, state, parent, move, cost):
        # creating tuple with cost to come, index, parent node index=0 and coordinate values (x,y) 
        self.state = state      #index
        self.parent = parent    #parent of current node
        self.move = move
        self.cost = cost   #cost to come; default will be set to math.inf

        
    #---Accessors---
    def getState(self):
        return self.state
		
    def getParent(self):
        return self.parent
    
    def getMove(self):
	    return self.move
		
    def getCost(self):
        return self.cost
    
    #Override less than operator
    def __lt__(self, other):
        return self.cost < other.cost
    
    #Override greater than operator
    def __gt__(self, other):
            return self.cost < other.cost

    '''Backtracking Function'''
    def getFullPath(self):
        moves = []
        nodes = []
        current_node = self
        
        while(current_node.getMove() is not None):

            moves.append(current_node.getMove())
            nodes.append(current_node)
            current_node = current_node.getParent()

        nodes.append(current_node)
        moves.reverse()
        nodes.reverse()
        
        return moves, nodes
