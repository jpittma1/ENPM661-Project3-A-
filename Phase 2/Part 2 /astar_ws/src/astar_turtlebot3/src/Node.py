#!/usr/bin/env python3

#ENPM661 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Phase 2

import numpy as np

class Node():
    def __init__(self, state, parent, move, cost, path_array): 

        self.state = state
        self.parent = parent
        self.move = move
        self.cost = cost
        self.pathArray = path_array
        # self.parent_state = parent_state
        
    def getState(self):
        return self.state

    def getPathArray(self):
        return self.pathArray
		
    def getParent(self):
        return self.parent

    def getParentState(self):
        if self.getParent() is None:
            return None
        return self.getParent().getState()
		
    def getMove(self):
	    return self.move
		
    def getCost(self):
        return self.cost

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

    def printStats(self):
        pass
    def __lt__(self,other):
        return self.getState() < other.getState()