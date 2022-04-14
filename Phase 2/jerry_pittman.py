#!/usr/bin/env python3

#ENPM661 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Phase 2

from Node import *
from functions import *
from obstacles import *


'''--User input for initial and goal State--'''
Xi,Xg, rigid_body, step_size, theta=GetInitialStates()
clearance = rigid_body[0]       #overrides default (10) in obstacles.py
robot_radius = rigid_body[1]    #overrides default (5) in obstacles.py

####--for testing without user input--####
# Xi = [0,0,0] #starts at Origin
# Xg=[380, 20] #above hexagon
# Xg=[20, 20] #left hexagon
# Xg=[200, 20] #below hexagon
# Xg=[20, 100] #left hexagon
# Xg=[390,240] #behind circle
# step_size = 5
# theta = 30
###########################################

# # Xg=[380, 20] #above hexagon
# # Xg=[20, 20] #above hexagon
# Xg=[200, 200] #above hexagon
# # Xg=[390,240] #behind circle
# step_size = 5
# theta = 30
################################


print("Initial State is ", Xi) #(x, y, theta_s)
print("Goal state is: ", Xg)

#######CHECK IF ENTERED VALUES ARE VALID###########
if isInObstacleSpace(Xi[0],Xi[1]):
    print("Initial state is in an obstacle or off the map, please provide new valid initial state")
    exit()
    
if isInObstacleSpace(Xg[0],Xg[1]):
    print("Goal state is in an obstacle or off the map, please provide new valid goal state")
    exit()

theta_s = Xi[2]

#--Goal Thresholds---
goal_threshold=int(1.5*robot_radius)
threshold=0.5

####################INITIALIZE NODEs AND MAP###############
'''Initialize tuple and store in OpenList priority queue.
       [0,1]=coordinate values (x,y) from user input
       [2]=index of node; initially set to 0
       [3]=parent node index; initially set to -1
       [4]= cost to come; initially set to 0
       [5]=total cost'''

#-------Class object and Priority Queue Initialization--------
OpenList = PriorityQueue()  
start_node = Node(Xi, None, None, 0)
OpenList.put((start_node.getCost(), start_node))

reachedGoal = False

# action_set=straight, CCW60, CCW30, CW30, CW60
#x, y, theta

print("Initial and Goal points are valid...Generating map...")
'''**Visualization Code**
    Map Background=Black
    Start=Red
    Goal= Red
    Obstacles=Yellow
    Completed Nodes=Green
    Path=white
'''
map_size = [250, 400] 
map_y, map_x = map_size
videoname=('project3-phase2-jerry-pittman')
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video = cv2.VideoWriter(str(videoname)+".avi",  fourcc, 300, (map_x, map_y))

space = np.zeros([map_size[0], map_size[1], 3], dtype=np.uint8) 
# print("space shape", space.shape)
'''Plot start and goal Nodes on Map--Red'''
space = updateNodesOnMap(space, start_node, [0,0,255])
space = plotPointOnMap(space, Xi, [0,0,255])
space = plotPointOnMap(space, Xg, [0,0,255])

# space = plotGoalThresholdOnMap(space,Xg,goal_threshold,[0,0,255])   #Plot goal threshold circle
# # cv2.circle(space,(Xg[0],Xg[1]),goal_threshold, (0,0,255), 2) 

'''Plot Obstacles on Map--Yellow'''
space = addObstacles2Map(space)

cv2.imwrite('Initial_map.jpg', space)
print("Initial map created named 'initial_map.jpg' ")

cv2.imshow('Initial_map', space)

########----Determine if Duplicate Node Matrix "V"----###########
'''For finding duplicate nodes, 
Initially set all to zero (0)
Take node visited, round, then set the corresponding V[i][j][k]=1'''
VisitedNodes = np.array([[[0 for k in range(int(360/theta))] for j in range(int(map_y/threshold))] for i in range(int(map_x/threshold))])

#Make initial/Starting Node as visited
VisitedNodes[int(Xi[0]*2)][int(Xi[1]*2)][int(theta_s%30)]=1
###########A*-star Algorithm While Loop#############
'''Conduct A*-star algorithm to find path between 
initial and goal node avoiding obstacles'''

start = timeit.default_timer()
print("Commencing A-star Search.......")

while not (OpenList.empty()):
# while not (OpenList.empty() and reachedGoal):
    
    curr_node = OpenList.get()[1]
    i, j, direction = curr_node.getState()
    # print("curr_node is: ",curr_node)
    # print("current node (x,y) is: (", i, ", ",j,")")
    # print("Orientation is: ", direction)
   
    space = updateNodesOnMap(space, curr_node, [0, 255, 0])
    # space = updateNodesOnMap(space, curr_node.getState(), [0, 255, 0])
    video.write(space)
    
    reachedGoal =compare2Goal(curr_node.getState(),Xg, goal_threshold)

    if reachedGoal:
        print("Goal Reached!!")
        print("Total cost of path is ", curr_node.getCost())
        # print("Total cost of path is ", curr_node.getCostTotal())

        '''Backtracking Function'''
        moves_path, path = curr_node.getFullPath()

        # print("Backtracked Moves is ", moves_path)
        # print("Backtracked Node path is ", path)
        
        for node in path:  #Make white Node pathway on map
                # pos = node.getState()

                space = updateNodesOnMap(space, node, [255, 255, 255]) #White
                cv2.imshow('Map',space)
                video.write(space)
        
        #Video ends abruptly at goal, want to have goal shown for a little longer
        for i in range(300):       
            video.write(space)

    
    else:
        X_prime=possibleMoves(curr_node, step_size,theta) #array of new Node Class Objects
        # print("possible directions of current node are: ", X_prime)
        # parent_cost=curr_node.getCost()    #current cost to come
        # cost_to_come = parent_cost + step_size  #Cost2Come increase by step_size
        
        '''Iterate through all Possible Actions/Directions'''
        for move in X_prime:
            # print("move is: ", move.getMove())
            move_state=move.getState()
            '''Verify not Visited; if so, update'''
            if VisitedNodes[int(round(move_state[0])*2)][int(round(move_state[1])*2)][int(move_state[2]%30)]== 0:
                VisitedNodes[int(round(move_state[0])*2)][int(round(move_state[1])*2)][int(move_state[2]%30)]== 1 #Mark as visited
                
                '''Q.insert (x'):
                self.move possibilities=["maxPort","port", "straight", "starboard", "maxStarboard"]
                cost_to_come "self.cost" updated in possibleMoves function as step_size'''
                #self.move possibilities=["maxPort","port", "straight", "starboard", "maxStarboard"]
                total_cost = move.getCost()+ heuristicEuclidean(move_state,Xg)
                OpenList.put((total_cost, move))    #Update OpenList Queue
                
            else: #Node has been visited
                '''Cost(x') > CostToCome(x) + l(x,u)+CostToGo(x')'''
                rightHandSide=curr_node.getCost() + step_size + heuristicEuclidean(move,Xg)
                if move.getCost() > rightHandSide:
                # if move.getCost() > (curr_node.getcost() + step_size + heuristicEuclidean(move,Xg)):
                    
                    '''Update Cost to come'''
                    cost_to_come = curr_node.getCost() + step_size
                    
                    '''Update Total Cost'''
                    move.cost = cost_to_come+heuristicEuclidean(move_state,Xg)
                    
                    '''Update Parent'''
                    move.parent = curr_node
      
    if reachedGoal: break
    
stop = timeit.default_timer()
print("That algorithm took ", stop-start, " seconds")


cv2.namedWindow("map", cv2.WINDOW_NORMAL)
cv2.imshow('Final_ map', space)
cv2.imwrite('Final_map.jpg', space)
print("Final map created named 'final_map.jpg' ")

if cv2.waitKey(1) == ord('q'):
    video.release()

video.release()
cv2.destroyAllWindows()
