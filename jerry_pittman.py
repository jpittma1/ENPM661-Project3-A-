#!/usr/bin/env python3

#ENPM661 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Phase 1


from Node import *
from functions import *
from obstacles import *


'''--User input for initial and goal State--'''
Xi,Xg, rigid_body, step_size, theta=GetInitialStates()
clearance = rigid_body[0]       #overrides default (10) in obstacles.py
robot_radius = rigid_body[1]    #overrides default (5) in obstacles.py

####--for testing without user input--####
# Xi = [0,0,0] #starts at Origin
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
    print("Goal state is in an obstacle or off the map, please provide new valid initial state")
    exit()

theta_s = Xi[2]

#--Goal Thresholds---
goal_threshold=1.5*robot_radius
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

#Node Object: self, state, parent, move, cost; give each node cost of infinity

ClosedList=np.array([[Node([i,j],None, None, math.inf) for j in range(250)] for i in range(400)])

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
videoname=('project3-jerry-pittman')
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video = cv2.VideoWriter(str(videoname)+".avi",  fourcc, 300, (map_x, map_y))

space = np.zeros([map_size[0], map_size[1], 3], dtype=np.uint8) 
# print("space shape", space.shape)
space = updateNodesOnMap(space, Xi, [0,0,255])
space = updateNodesOnMap(space, Xg, [0,0,255])
space = addObstacles2Map(space)

cv2.imwrite('Initial_map.jpg', space)
print("Initial map created named 'Initial_map.jpg' ")

cv2.imshow('Initial_map', space)

########----Determine if Duplicate Node Matrix "V"----###########
'''For finding duplicate nodes, 
Initially set all to zero (0)
Take node visited, round, then set the corresponding V[i][j][k]=1'''
V = np.array([[[math.inf  for k in range(360/theta)] for j in range(int(map_y/threshold))] for i in range(int(map_x/threshold))])


###########A*-star Algorithm While Loop#############
'''Conduct A*-star algorithm to find path between 
initial and goal node avoiding obstacles'''

start = timeit.default_timer()
print("Commencing A-star Search.......")

while not (OpenList.empty() and reachedGoal):
    
    curr_node = OpenList.get()[1]
    i, j = curr_node.getState()
    # current_node = get_min_node(queue)
    # current_point = [current_node.x,current_node.y]
    # orientation = current_node.orientation
    # visitedNodes[int(current_node.x*2)][int((current_node.y)*2)][int(orientation%30)]=1
    # print("current node (x,y) is: (", i, ", ",j,")")
    
    space = updateNodesOnMap(space, curr_node.getState(), [0, 255, 0])
    video.write(space)
    
    
    reachedGoal =compare2Goal(curr_node.getState(),Xg, goal_threshold)

    if reachedGoal:
        # path = []
        print("Goal Reached!!")
        print("Total cost of path is ", curr_node.getCost())

        moves_path, path = curr_node.getFullPath()

        # print("Backtracked Moves is ", moves_path)
        # print("Backtracked Node path is ", path)
        
        for node in path:  #Make white Node pathway on map
                pos = node.getState()
                space = updateNodesOnMap(space, pos, [255, 255, 255]) #White
                cv2.imshow('Map',space)
                video.write(space)
        
        #Video ends abruptly at goal, want to have goal shown for a little longer
        for i in range(250):       
            video.write(space)
        break
    
    else:
        X_prime=possibleMoves(curr_node, step_size,theta)
        # print("possible directions of current node are: ", X_prime)
        parent_cost=curr_node.getCost()    #current cost to come
        
        ########
        #TODO: Cost to come ==Step_size
        ################
        '''Iterate through all compass point directions'''
        for move in X_prime:
            
            
            # if move is not visited:
            #  if visitedNodes[int(new_round(new_node.x)*2)][int(new_round(new_node.y)*2)][int(orientation%30)]== 0:
                #####TODO: Visited Nodes?
                # visitedNodes[int(current_node.x*2)][int((current_node.y)*2)][int(orientation%30)]=1
                # V[int(halfRound(branch_state[0])/threshold), int(halfRound(branch_state[1])/threshold), int(halfRound(branch_state[2])/30)] = branch_node.getCost() + computeHeuristicCost(branch_state, goal_state)
                
                ########TODO: A-star######
                #update Parent
                #C2C
                #Total Cost
                
                #else:
                #if total cost > C2C+cost+Cost2Goal
                    #C2c new=C2C previous + cost
                    #total cost=c2C_new+C2G_new
                    #update parent
                
                
                child_pos = [move[0], move[1]]
                cost_to_come = parent_cost + moves_cost.get(move)
                total_cost = cost_to_come + heuristicEuclidean()
                
                
                
                #Verify not visited based on cost2come set to infinity
                if (ClosedList[child_pos[0], child_pos[1]].getCost() == math.inf):
                    child_Node = Node(child_pos, curr_node, move, cost_to_come)
                    ClosedList[child_pos[0], child_pos[1]] = child_Node  #update parent
                    OpenList.put((child_Node.getCost(), child_Node))
                else:
                    #Check if cost is larger than c2c+local_cost
                    if (cost_to_come < ClosedList[child_pos[0], child_pos[1]].getCost()): 
                        child_Node = Node(child_pos, curr_node, move, cost_to_come)
                        ClosedList[child_pos[0], child_pos[1]] = child_Node
                        OpenList.put((child_Node.getCost(), child_Node))
    
                
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
