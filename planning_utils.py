from enum import Enum
from queue import PriorityQueue
import heapdict
import queue
import numpy as np
import os
import sys
from math import *
from decimal import Decimal


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)
# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    return valid_actions


def a_star(grid, h, start, goal):
    print('-------------------------------------------------')
    print("a_star")
    print('-------------------------------------------------')
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {} #dictionary 
    found = False
    
    while not queue.empty():
        # print(list(queue.queue))
        item = queue.get()
        current_node = item[1]
        # print('Current node', current_node)
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node): #returns North, west, south,east
                # get the tuple representation
                da = action.delta #return direction of the action 
                next_node = (current_node[0] + da[0], current_node[1] + da[1]) #updates new_node based on the previous location+new lcoation
                # print('Next node:     ', next_node)
                branch_cost = current_cost + action.cost # updates branch cost = current cost + action.cost
                # print('Branch cost:   ', branch_cost)
                queue_cost = branch_cost + h(next_node, goal) #updates queue cost based on the brancj cost and manhatan distance between current node and goal
                # print('Queue Cost:   ', queue_cost)
                if next_node not in visited:                
                    visited.add(next_node)        #add Neighbor nodes to visited set        
                    branch[next_node] = (branch_cost, current_node, action) #input in dictionarty {key:next_node, value: branch_cost(total cost of th path), current_node(predecessor), action(W,S,N,E)}
                    queue.put((queue_cost, next_node)) #puts (queue_cost, next_node) inside the queue, puts node inside the queue
            # print('Next Iteration--------------------------------------------------------------------')   
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0] #cost
        path.append(goal) 
        while branch[n][1] != start: #while current node != start
            path.append(branch[n][1]) 
            n = branch[n][1]
        path.append(branch[n][1])
        arr = path[::-1]
        for i in arr:
            print(i[0],' ',i[1],'')
            
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def dfs(grid, h, start, goal):
    print('-------------------------------------------------')
    print("dfs")
    print('-------------------------------------------------')
    path = [] #Need to use LIFO Queue
    path_cost = 0
    stack = []
    stack.append((0, start))
    visited = set(start)
    
    depth = h(start, goal)**2 #in case correct path is outside of the box
    # print('depth', depth)
    branch = {}
    found = False
    
    while stack:
        # print(stack) 
        item = stack.pop()
        
        current_node = item[1]
        
        # print('Current Node is', current_node)
        
        # print('-------------------------------------------------')
        if current_node == start:
                current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
        if current_node == goal:
            print('Found a path!')
            found = True
            break
        elif item[0]<=depth:
            for action in valid_actions(grid, current_node):
                da = action.delta 
                next_node = (current_node[0]+da[0],current_node[1]+da[1])
                branch_cost = current_cost+action.cost
                if next_node not in visited:
                    if branch_cost<depth:
                        visited.add(next_node)
                        branch[next_node] = (branch_cost,current_node,action)
                        stack.append((branch_cost,next_node))
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0] #cost
        path.append(goal) 
        while branch[n][1] != start: #while current node != start
            path.append(branch[n][1]) 
            n = branch[n][1]
        path.append(branch[n][1])
        arr = path[::-1]
        for i in arr:
            print(i[0],' ',i[1],'')
            
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost
           
def iterative_astar(grid, h, start, goal):
    print('-------------------------------------------------')
    print("iterative_a_star")
    print('-------------------------------------------------')
    threshold = h(start,goal)
    
    path = []
    path.append(start)
    branch = {}
    
    found = False
    while True :
        
        temp = search(path,0.0,h,grid,threshold,goal,branch)
        
        if temp < 0:
            print('This is the way')
            found = True
            
            
            
            break
        if temp == float('inf'):
            print('**********************')
            print('Wasted')
            print('**********************')
            break
        
        threshold = temp
        print('threshold', threshold)
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0] #cost
        print(path_cost)
        for i in path:
            print(i[0],' ',i[1],'')
            
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
        
    return path[::-1], path_cost
    
    
def search(path,g,h,grid, threshold,goal,branch):
    
    current_node = path[-1] # might be problem
   
    f = g + h(current_node,goal)
    
    if f>threshold:
        
        return f
    if current_node == goal:
        return -g
    
    min = sys.maxsize * 2 + 1
    
    for action in valid_actions(grid, current_node):
        da = action.delta 
        next_node = (current_node[0]+da[0],current_node[1]+da[1])
        if next_node not in path:
            path.append(next_node) #works
            branch_cost = g+action.cost
        
            branch[next_node] = (branch_cost,current_node,action)
            
            temp = search(path,branch_cost,h,grid,threshold,goal,branch) # problem here
            if temp < 0:
                return temp
            if(temp<min):
                min = temp
            path.pop()
            
            branch.pop(next_node)
    
         
    return min  

def ucs(grid, h, start, goal):
    print('-------------------------------------------------')
    print("ucs")
    print('-------------------------------------------------')
    path = []
    path_cost = 0
    heapDict = heapdict.heapdict()
    heapDict[start] = 0
    visited = set(start)
    branch = {}
    found = False
    
    while heapDict:
        item = heapDict.popitem()  
        current_node = item[0]
        if current_node == start:
            current_cost = 0.0
        else: 
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path!')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    heapDict[next_node] = branch_cost
                # if finds less pricy path
                elif next_node in heapDict.keys() and branch_cost < heapDict[next_node]:
                    heapDict[next_node] = branch_cost
                    branch[next_node] = (branch_cost, current_node, action)
                
                    

             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('********')
        print('Failed to find a path!')
        print('********') 
    return path[::-1], path_cost          


def a_star_Q6(grid, h, start, goal,route):
    print('-------------------------------------------------')
    print("a_star_Q6")
    print('-------------------------------------------------')
    path = []
    path_cost = 0
    begin = start
    
    
    while len(route):
        min = h(begin, route[0])
        idx = 0
        for p in route:
            temp_h = h(begin, p)
            if min>temp_h:
                min = h(begin, p)
                idx = route.index(p)
        temp_goal = route[idx]
        print(route[idx])
        partOfPath, partialCost = a_star(grid,heuristic,begin,route[idx])
        path += partOfPath
        path_cost+=partialCost
        begin = route[idx]
        route.pop(idx)
    lastPath,lastCost = a_star(grid,heuristic,begin,goal)
    path+=lastPath
    path_cost+=lastCost
    
    return path, path_cost

    
    
             
def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))
#sum of absolute difference
def manhattanDistance(position, goal_position):
    return abs(position[0]-goal_position[0])+abs(position[1]-goal_position[1])

