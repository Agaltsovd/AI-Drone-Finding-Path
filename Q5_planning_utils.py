from enum import Enum
from queue import PriorityQueue
import numpy as np
import queue
import heapdict
from scipy.spatial import Voronoi
from bresenham import bresenham as bh
from udacidrone.frame_utils import global_to_local
import numpy.linalg as LA
import networkx as nx
import traceback as tb

def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    # minimum and maximum north coordinates

    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    #given the minimum and maximum coordinates we can calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    # Define a list to hold Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
            int(north - d_north - safety_distance - north_min_center),
            int(north + d_north + safety_distance - north_min_center),
            int(east - d_east - safety_distance - east_min_center),
            int(east + d_east + safety_distance - east_min_center),
            ]
            grid[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3]] = 1
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])
        
    # TODO: create a voronoi graph based on
    # location of obstacle centres
    
    try:
        graph = Voronoi(points)
        print(graph)
    except Exception as e:
        print(e)
        print('\nError Displaying \n')
    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for edge in graph.ridge_vertices:
        point1 = graph.vertices[edge[0]]
        point2 = graph.vertices[edge[1]]
        cells = list(bh(int(point1[0]), int(point1[1]), int(point2[0]), int(point2[1])))
        infeasible = False
        for cell in cells:
            if np.amin(cell) < 0 or cell[0] >= grid.shape[0] or cell[1] >=grid.shape[1]:
                infeasible = True
                break
            if grid[cell[0], cell[1]] == 1:
                infeasible = True
                break
        if infeasible == False:
            point1 = (point1[0], point1[1])
            point2 = (point2[0], point2[1])
            edges.append((point1,point2))
    try:
        print('\nEdge is currently {}\n'.format(edge))
    except Exception as e:
        print(e)
    
    return grid, edges, int(north_min), int(east_min)


def bfs(graph, start, goal):
    

    path = []
    path_cost = 0
    q = queue.Queue()
    q.put(start)
    visited = set(start)

    branch = {}
    found = False
    
    while not q.empty():
        item = q.get()
        if item == start:
            current_cost = 0.0
        else:              
            current_cost = branch[item][0]
            
        if item == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph.neighbors(item):
                # get the tuple representation
                branch_cost = current_cost + 1
                
                if next_node not in visited:                
                    visited.add(next_node)              
                    branch[next_node] = (branch_cost, item)
                    q.put(next_node)
             
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
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost
