from enum import Enum
from queue import PriorityQueue
import numpy as np
import utm
import matplotlib
matplotlib.use('wxAgg')
import matplotlib.pyplot as plt
from bresenham import bresenham


def create_grid(data, drone_altitude, safety_distance):#, resolution):
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
    north_size = int(np.ceil((north_max - north_min)))#/resolution))
    east_size = int(np.ceil((east_max - east_min)))#/resolution))

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

    # add diagonal motions with a cost of sqrt(2) to your A* implementation
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

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

    # add diagonal motions with a cost of sqrt(2) to your A* implementation
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def a_star(grid, h, start, goal):
    print('A* Algorithm start...')
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
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



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position),1)


def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def bresenham_check(grid,line):
    cells = list(bresenham(line[0], line[1], line[2], line[3]))
    for c in cells:
        #print(grid[c[0]][c[1]])
        if grid[c[0]][c[1]]:
            return False
    return True
        

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def prune_path_collinearity(path):
    print('collinearity')
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def prune_path_bresenham(grid, path):
    print('bresenham')
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p3 = pruned_path[i+2]
        line = [p1[0],p1[1],p3[0],p3[1]]
        if bresenham_check(grid, line):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def global_to_local(global_position, global_home):
    (easting_h, northing_h, _, _) = utm.from_latlon(global_home[1], global_home[0])
    (easting_p, northing_p, _, _) = utm.from_latlon(global_position[1], global_position[0])
    
    lat = (easting_p-easting_h)
    long = (northing_p-northing_h)
    alt = -(global_position[2]-global_home[2])
    
    return (long,lat,alt)


def local_to_global(local_position, global_home):
    (east_h, north_h, zone_number_h, zone_letter_h) = utm.from_latlon(global_home[1], global_home[0])
    (lat, long) = utm.to_latlon(east_h+local_position[1],
                                north_h+local_position[0], 
                                zone_number_h, zone_letter_h)
    alt = global_home[2]-local_position[2];
    
    return (long,lat,alt)
    

def plot_path(grid,start_ne,goal_ne,path):
    plt.figure()
    plt.imshow(grid, origin='lower')
        
    plt.plot(start_ne[1], start_ne[0], 'ro')
    plt.plot(goal_ne[1], goal_ne[0], 'rx')
    
    if not path==[]:
        pp = np.array(path)
        plt.plot(pp[:, 1], pp[:, 0], 'g')
    
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()#block=False)
    
    
def find_nearest_feasible(skel, point):
    if skel[point[0],point[1]]:
        print('Find nearest feasible goal')
        skel_cells = np.transpose(np.nonzero(skel==0))
        point_min_dist = np.linalg.norm(np.array(point) - np.array(skel_cells), axis=1).argmin()
        point = skel_cells[point_min_dist]
    return (int(point[0]),int(point[1]))


def adjust_waypoint_headings(waypoints):
    for i in range(1,len(waypoints)-1):
        wp1 = waypoints[i-1]
        wp2 = waypoints[i]
        angle = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
        waypoints[i][3] = angle
    return waypoints

def LocationGlobal_to_LonLatAlt(loc):
    return [loc.lon,loc.lat,loc.alt]

def LocationLocal_to_NorthEastUp(loc):
    return [loc.north,loc.east,-loc.down]