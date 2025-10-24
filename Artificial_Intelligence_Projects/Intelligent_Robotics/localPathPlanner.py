import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random
import math
import json
import time
from Obstacle import *



def read_true_map(fname):
    """Read the ground truth map and output the pose of the ArUco markers and 5 target fruits&vegs to search for

    @param fname: filename of the map
    @return:
        1) list of targets, e.g. ['lemon', 'tomato', 'garlic']
        2) locations of the targets, [[x1, y1], ..... [xn, yn]]
        3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
    """
    
    with open(fname, 'r') as fd:
        file_content = fd.read().strip()  # Read and remove any leading/trailing spaces

        # Check if the file is empty
        if not file_content:
            raise ValueError(f"The file {fname} is empty or contains invalid content.")

        # Replace single quotes with double quotes
        file_content = file_content.replace("'", '"')

        # Load the content as JSON
        gt_dict = json.loads(file_content)

    return gt_dict


def read_search_list():
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open('M4_prac_shopping_list.txt', 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    return search_list


# Euclidean distance heuristic function
def euclidean_distance(node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

# Check if a point is inside a circular obstacle
def is_in_obstacle(node, obstacles):
    x, y = node
    for ox, oy in obstacles:
        if euclidean_distance((x, y), (ox, oy)) < 0.18:
            return True
    return False

# Check if an edge intersects with an obstacle
def edge_intersects_obstacle(node1, node2, obstacles):
    for ox, oy in obstacles:
        # Find closest point on the edge to the obstacle center
        edge_vec = (node2[0] - node1[0], node2[1] - node1[1])
        obs_vec = (ox - node1[0], oy - node1[1])
        t = max(0, min(1, (obs_vec[0] * edge_vec[0] + obs_vec[1] * edge_vec[1]) / (edge_vec[0]**2 + edge_vec[1]**2)))
        closest_point = (node1[0] + t * edge_vec[0], node1[1] + t * edge_vec[1])
        if euclidean_distance(closest_point, (ox, oy)) < 0.18:
            return True
    return False

import math

def move_closer_to_origin(x, y, distance):

  # Calculate the distance from the origin
  distance_to_origin = math.sqrt(x**2 + y**2)

  # Check if the point is already closer than the desired distance
  if distance_to_origin <= distance:
    return x, y

  # Calculate the unit vector pointing from the origin to the point
  unit_vector_x = x / distance_to_origin
  unit_vector_y = y / distance_to_origin

  # Move the point closer to the origin
  new_x = x - distance * unit_vector_x
  new_y = y - distance * unit_vector_y

  return new_x, new_y

def move_sideways_to_origin(x, y, distance):
  # Calculate the distance from the origin
  distance_to_origin = math.sqrt(x**2 + y**2)

  # If the point is at the origin, no sideways move is possible
  if distance_to_origin == 0:
    return x, y

  # Calculate the unit vector pointing from the origin to the point
  unit_vector_x = x / distance_to_origin
  unit_vector_y = y / distance_to_origin

  # Rotate the unit vector by 90 degrees to move sideways
  perpendicular_vector_x = -unit_vector_y
  perpendicular_vector_y = unit_vector_x

  # Move the point sideways by the given distance
  new_x = x + distance * perpendicular_vector_x
  new_y = y + distance * perpendicular_vector_y

  return new_x, new_y

def random_move(x, y, distance):
    # x = goal[0]
    # y = goal[1]
    targetGoalInArea = False
    while targetGoalInArea == False:
        newRandomX = random.uniform(-0.15, 0.15)
        newRandomY = random.uniform(-0.15, 0.15)
        targetGoal = [x + newRandomX ,  y+ newRandomY]
        
        if(abs(targetGoal[0]) > 1.45 or abs(targetGoal[1]) > 1.45):
            targetGoalInArea = False
        else:
            targetGoalInArea = True
    
    finalgoal = (targetGoal[0], targetGoal[1]) 

    return finalgoal

def plan_local_path_Astar(start,robotAngle, goal, obstacles_given,endTime = 3):
    # Create graph
    G = nx.Graph()

    # Define obstacles as (x, y, radius) ---------------------------------------------
    r = 0.15

    # for coords in obstacles_given.values():
    #     x = coords['x']
    #     y = coords['y']
    #     # print(f"Obstacle: {obstacle}, X: {x}, Y: {y}")
    obstacles = obstacles_given

    # obstacles = [(coords["x"], coords["y"], r) for coords in obstacles_given.values()]
    # obstacles = [(coords[' x '], coords[' y '], r) for coords in obstacles_given.values()]
    # ---------------------------------------------------------------------------------------------
    l = 1 #TODO uncomment this
    # Generate random nodes within the square [-1.5, 1.5] for x and y
    # num_nodes = 300
    # nodes = [(random.uniform(-l, l), random.uniform(-l, l)) for _ in range(num_nodes)]

    # Generate a 15x15 grid of evenly spaced nodes
    across = 20
    
    # x_vals = np.linspace(-l, l, across)
    
    x_vals = np.linspace(0, l, across)
    # y_vals = np.linspace(-l, l, across)
    # nodes = [(x, y) for x in x_vals for y in y_vals]  # Total 225 nodes
    nodes = []
    unrotatedNodes = []
    
    
    # for y in y_vals:
    #     # The base width increases as y increases
    #     max_x = y/1.2  # Width increases linearly as y increases
        
    #     # Generate x values within the triangular bounds at each y level
    #     x_vals = np.linspace(-max_x, max_x, across)
        
    #     # Add the valid (x, y) pairs to the list of nodes
    #     unrotatedNodes.extend([(x, y) for x in x_vals])
        
    for x in x_vals:
        # The base width increases as y increases
        max_y = x*1.5 # Width increases linearly as y increases
        
        # Generate x values within the triangular bounds at each y level
        y_vals = np.linspace(-max_y, max_y, across)
        
        # Add the valid (x, y) pairs to the list of nodes
        unrotatedNodes.extend([(x, y) for y in y_vals])
        

    # for x, y in unrotatedNodes:
    #     # Rotation matrix application
    #     x_rotated = x * np.cos(robotAngle) - y * np.sin(robotAngle)
    #     y_rotated = x * np.sin(robotAngle) + y * np.cos(robotAngle)
        
    #     nodes.append((x_rotated, y_rotated))
    
    for x, y in unrotatedNodes:
        # Rotation matrix application
        x_rotated = x * np.cos(robotAngle) - y * np.sin(robotAngle)
        y_rotated = x * np.sin(robotAngle) + y * np.cos(robotAngle)
        
        # Apply translation after rotation
        x_translated = x_rotated + start[0]
        y_translated = y_rotated + start[1]
        
        # Add the rotated and translated node to the list
        nodes.append((x_translated, y_translated))
    


    
    # to stay away from object move have goal moved closer to origin
    finalgoal = move_closer_to_origin(*(goal), r)
    
    # # if new goal is still next to obstacles move it further to origin
    # if is_in_obstacle(finalgoal, obstacles):
    #     finalgoal = move_closer_to_origin(*(finalgoal), r)
    # # if new goal is still next to obstacles move sideways
    # if is_in_obstacle(finalgoal, obstacles):
    #     finalgoal = move_sideways_to_origin(*(goal), r)   

    # if new goal is still next to obstacles move it further to origin
    if is_in_obstacle(finalgoal, obstacles):
        finalgoal = random_move(*(goal), r)
        
    # boarder = 1.45
    boarder = 1.4
    if finalgoal[0] > boarder:
        finalgoal = (boarder,finalgoal[1])
    elif finalgoal[1] > boarder:
        finalgoal = (finalgoal[0], boarder)
    elif finalgoal[0] < -boarder:
        finalgoal = (-boarder,finalgoal[1])
    elif finalgoal[1] < -boarder:
        finalgoal = (finalgoal[0], -boarder)
    
    # (1.4696727089131132, -1.4819185103899406) 
    
    nodes.extend([start, finalgoal]) #add start can goal to nodes
    # print("below are nodes")
    # print(nodes)

    # Filter nodes that are inside obstacles
    nodes = [node for node in nodes if not is_in_obstacle(node, obstacles)]

    for node in nodes:
        G.add_node(node)
    # how many neighbours
    num_neigh = 100
    # Connect each node to its (#no. num_neigh) nearest neighbors if edges do not intersect with obstacles
    for node in nodes:
        distances = {other_node: euclidean_distance(node, other_node) for other_node in nodes if other_node != node}
        nearest_neighbors = sorted(distances, key=distances.get)[:num_neigh]
        
        
        minDis = 999
        
        for i in obstacles:
            prepDis = math.sqrt((node[0]-i[0])**2+(node[1]-i[1])**2)
            if prepDis < minDis:
                minDis = prepDis
            # print(f"prepDis is {prepDis}")
        # input(f"node is {node}, nearest dist is {minDis}")
        if minDis < 0.3:
            weightFactor = minDis*2
            # print("adding weight")
        # elif minDis > 0.5:
        #     weightFactor = -0.5
        else:
            weightFactor = 0
        # endTime = 5


        for neighbor in nearest_neighbors:
            if not edge_intersects_obstacle(node, neighbor, obstacles):
                G.add_edge(node, neighbor, weight=distances[neighbor]+weightFactor)

    # A* path finding
    pathNotFound = True
    startTime = time.time()
    while pathNotFound:
        
        try:
            path = nx.astar_path(G, start, finalgoal, heuristic=euclidean_distance, weight='weight')
            pathNotFound = False
        except:
            G = nx.Graph()
            # obstacles = [(coords["x"], coords["y"], r) for coords in obstacles_given.values()]
            obstacles = obstacles_given
            nodes = [(x, y) for x in x_vals for y in y_vals] 
            nodes = [node for node in nodes if not is_in_obstacle(node, obstacles)]
            
            finalgoal = random_move(*(goal), r)
            nodes.extend([start, finalgoal])
            
            # print(obstacles)
            # input()
            
            for node in nodes:
                G.add_node(node)
            for node in nodes:
                distances = {other_node: euclidean_distance(node, other_node) for other_node in nodes if other_node != node}
                nearest_neighbors = sorted(distances, key=distances.get)[:num_neigh]
                
                minDis = 999
                
                for i in obstacles:
                    prepDis = math.sqrt((node[0]-i[0])**2+(node[1]-i[1])**2)
                    if prepDis < minDis:
                        minDis = prepDis
                        
                    # print(f"prepDis is {prepDis}")
                # input(f"node is {node}, nearest dist is {minDis}")
                if minDis < 0.3:
                    weightFactor = minDis*2
                # elif minDis > 0.5:
                #     weightFactor = -0.5
                else:
                    weightFactor = 0
                endTime = 3
                for neighbor in nearest_neighbors:
                    if not edge_intersects_obstacle(node, neighbor, obstacles):

                        G.add_edge(node, neighbor, weight=distances[neighbor]+weightFactor)
                    # print(f"struck, time is {startTime - time.time()}")
                    if abs(startTime - time.time()) > endTime:
                        # print(f"struck, time is {startTime - time.time()}")
                        return None, None, None

                        
                #         break
                # break

    # Output the waypoints (path nodes)
    
    generatedPath = []
    for point in path:
        # print(point)
        generatedPath.append([point[0],point[1]])
    print(f"Waypoints from start to goal ({len(generatedPath)}):")
    # print(path)
    print(generatedPath)
# --------------------------------

    # Plotting the graph, obstacles, and path
    pos = {node: node for node in nodes}  # node positions correspond to their coordinates

    
    return generatedPath, pos,G



# --------------------------------

    # Plotting the graph, obstacles, and path
    pos = {node: node for node in nodes}  # node positions correspond to their coordinates

    plt.figure(figsize=(6, 6))
    
    # Draw the nodes and edges
    nx.draw(G, pos, with_labels=False, node_size=20, node_color='blue')

    # Highlight the path
    path_edges = list(zip(path, path[1:]))
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)

    # Plot obstacles as circles
    for (ox, oy, radius) in obstacles:
        obstacle_circle = plt.Circle((ox, oy), radius, color='black', alpha=0.5)
        plt.gca().add_patch(obstacle_circle)

    # Plot start and goal
    plt.scatter(*start, color='green', s=300, label='Start')
    plt.scatter(*goal, color='red', s=300, label='Goal')

    plt.legend()
    plt.xlim(-1.5, 1.5)
    plt.ylim(-1.5, 1.5)
    plt.gca().set_aspect('equal', adjustable='box')

    
    plt.show(block=False)

    
    input("Press return to close the plot...")


def local_plot_single_path_and_obstacles(aStar,pos,path, obstacles, start, goal):
    """
    Plots a path and obstacles on the same graph.

    Parameters:
    - path: A list of coordinates [[x1, y1], [x2, y2], ...] representing the path.
    - obstacles: A list of tuples [(x_center, y_center, radius), ...] representing obstacles as circles.
    """
    # Create a figure and axis
    fig, ax = plt.subplots(figsize=(10,10))
    
    square = plt.Rectangle((-1.5, -1.5), 3, 3, fill=False, color='red', label="3x3 Box at Origin")
    plt.gca().add_patch(square)
    

    # Extract x and y coordinates from the path
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]

    # Plot the path as a line connecting the points
    
    # ax.plot(x_coords, y_coords, color='blue', marker='o', linestyle='-', linewidth=1, label='Path')
    
    nx.draw(aStar, pos, with_labels=False, node_size=1, node_color='blue', alpha=0.5)
    path_edges = list(zip(path, path[1:]))
    
    nx.draw_networkx_edges(aStar, pos, edgelist=path_edges, edge_color='red', width=2)
    
    # nx.draw(aStar, pos, with_labels=True, node_size=5, node_color='skyblue', font_size=10)

    # Draw the edge labels (weights)
    edge_labels = nx.get_edge_attributes(aStar, 'weight')
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=5)

    # Plot each obstacle as a circle
    for i in range(len(obstacles)):
        
        obs = Circle(obstacles[i][0], obstacles[i][1], radius=0.08,name="NONE")
        shape = plt.Circle(obs.center, obs.radius, color='red', fill=False, linewidth=1, label='1')
        ax.add_patch(shape)
        
    startCircle = plt.Circle(start,radius=0.05 ,color='purple', fill=False, linewidth=1, label='1')
    ax.add_patch(startCircle)
    goalCircle = plt.Circle(goal,radius=0.5 ,color='green', fill=False, linewidth=1, label='1')
    ax.add_patch(goalCircle)
    
    # Set the aspect of the plot to 'equal' to ensure shapes aren't distorted
    ax.set_aspect('equal', 'box')

    # Set the plot limits (adjust as needed based on your data)
    ax.set_xlim(-1.8,1.8)
    ax.set_ylim(-1.8,1.8)

    # Add labels and a title
    plt.title('Path and Obstacles')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # Show the plot
    # plt.savefig('PATH_GRAPH/path_plot.png',format = 'png')
    plt.show()
    
    
    

def find_optimal_goal_point(obstacles, startPoint , goal, r, grid_resolution=100):
    """
    Function to find the point that is furthest away from all given obstacles, 
    while restricting the search to a circular region of radius 'r' around the target goal location.
    
    Parameters:
    obstacles (dict): A dictionary of obstacles with coordinates as values.
    goal (tuple): The (x, y) coordinates of the target goal location (can be float).
    r (float): The radius around the goal location to search within.
    grid_resolution (int): The number of points to sample along each axis in the grid.
    
    Returns:
    tuple: The coordinates of the furthest point and the distance to the nearest obstacle.
    """
    # Ensure the goal coordinates are treated as floats
    goal_x, goal_y = float(goal[0]), float(goal[1])
    
    # Extract obstacle coordinates
    # obstacle_coords = np.array(list(obstacles.values()))
    obstacle_coords = np.array([(coords["x"], coords["y"], r) for coords in obstacles.values()])
    # print(obstacle_coords)
    # print(obstacles)


    # Create grid points within a square around the goal
    x_grid = np.linspace(goal_x - r, goal_x + r, grid_resolution)
    y_grid = np.linspace(goal_y - r, goal_y + r, grid_resolution)

    # Initialize the best distance and point
    best_distance = -1
    best_point = None

    # Iterate over the grid
    for x in x_grid:
        for y in y_grid:
            # Only consider points within the circular region of radius 'r' around the goal
            if np.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2) <= r:
                # Calculate the distance to all obstacles
                distances = np.sqrt((obstacle_coords[:, 0] - x) ** 2 + (obstacle_coords[:, 1] - y) ** 2)
                disFromStart = np.sqrt((startPoint[0] - x) ** 2 + (startPoint[1] - y) ** 2)

                borderDis = min(abs(x + 1.4), abs(x - 1.4), abs(y + 1.4), abs(y - 1.4))  # Minimum distance to the rectangle's edge
                # Find the minimum distance to the nearest obstacle

                distances = distances[distances <= 2]                

                min_distance = distances.min() + borderDis*0.8
                
                print(f"point {x},{y} have a are {borderDis} from edge and have min dis of {min_distance} and is {disFromStart} from start")

                # Update the best point if this is the furthest from any obstacle
                if (min_distance > best_distance) and (abs(x)<1.45 and abs(y)<1.45):
                    best_distance = min_distance
                    best_point = (x, y)

    return best_point, best_distance
# -----------------------------------------
# If this script is run directly, provide an example usage
if __name__ == "__main__":
    start_point = (0, 0)
    
    

    r = 0.15
    # goalObject = {"y": -0.665625, "x": -0.9375}

 

    
    obstacles_default = read_true_map('est_maps/estMap1.txt')
    search_list = read_search_list()
    
    goalObject = obstacles_default.get('pear_0')
    # input(test)
    xCoord = goalObject.get('x')
    yCoord = goalObject.get('y')       
    # goal_point = (0.691, -0.0035)    
    goal_point = (0.767, 0.238)    
    
    # optimisedGoal,_ = find_optimal_goal_point(obstacles_default,start_point,goal_point,0.3)
    # goal_point = optimisedGoal
    findingPath = True
    # input(f"optimised goal is {optimisedGoal}")
    robot_angle = 30
    while findingPath:
        path,pos, G = plan_local_path_Astar(start_point,robot_angle, goal_point, obstacles_default)
        if path == None:
            
            newRandomX = random.uniform(-0.1, 0.1)
            newRandomY = random.uniform(-0.1, 0.1)
            
            start_point = (start_point[0] + newRandomX, start_point[1] + newRandomY)
        else:
            findingPath = False
            
    
    from Obstacle import *
    obstaclesList = []
    radius = 0.12
        
    for key,value in obstacles_default.items():

        xCoord = value.get('x')
        yCoord = value.get('y')
        obstaclesList.append([xCoord,yCoord])
        
            
        # obstaclesList.append(Circle(xCoord, yCoord, radius=radius,name="NONE"))
        
    formatPathPlot = []
    for i in path:
        formatPathPlot.append((i[0],i[1]))
            
    
    local_plot_single_path_and_obstacles(G,pos,formatPathPlot,obstaclesList,start_point,goal_point)