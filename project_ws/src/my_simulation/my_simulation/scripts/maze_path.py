import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import heapq
import os

# This script contains functions that find the shortes path from start to goal in a given maze
def parse_sdf_wall_positions():
    '''Gets wall positions and properties from a .sdf file'''
    sdf_file = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'worlds', 'maze_gen.world')
    tree = ET.parse(sdf_file)
    root = tree.getroot()

    # Define the namespaces
    namespaces = {'sdf': 'http://www.gazebosim.org/sdf'}

    # Find all wall links
    walls = []
    for link in root.findall('.//link', namespaces):
        model_name = link.get('name')
        if "Wall" in model_name:
            # Extract the wall's visual and collision information
            collision = link.find('.//collision', namespaces)
           
            # Get position and orientation
            pose = link.find('pose', namespaces).text
            geometry = collision.find('geometry/box/size', namespaces).text
           
            wall_data = {
                'name': model_name,
                'pose': [float(i) for i in pose.split()],
                'size': [float(i) for i in geometry.split()]
            }
            walls.append(wall_data)

    return walls

def create_2d_grid(grid_resolution=0.01, padding=0.06):
    '''Creates a 2d grid of a given maze and adds padding'''
    walls = parse_sdf_wall_positions()
    room_hor = []
    room_ver = []
    walls_hor = []
    walls_ver = []
    for wall in walls:
        if wall['name'] == 'Wall_2' or wall['name'] == 'Wall_4':
            room_hor.append(wall)
        elif wall['name'] == 'Wall_1' or wall['name'] == 'Wall_3':
            room_ver.append(wall)
        elif wall['pose'][-1] == 0.0:
            walls_ver.append(wall)
        else:
            walls_hor.append(wall)

    # Determine the bounds of the maze
    min_x = min(wall['pose'][0] - wall['size'][1]/2 for wall in room_hor)
    max_x = max(wall['pose'][0] + wall['size'][1]/2 for wall in room_hor)
    min_y = min(wall['pose'][1] - wall['size'][1]/2 for wall in room_ver)
    max_y = max(wall['pose'][1] + wall['size'][1]/2 for wall in room_ver)

    # Define the size of the grid
    grid_width = int((max_x - min_x) / grid_resolution)
    grid_height = int((max_y - min_y) / grid_resolution)

    # Initialize the grid (0: free space, 1+: wall)
    grid = np.zeros((grid_width, grid_height), dtype=int)

    # Map walls of the grid
    for wall in walls_hor:
        # Calculate the grid coordinates of the wall
        wall_min_x = int((wall['pose'][0] - wall['size'][1]/2 - min_x) / grid_resolution)
        wall_max_x = int((wall['pose'][0] + wall['size'][1]/2 + max_x) / grid_resolution)
        wall_min_y = int((wall['pose'][1] - wall['size'][0]/2 - min_y) / grid_resolution)
        wall_max_y = int((wall['pose'][1] + wall['size'][0]/2 + max_y) / grid_resolution)

        # Mark the grid cells occupied by the wall
        grid[wall_min_x:wall_max_x, wall_min_y:wall_max_y] = 2

        # Add padding
        padding_min_x = int((wall['pose'][0] - wall['size'][1]/2 - min_x - padding) / grid_resolution)
        padding_max_x = int((wall['pose'][0] + wall['size'][1]/2 + max_x + padding) / grid_resolution)
        padding_min_y = int((wall['pose'][1] - wall['size'][0]/2 - min_y - padding) / grid_resolution)
        padding_max_y = int((wall['pose'][1] + wall['size'][0]/2 + max_y + padding) / grid_resolution)
        
        grid[padding_min_x:wall_min_x, padding_min_y:padding_max_y] = np.where(
                grid[padding_min_x:wall_min_x, padding_min_y:padding_max_y] == 0,
                1,
                grid[padding_min_x:wall_min_x, padding_min_y:padding_max_y]
            )

        grid[wall_max_x:padding_max_x, padding_min_y:padding_max_y] = np.where(
                grid[wall_max_x:padding_max_x, padding_min_y:padding_max_y] == 0,
                1,
                grid[wall_max_x:padding_max_x, padding_min_y:padding_max_y]
            )

        grid[padding_min_x:padding_max_x, padding_min_y:wall_min_y] = np.where(
                grid[padding_min_x:padding_max_x, padding_min_y:wall_min_y] == 0,
                1,
                grid[padding_min_x:padding_max_x, padding_min_y:wall_min_y]
            )

        grid[padding_min_x:padding_max_x, wall_max_y:padding_max_y] = np.where(
                grid[padding_min_x:padding_max_x, wall_max_y:padding_max_y] == 0,
                1,
                grid[padding_min_x:padding_max_x, wall_max_y:padding_max_y]
            )


    for wall in walls_ver:
        # Calculate the grid coordinates of the wall
        wall_min_x = int((wall['pose'][0] - wall['size'][0]/2 - min_x) / grid_resolution)
        wall_max_x = int((wall['pose'][0] + wall['size'][0]/2 + max_x) / grid_resolution)
        wall_min_y = int((wall['pose'][1] - wall['size'][1]/2 - min_y) / grid_resolution)
        wall_max_y = int((wall['pose'][1] + wall['size'][1]/2 + max_y) / grid_resolution)

        # Mark the grid cells occupied by the wall
        grid[wall_min_x:wall_max_x, wall_min_y:wall_max_y] = 2

        # Add padding
        padding_min_x = int((wall['pose'][0] - wall['size'][0]/2 - min_x - padding) / grid_resolution)
        padding_max_x = int((wall['pose'][0] + wall['size'][0]/2 + max_x + padding) / grid_resolution)
        padding_min_y = int((wall['pose'][1] - wall['size'][1]/2 - min_y - padding) / grid_resolution)
        padding_max_y = int((wall['pose'][1] + wall['size'][1]/2 + max_y + padding) / grid_resolution)

        grid[padding_min_x:wall_min_x, padding_min_y:padding_max_y] = np.where(
                grid[padding_min_x:wall_min_x, padding_min_y:padding_max_y] == 0,
                1,
                grid[padding_min_x:wall_min_x, padding_min_y:padding_max_y]
            )

        grid[wall_max_x:padding_max_x, padding_min_y:padding_max_y] = np.where(
                grid[wall_max_x:padding_max_x, padding_min_y:padding_max_y] == 0,
                1,
                grid[wall_max_x:padding_max_x, padding_min_y:padding_max_y]
            )

        grid[padding_min_x:padding_max_x, padding_min_y:wall_min_y] = np.where(
                grid[padding_min_x:padding_max_x, padding_min_y:wall_min_y] == 0,
                1,
                grid[padding_min_x:padding_max_x, padding_min_y:wall_min_y]
            )

        grid[padding_min_x:padding_max_x, wall_max_y:padding_max_y] = np.where(
                grid[padding_min_x:padding_max_x, wall_max_y:padding_max_y] == 0,
                1,
                grid[padding_min_x:padding_max_x, wall_max_y:padding_max_y]
            )

    for wall in room_hor:
        # Calculate the grid coordinates of the wall
        wall_min_x = int((wall['pose'][0] - wall['size'][1]/2 - min_x) / grid_resolution)
        wall_max_x = int((wall['pose'][0] + wall['size'][1]/2 + max_x) / grid_resolution)
        wall_min_y = int((wall['pose'][1] - wall['size'][0]/2 - min_y) / grid_resolution)
        wall_max_y = int((wall['pose'][1] + wall['size'][0]/2 + max_y) / grid_resolution)

        # Mark the grid cells occupied by the wall
        grid[wall_min_x:wall_max_x, wall_min_y:wall_max_y] = 2

        # Add padding
        if '2' in wall['name']:
            room_padding = int((wall['pose'][0] - wall['size'][1]/2 - min_x - padding) / grid_resolution)
            grid[room_padding:wall_min_x, wall_min_y:wall_max_y] = np.where(
                grid[room_padding:wall_min_x, wall_min_y:wall_max_y] == 0,
                1,
                grid[room_padding:wall_min_x, wall_min_y:wall_max_y]
            )
        elif '4' in wall['name']:
            room_padding = int((wall['pose'][0] + wall['size'][1]/2 + max_x + padding) / grid_resolution)
            grid[wall_max_x:room_padding, wall_min_y:wall_max_y] = np.where(
                grid[wall_max_x:room_padding, wall_min_y:wall_max_y] == 0,
                1,
                grid[wall_max_x:room_padding, wall_min_y:wall_max_y]
            )

    for wall in room_ver:
        # Calculate the grid coordinates of the wall
        wall_min_x = int((wall['pose'][0] - wall['size'][0]/2 - min_x) / grid_resolution)
        wall_max_x = int((wall['pose'][0] + wall['size'][0]/2 + max_x) / grid_resolution)
        wall_min_y = int((wall['pose'][1] - wall['size'][1]/2 - min_y) / grid_resolution)
        wall_max_y = int((wall['pose'][1] + wall['size'][1]/2 + max_y) / grid_resolution)

        # Mark the grid cells occupied by the wall
        grid[wall_min_x:wall_max_x, wall_min_y:wall_max_y] = 2

        # Add padding
        if '1' in wall['name']:
            room_padding = int((wall['pose'][1] - wall['size'][1]/2 - min_y - padding) / grid_resolution)
            grid[wall_min_x:wall_max_x, room_padding:wall_min_y] = np.where(
                grid[wall_min_x:wall_max_x, room_padding:wall_min_y] == 0,
                1,
                grid[wall_min_x:wall_max_x, room_padding:wall_min_y]
            )
        elif '3' in wall['name']:
            room_padding = int((wall['pose'][1] + wall['size'][1]/2 + max_y + padding) / grid_resolution)
            grid[wall_min_x:wall_max_x, wall_max_y:room_padding] = np.where(
                grid[wall_min_x:wall_max_x, wall_max_y:room_padding] == 0,
                1,
                grid[wall_min_x:wall_max_x, wall_max_y:room_padding]
            )

    ## Visualize
    #plt.imshow(grid.T, origin='lower', cmap='Greys', interpolation='none')
    #plt.show()

    return grid

def heuristic(point, goal):
    '''Calculates the manhattan distance between two points'''
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

def astar(start, goal):
    '''Finds the shortes path between the start and goal points'''
    grid = create_2d_grid()
    open_set = []

    # Priority queue with (F-score, node)
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            # Reconstruct the path and return
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()

            # Visualize
            for p in path:
                grid[p] = 5
            plt.imshow(grid.T, origin='lower', cmap='Greys', interpolation='none')
            plt.show()

            return path
        
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            x, y = current[0] + dx, current[1] + dy
            neighbor = (x, y)

            # Assuming uniform cost for each step
            tentative_g = g_score[current] + 1
            
            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0:
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = current
    
    return None # No path found