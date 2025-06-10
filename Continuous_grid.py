import random
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
import math

class Grid:
    def __init__(self, side, obstacles, targets, number_agents, agent_half_size, agent_radius, obstacle_half_size, target_half_size, seed):
        #Checks if a seed has been given
        self.seed = seed
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)      

        self.width = side
        self.height = side    

        self.number_agents = number_agents #Setter in function
        self.agent_half_size = agent_half_size
        self.agent_radius = agent_radius
        self.agents= []
        self.grid = self.make_grid()
        self.safe_space = [[5, int(side/2), 5]]
        self.set_grid(self.safe_space[0][0], self.safe_space[0][1], self.safe_space[0][2], self.safe_space[0][2], 4)
        self.initialize_agents()

        #Check if you need to randomize the obstacles 
        if type(obstacles) == int:

            self.nr_of_obstacles = obstacles
            self.obstacles = []
            self.obstacle_half_size = obstacle_half_size

            self.initialize_obstacles()
        
        #Set predefined obstacles
        else:
            self.obstacles = obstacles
            self.set_obstacles()

        # Check if you need to randomize the targets
        if type(targets)  == int:
            self.nr_of_targets = targets
            self.targets = []
            self.target_half_size = target_half_size 
            self.initialize_targets()
        
        #Set predefined targets
        else:
            self.targets = targets
            self.set_targets()  

    # create a integer matrix representation of the grid (self.height x self.width)
    def make_grid(self):
        return np.full((self.height, self.width), 0, dtype=int)

    def set_grid(self, x, y, half_width, half_height, value):
        x_min = max(0, math.ceil((x - half_width)))
        x_max = min(self.width, math.ceil((x + half_width)))
        y_min = max(0, math.ceil((y - half_height)))
        y_max = min(self.height, math.ceil((y + half_height)))
        
        self.grid[y_min:y_max, x_min:x_max] = value

    # randomly place a number of obstacles in the grid
        # Randomly place obstacles in the grid
    def initialize_obstacles(self):
        for i in range(self.nr_of_obstacles):
            placed = False
            while not placed:
                half_width = int(random.uniform(self.obstacle_half_size[0], self.obstacle_half_size[1]))
                half_height = int(random.uniform(self.obstacle_half_size[0], self.obstacle_half_size[1]))
                
                x = random.uniform(half_width, self.width - half_width)
                y = random.uniform(half_height, self.height - half_height)
                
                # Check if this position overlaps with existing obstacles
                if (not self.check_overlap(x, y, half_width, half_height, self.obstacles) and 
                    not self.check_overlap(x, y, half_width, half_height, self.agents) and
                    not self.check_overlap(x, y, half_width, half_height, self.safe_space)):

                    self.obstacles.append([x, y, half_width, half_height])
                    
                    # Update visualization grid
                    self.set_grid(x, y, half_width, half_height, 1)
                    placed = True
                    
                    # Add adjacent obstacles with some probability
                    self.place_adjacent_obstacles(x, y, half_width, half_height)
    
    def set_obstacles(self):
        for x, y, width, height in self.obstacles:
            self.set_grid(x, y, width, height, 1)        

    def place_adjacent_obstacles(self, x, y, half_width_prev, half_height_prev):
        if random.random() < 0.5:
            placed = False
            attempts = 0
            while not placed and attempts < 10:  # Limit number of attempts
                attempts += 1
                
                half_width = random.uniform(self.obstacle_half_size[0], self.obstacle_half_size[1])
                half_height = random.uniform(self.obstacle_half_size[0], self.obstacle_half_size[1])
                
                if random.random() < 0.5:
                    offset_x = half_width + half_width_prev
                    offset_y = random.uniform(-(half_height_prev+half_height), half_height_prev+half_height)
                else:
                    offset_x = random.uniform(-(half_width_prev+half_width), half_width_prev+half_width)
                    offset_y = half_height + half_height_prev
                
                offset_x = offset_x if random.random() < 0.5 else -offset_x
                offset_y = offset_x if random.random() < 0.5 else -offset_y

                new_x = x + offset_x
                new_y = y + offset_y
                
                # Check if position is valid
                if (half_width <= new_x <= self.width - half_width and 
                    half_height <= new_y <= self.height - half_height and 
                    not self.check_overlap(new_x, new_y, half_width, half_height, self.agents) and
                    not self.check_overlap(new_x, new_y, half_width, half_height, self.obstacles) and
                    not self.check_overlap(new_x, new_y, half_width, half_height, self.safe_space)):
                    
                    self.obstacles.append([new_x, new_y, half_width, half_height])
                    
                    # Update visualization grid
                    self.set_grid(new_x, new_y, half_width, half_height, 1)
                    placed = True
                    
                    # Recursive call to potentially place more adjacent obstacles
                    self.place_adjacent_obstacles(new_x, new_y, half_width, half_height)
    
    def set_targets(self):
        for x, y, half_size in self.targets:
            self.set_grid(x, y, half_size, half_size, 3)

    def initialize_targets(self):
        for i in range(self.nr_of_targets):
            placed = False
            while not placed:
                x = random.uniform(self.target_half_size, self.width - self.target_half_size)
                y = random.uniform(self.target_half_size, self.height - self.target_half_size)
                
                # Check if this position is clear of obstacles
                if (not self.check_overlap(x, y, self.target_half_size, self.target_half_size, self.obstacles) and 
                    not self.check_overlap(x, y, self.target_half_size, self.target_half_size, self.agents) and 
                    not self.check_overlap(x, y, self.target_half_size, self.target_half_size, self.targets) and
                    not self.check_overlap(x, y, self.target_half_size, self.target_half_size, self.safe_space)):

                    # Store target as [x, y, radius]
                    self.targets.append([x, y, self.target_half_size])
                    
                    # Update visualization grid
                    self.set_grid(x, y, self.target_half_size, self.target_half_size, 3)
                    placed = True

    def initialize_agents(self): 
        for i in range(self.number_agents):
            x = self.agent_radius + math.floor(i / 3) * self.agent_half_size * 3
            y = self.height/2 + (i % 3 - 1) * self.agent_half_size * 3
            agent = Agent(self.agent_radius, x, y, self.agent_half_size)
            self.agents.append(agent)

            self.set_grid(x, y, agent.half_size, agent.half_size, 2)

    def pos_change(self):
        for agent in self.agents:
            self.set_grid(agent.last_x, agent.last_y, agent.half_size, agent.half_size, 0)
            self.set_grid(agent.x, agent.y, agent.half_size, agent.half_size, 2)


    def check_overlap(self, x, y, half_width, half_height, objects):
            """Check if a rectangle at (x,y) with given width/height overlaps with any object in the list"""
            for obj in objects:
                if type(obj) == Agent:
                    obj_x = obj.x
                    obj_y = obj.y
                    obj_hw = obj.half_size
                    obj_hh = obj.half_size
                elif len(obj) == 3:
                    obj_x, obj_y, obj_o = obj
                    obj_hw = obj_o
                    obj_hh = obj_o
                else:    
                    obj_x, obj_y, obj_hw, obj_hh = obj
                
                # Calculate distances between centers
                dx = abs(x - obj_x)
                dy = abs(y - obj_y)
                
                # Calculate sum of half-widths and half-heights
                width_sum = (half_width + obj_hw)
                height_sum = (half_height + obj_hh)
                
                # Check for overlap
                if dx < width_sum and dy < height_sum:
                    return True
            
            return False
        
    def remove_target(self, x, y, half_size):
            self.set_grid(x, y, half_size, half_size, 0)

    def total_distance_covered(self):
        total_distance = 0
        for agent in self.agents:
            for i in range(len(agent.list_x)-1):
                total_distance += math.dist([agent.list_x[i], agent.list_y[i]], [agent.list_x[i+1], agent.list_y[i+1]])
        return total_distance

class Agent:
    pbest = np.inf
    pbest_p = np.inf
    pbest_avg = np.inf

    def __init__(self, radius, x, y, half_size):
        self.radius = radius
        self.x = x
        self.y = y
        self.Vx = np.random.uniform(-5, 5)
        self.Vy = np.random.uniform(-5, 5)
        self.last_x = x
        self.last_y = y
        self.px = x
        self.py = y
        self.pbest = float('inf')
        self.list_y = [y]
        self.list_x = [x]
        self.half_size = half_size  # Agent's physical size
        self.targets_found = 0
    
    def setcoords(self, x, y):
        self.last_x = self.x
        self.last_y = self.y
        self.x = x
        self.y = y
        self.list_x.append(x)
        self.list_y.append(y)
    
    def setVelocity(self, vx, vy):
        self.Vx = vx
        self.Vy = vy

fig, ax = plt.subplots(figsize=(6, 6))
plt.ion()  # Turn on interactive mode

def visualize_grid(grid, t, trails=True, done=False, name=None):
    color_grid = grid.grid
    cmap = ListedColormap(["gray", "black", "red", "yellow", 'darkgray'])
    labels = ['Empty', 'Obstacle', 'Agent', 'Target', 'Safe space']
    colors = ['gray', 'black', 'red', 'yellow', 'darkgray']
    legend_elements = [Patch(facecolor=colors[i], label=labels[i]) for i in range(len(labels))]
    ax.clear()  # Clear previous frame
    ax.imshow(color_grid, cmap=cmap)
    ax.axis("off")
    ax.set_title(f"PSO Agent Paths for iteration {t}")

    for robot in grid.agents:
        if trails:
            ax.plot(robot.list_x, robot.list_y, color='blue', linewidth=1, alpha=0.6)

    # Add legend once (optional: move outside if it causes clutter)
    ax.legend(handles=legend_elements, loc='upper right', fontsize='small')

    if t in [10, 50, 100] or done == True:
        plt.savefig(f'plots/{name}_agents-{grid.number_agents}_targets-{grid.nr_of_targets}_obstacles-{grid.nr_of_obstacles}_seed-{grid.seed}_step-{t}.png', bbox_inches='tight')
