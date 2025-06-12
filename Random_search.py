from Continuous_grid import Grid, Agent, visualize_grid
import random
import math

def check_target_detection(grid):
    """Check if an agent has detected any target

    Args:
        grid (Grid): Instance of Grid

    Returns:
        Bool: True if a target is detected, False otherwise. 
    """
    for agent in grid.agents:
        for i, target in enumerate(grid.targets):
            if target is None: 
                 # Skip already found targets
                continue
            target_x, target_y, target_radius = target
            
            # Calculate distance between agent and target centers
            distance = math.sqrt((agent.x - target_x) ** 2 + (agent.y - target_y) ** 2)
            
            # Check if agent's detection radius overlaps with target
            if distance < grid.agents[0].radius + grid.target_half_size + 3:
                agent.targets_found += 1
                grid.remove_target(target_x, target_y, target_radius)
                grid.targets[i] = None  # Mark target as found
                return True
        
    return False

def is_collision( grid, x, y, radius):
    """Check if a point (x,y) with given radius collides with any obstacle

    Args:
        grid (Grid): instance of Grid
        x (float): x coordinate of the agent
        y (float): y coordinate of the agent
        radius (int): sensing range of the agent

    Returns:
        Bool: True if there is a collision, false otherwise
    """
    # Check bounds
    if x - radius < 0 or x + radius > grid.width or y - radius < 0 or y + radius > grid.height:
        return True
    
    # Check obstacles
    for obs_x, obs_y, obs_w, obs_h in grid.obstacles:
        
        # Calculate closest point on rectangle to circle center
        closest_x = max(obs_x - obs_w, min(x, obs_x + obs_w))
        closest_y = max(obs_y - obs_h, min(y, obs_y + obs_h))
        
        # Calculate distance between closest point and circle center
        distance = math.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)
        
        # Check collision
        if distance < radius:
            return True
    
    return False

def move(agent, grid, step_size):
    """Function that moves an agent if there is no collision

    Args:
        agent (Agent): instance of an Agent
        grid (Grid): instance of a Grid
        step_size (float): maximum velocity
    """
                # Calculate movement vector
    
    while True:
        angle = random.uniform(0, 2 * math.pi)
        dx = step_size * math.cos(angle)
        dy = step_size * math.sin(angle)
        new_x = agent.x + dx
        new_y = agent.y + dy
        
        # Check if new position would cause collision
        if not is_collision(grid, new_x, new_y, agent.radius):
            agent.setcoords(new_x, new_y) 
            return True
        
        return False

def random_search(grid, max_steps=1000, step_size=2):
    """Function that controls the process of random search

    Args:
        grid (Grid): Instance of the grid
        max_steps (int): maximum amount of steps. Defaults to 1000.
        step_size (int): maximum step size. Defaults to 2.

    Returns:
        int: number of steps needed
        int: number of targets found
        float: distance covered by all the agents
    """
    total_targets = len(grid.targets)
    targets_found = 0
    steps = 0
    
    visualize_grid(grid, 0)

    for steps in range(max_steps):
        # Move each agent in a random direction
        for agent in grid.agents:
            # Generate random movement direction
            
            # Try to move the agent
            move(agent, grid, step_size)
            
            # Check if agent has found a target

        grid.pos_change()
        if check_target_detection(grid):
            targets_found += 1
        # Optional: Visualize grid state at each step
        if steps % 50 == 0:     
            visualize_grid(grid, steps, name='Random')
        if targets_found == total_targets:
            break
    

    total_distance = grid.total_distance_covered()

    
    # Visualize the final grid state
    visualize_grid(grid, steps, name='Random')
    
    return steps, targets_found, total_distance
