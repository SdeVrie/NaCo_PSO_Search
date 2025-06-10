from Continuous_grid import Grid, Agent, visualize_grid
import random
import math

def check_target_detection(grid):
    """Check if an agent has detected any target"""
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
    """Check if a point (x,y) with given radius collides with any obstacle"""
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
    """Move agent by (dx,dy) if no collision"""
                # Calculate movement vector
    
    while True:
        angle = random.uniform(0, 2 * math.pi)
        dx = step_size * math.cos(angle)
        dy = step_size * math.sin(angle)
        new_x = agent.x + dx
        new_y = agent.y + dy
        
        # Check if new position would cause collision
        if not is_collision(grid, new_x, new_y, agent.radius):
            # Update visualization grid
            # old_x_min = max(0, int((agent.x - agent.size) * grid.scale))
            # old_x_max = min(grid.grid_width, int((agent.x + agent.size) * grid.scale))
            # old_y_min = max(0, int((agent.y - agent.size) * grid.scale))
            # old_y_max = min(grid.grid_height, int((agent.y + agent.size) * grid.scale))
            
            agent.setcoords(new_x, new_y) 
            return True
        
        return False

def random_search(grid, max_steps=1000, step_size=2):
    """Random search algorithm in continuous space"""
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
