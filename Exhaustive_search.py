from Continuous_grid import Grid, Agent, visualize_grid
import random
import math
import numpy as np

def check_target_detection(grid):
    """Check if an agent has detected any target"""
    for agent in grid.agents:
        for i, target in enumerate(grid.targets):
            if target is None:  # Skip already found targets
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

def is_collision(grid, x, y, radius):
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

def divisors(n):
    return [i for i in range(1, n + 1) if n % i == 0]


def move(agent, dx, dy, grid):
    """Move agent by (dx,dy) if no collision"""
    new_x = agent.x + dx
    new_y = agent.y + dy
    # Check if new position would cause collision
    if not is_collision(grid, new_x, new_y, agent.radius):

        # Update agent position
        agent.setcoords(new_x, new_y)        
        return True

    return False

def exhaustive_search(grid, max_steps=1000, step_size=2):
    total_targets = len(grid.targets)
    targets_found = 0
    steps = 0
    
    visualize_grid(grid, 0)
    # Create search patterns for each agent
    search_patterns = []
    search_positions = []
    
    # We'll divide the grid into sections, one for each agent
    num_agents = grid.number_agents
    
    # Calculate how to divide the grid
    if num_agents == 1:
        regions = [(0, grid.width, 0, grid.height)]
    else:
        # Try to create a grid of regions
        divisors_list = divisors(num_agents)
        rows = divisors_list[len(divisors_list)//2]
        cols = num_agents // rows
        
        width_per_region = grid.width / cols
        height_per_region = grid.height / rows
        
        regions = []
        for i in range(rows):
            for j in range(cols):
                if len(regions) < num_agents:
                    regions.append((
                        j * width_per_region,  # x_min
                        (j + 1) * width_per_region,  # x_max
                        i * height_per_region,  # y_min
                        (i + 1) * height_per_region   # y_max
                    ))
    # Create a search pattern for each region
    for i, (x_min, x_max, y_min, y_max) in enumerate(regions):
        if i >= num_agents:
            break
            
        pattern = []
        
        hor_step = grid.agent_radius * 0.9
        vert_step = grid.agent_radius * 0.9

        # Calculate number of points to create in each dimension
        x_points = int(max(2, int((x_max - x_min)))//hor_step)
        y_points = int(max(2, int((y_max - y_min)))//vert_step)

        # Create zigzag pattern: alternate left-to-right and right-to-left rows
        for y_idx in range(y_points):
            y = y_min + (y_idx + 0.5) * vert_step
        
            if y_idx % 2 == 0:  # Left to right
                for x_idx in range(x_points):
                    x = x_min + (x_idx + 0.5) * hor_step
                    pattern.append((x, y))
            else:  # Right to left
                for x_idx in range(x_points - 1, -1, -1):
                    x = x_min + (x_idx + 0.5) * hor_step
                    pattern.append((x, y))
        search_patterns.append(pattern)
        search_positions.append(0)  # Start at the beginning of each pattern

    stuck_counter = [0] * len(grid.agents)
    detour_steps = [0] * len(grid.agents)
    step_dir = [0, 0] * len(grid.agents)
    reached_pattern_start = [False] * len(grid.agents)
    previous_distances = [float('inf')] * len(grid.agents)
    detour_directions = [(step_size, 0), (-step_size, 0)]

    # Main search loop
    for steps in range(max_steps):
        
        # Move each agent according to its search pattern
        for agent_idx, agent in enumerate(grid.agents):
            if agent_idx < len(search_patterns):
                pattern = search_patterns[agent_idx]
                pos_idx = search_positions[agent_idx]

                # Skip unreachable waypoints
                while pos_idx < len(pattern):
                    target_x, target_y = pattern[pos_idx]
                    if not is_collision(grid, target_x, target_y, agent.radius):
                        break
                    else:
                        search_positions[agent_idx] += 1
                        pos_idx += 1
                
                if pos_idx < len(pattern):
                    target_x, target_y = pattern[pos_idx]

                    dx = target_x - agent.x
                    dy = target_y - agent.y

                    # Calculate distance to the target
                    distance = math.sqrt(dx**2 + dy**2)

                    if distance < step_size:  # We're close enough to consider it reached
                        # Move directly to the target
                        move(agent, dx, dy, grid)
                        # Go to next waypoint
                        search_positions[agent_idx] += 1
                        reached_pattern_start[agent_idx] = True
                        stuck_counter[agent_idx] = 0
                        previous_distances[agent_idx] = float('inf')

                    elif detour_steps[agent_idx] > 0:
                        detour_steps[agent_idx] -= 1
                        #angle = math.atan2(dy, dx) + math.radians(angle_detour[agent_idx])
                        #ndx = math.cos(angle) * step_size
                        #ndy = math.sin(angle) * step_size
                        ndy = step_dir[agent_idx][0]
                        ndx = step_dir[agent_idx][1]

                        move(agent, ndx, ndy, grid)

                    #elif stuck_counter[agent_idx] >= 10:
                    #    detour_steps[agent_idx] = 5
                    #    stuck_counter[agent_idx] = 0 
                        
                    else:   
                        moved = False              
                        # Normalize direction and scale by step size
                        dx = (dx / distance) * step_size
                        dy = (dy / distance) * step_size

                        for angle_offset in [0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90]:
                            angle = math.atan2(dy, dx) + math.radians(angle_offset)
                            ndx = math.cos(angle) * step_size
                            ndy = math.sin(angle) * step_size

                            if move(agent, ndx, ndy, grid):
                                moved = True
                                break
                        
                        if not moved:
                            for _ in range(5):
                                angle = random.uniform(0, 2 * math.pi)
                                ndx = math.cos(angle) * step_size
                                ndy = math.sin(angle) * step_size
                                if move(agent, ndx, ndy, grid):
                                    break

                        if detour_steps[agent_idx] == 0:
                            if distance >= previous_distances[agent_idx] - step_size*0.5:
                                stuck_counter[agent_idx] += 1
                            else:
                                stuck_counter[agent_idx] = 0

                            previous_distances[agent_idx] = distance

                            if stuck_counter[agent_idx] >= 3:
                                detour_steps[agent_idx] = random.choice([5, 10])
                                step_dir[agent_idx] = random.choice(detour_directions)
                                stuck_counter[agent_idx] = 0

                else:
                    # Agent has completed its search pattern, use random movement
                    angle = random.uniform(0, 2 * math.pi)
                    dx = step_size * math.cos(angle)
                    dy = step_size * math.sin(angle)
                    move(agent, dx, dy, grid)
            else:
                # Extra agents use random movement
                angle = random.uniform(0, 2 * math.pi)
                dx = step_size * math.cos(angle)
                dy = step_size * math.sin(angle)
                move(agent, dx, dy, grid)
            
        grid.pos_change()
        if check_target_detection(grid):
            targets_found += 1
        # Optional: Visualize grid state periodically
        if steps % 10 == 0:
            visualize_grid(grid, steps, name='Exhaustive')    
        if targets_found == total_targets:
            break

    total_distance = grid.total_distance_covered()

    

    visualize_grid(grid, steps, name='Exhaustive') 
    
    return steps, targets_found, total_distance

