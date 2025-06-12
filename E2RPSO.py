# Adapted from https://github.com/xrl2408/E2RPSO/tree/main
# Original author: xrl2408

import math

import random
import E2RPSO_util
from Continuous_grid import  visualize_grid

'''
function E2RPSO
    main function of PSO

    parameter:
        Side        : the map size
        Size        : num of robots
        Goal_num    : num of goal

    returns
        Iteration time, goal num found


'''

def check_target_detection(grid):
    """Check if an agent has detected any target

    Args:
        grid (Grid): Instance of Grid

    Returns:
        Bool: True if a target is detected, False otherwise. 
    """
    for agent in grid.agents:
        for i, target in enumerate(grid.targets):
            if target is None:  # Skip already found targets
                continue
                
            target_x, target_y, target_radius = target
            
            # Calculate distance between agent and target centers
            distance = math.sqrt((agent.x - target_x) ** 2 + (agent.y - target_y) ** 2)
            
            # Check if agent's detection radius overlaps with target
            if distance < agent.radius + target_radius:
                agent.targets_found += 1
                grid.remove_target(target_x, target_y, target_radius)
                grid.targets[i] = None  # Mark target as found
                return True
        
    return False

def move(agent, dx, dy, grid):
    """Move agent by (dx,dy) if there is no collision

    Args:
        agent (Agent): instance of an Agent
        dx (float): proposed new x location
        dy (float): proposed new y location
        grid (Grid): instance of the grid

    Returns:
        Bool: True if the coordinates are suitable, else False
    """
    new_x = agent.x + dx
    new_y = agent.y + dy
    # Check if new position would cause collision
    if not (E2RPSO_util.is_collision(grid, new_x, new_y, agent.radius) or E2RPSO_util.is_collision_agents(agent, grid, new_x, new_y, agent.radius)):

        # Update agent position
        agent.setcoords(new_x, new_y)        
        return True

    return False

def E2RPSO(grid, max_steps = 1000, step_size = 2):
    """Function controlling the E2RPSO process

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
    # Side length of square map
    V_LIMIT = step_size
    SIDE = grid.height
    
    # initial
    Out_list, agents, goal_list_x, goal_list_y, gx, gy, gbest = E2RPSO_util.init_E2RPSO(step_size, grid)
    far_area_id_list = [0] * len(agents)

    # PSO parameters
    C1 = 0.4
    C2 = 0.9
    C3_index = 1.5
    C3 = C3_index * C2
    K = 1
    uf = 50 #update frequency
   
    # begin iteration
    for steps in range(max_steps):

        # update best_avg
        best_avg = E2RPSO_util.update_best_avg(agents)

        # Traverse all robots
        for agent_idx, agent in enumerate(agents):
            r = agent
            r1 = random.uniform(0, 1)
            r2 = random.uniform(0, 1)
            r3 = random.uniform(0, 1)

            # update inertial component
            w = 0.9 - 0.5 * (1 - (min(r.pbest_p, r.pbest) / max(r.pbest_p, r.pbest))) + (
                        min(gbest, best_avg) / max(gbest, best_avg))

            # Find the farthest & emptiest area
            area_id = int(int((r.y - 1)/ 10) * SIDE / 10 + int(r.x / 10))
            Out_list[area_id] -= K
            if steps % uf == 0:
                far_area_id_list[agent_idx] = E2RPSO_util.find_FarthestAndEmptiestArea(area_id, Out_list)
                # farthest & emptiest exploration rate increment - K
                Out_list[far_area_id_list[agent_idx]] -= K

            # Determine whether to enter local optimum or not, then set C3
            if Out_list[area_id] <= 0:
                c1, c2, c3 = 0, 0, 2 * C3
            elif Out_list[area_id] < 25:
                c1, c2, c3 = 0, 0, C3
            else:
                c1, c2, c3 = C1, C2, 0

            # prepare for unit speed x,y component
            pu_x = (far_area_id_list[agent_idx] % 10) * 10 + 5
            pu_y = int((far_area_id_list[agent_idx]) // 10) * 10 + 5

            # Unit speed for x and y component
            xp, yp, xg, yg = E2RPSO_util.Unit_speed(r, gx, gy)

            # update velocity, using E2RPSO
            dist = math.hypot(pu_x - r.x, pu_y - r.y)
            v_x = w * r.Vx + r1 * c1 * xp + c2 * r2 * xg + c3 * r3 * ((pu_x - r.x)/dist)
         
            v_y = w * r.Vy + r1 * c1 * yp + c2 * r2 * yg + c3 * r3 * ((pu_y - r.y)/dist)

            # limit max velocity
            v_x *= 100
            v_y *= 100
            v_x, v_y = E2RPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT)

            # update position
            if not move(r, v_x, v_y, grid):
                new_x, new_y, v_x, v_y = E2RPSO_util.avoidObstacle(r, v_x, v_y, V_LIMIT, grid)
                r.setcoords(new_x, new_y)

            r.setVelocity(v_x, v_y)

            # update Pbest & Gbest
            gbest, gx, gy = E2RPSO_util.update_GbestPbest(r, gx, gy, gbest, goal_list_x, goal_list_y, steps)
            
        grid.pos_change()
        if check_target_detection(grid):
            targets_found += 1

        # Optional: Visualize grid state periodically
        if steps % 10 == 0:
            visualize_grid(grid, steps, name='E2RPSO')

        if targets_found == total_targets:
            break
        else:
            a, b, c = zip(*[(ai, bi, ci) for ai, bi, ci in zip(grid.targets, goal_list_x, goal_list_y) if ai is not None])
            grid.targets = list(a)
            goal_list_x = list(b)
            goal_list_y = list(c)

    total_distance = grid.total_distance_covered()
        
    visualize_grid(grid, steps, done = True, name='E2RPSO')
        
    return steps, targets_found, total_distance













