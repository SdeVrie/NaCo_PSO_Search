import math

from Continuous_grid import visualize_grid
import random
import RDPSO_util

'''
    function RDPSO
    main function of PSO
    
    parameter:
    Side        : the map size
    Size        : num of robots
    Goal_num    : num of goal
    
    returns
    Iteration time, goal num found
    
    
    '''

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
            if distance < agent.radius + target_radius:
                agent.targets_found += 1
                grid.remove_target(target_x, target_y, target_radius)
                grid.targets[i] = None  # Mark target as found
                return True
        
    return False

def move(agent, dx, dy, grid):
    """Move agent by (dx,dy) if no collision"""
    new_x = agent.x + dx
    new_y = agent.y + dy
    # Check if new position would cause collision
    if not (RDPSO_util.is_collision(grid, new_x, new_y, agent.radius) or RDPSO_util.is_collision_agents(agent, grid, new_x, new_y, agent.radius)):

        # Update agent position
        agent.setcoords(new_x, new_y)        
        return True

    return False

def RDPSO(grid, max_steps = 1000, step_size = 2):
    total_targets = len(grid.targets)
    targets_found = 0
    # Side length of square map
    V_LIMIT = step_size
    
    # initial
    agents, goal_list_x, goal_list_y, gx, gy, gbest = RDPSO_util.init_RDPSO(step_size,grid)
    nr_of_agents = len(agents)
    fugitive_list = [0] * len(agents)
    
    # PSO parameters
    c1 = 0.4
    c2 = 0.9

    w_upper = 0.9
    w_lower = 0.4
    
    # begin iteration
    for steps in range(max_steps):
        
        w = w_upper - (steps / max_steps) * (w_upper - w_lower)
        
        # find and change fugitive
        if steps % 20 == 0 and steps > 0:
            best_agent_indices = sorted(range(len(agents)), key=lambda i: agents[i].pbest, reverse=True)[:int(nr_of_agents / 3)]
            for i in range(nr_of_agents):
                if i in best_agent_indices:
                    fugitive_list[i] = 1
                else:
                    fugitive_list[i] = 0
                                
        # Traverse all robots
        for agent_idx, agent in enumerate(agents):

            r1 = random.uniform(0, 1)
            r2 = random.uniform(0, 1)
                                                                                                          
            # Unit speed for x and y component
            xp, yp, xg, yg = RDPSO_util.Unit_speed(agent, gx, gy)

            # update velocity , using RDPSO                                                                       
            if fugitive_list[agent_idx] == 0:
                v_x = w * agent.Vx + r1 * c1 * xp + c2 * r2 * xg
                v_y = w * agent.Vy + r1 * c1 * yp + c2 * r2 * yg 

            else :
                v_x = (1+random.uniform(-1, 1))*w * agent.Vx
                v_y = (1+random.uniform(-1, 1))*w * agent.Vy
                                                                                                                                
            # limit max velocity
            v_x *= 100
            v_y *= 100
            v_x, v_y = RDPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT)          

            # update position
            if not move(agent, v_x, v_y, grid):
                new_x, new_y, v_x, v_y = RDPSO_util.avoidObstacle(agent, v_x, v_y, V_LIMIT, grid)
                agent.setcoords(new_x, new_y)

            agent.setVelocity(v_x, v_y)

            # # deal root out of map
            # if (agent.x < 0 or agent.y < 0 or agent.x > SIDE or agent.y > SIDE):
            #     agent.x = agent.x - v_x
            #     agent.y = agent.y - v_y
            #     agent.Vx = -agent.Vx
            #     agent.Vy = -agent.Vy
                            
            # update Pbest & Gbest
            gbest, gx, gy = RDPSO_util.update_GbestPbest(agent, gx, gy, gbest, goal_list_x, goal_list_y)

        grid.pos_change()
        if check_target_detection(grid):
            targets_found += 1

        # Optional: Visualize grid state periodically
        if steps % 10 == 0:
            visualize_grid(grid, steps, name='RDPSO')

        if targets_found == total_targets:
            break
        else:
            a, b, c = zip(*[(ai, bi, ci) for ai, bi, ci in zip(grid.targets, goal_list_x, goal_list_y) if ai is not None])
            grid.targets = list(a)
            goal_list_x = list(b)
            goal_list_y = list(c)

    total_distance = grid.total_distance_covered()
        
    visualize_grid(grid, steps, done = True, name='RDPSO')
        
    return steps, targets_found, total_distance





