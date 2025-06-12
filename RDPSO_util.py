# Adapted from https://github.com/xrl2408/E2RPSO/tree/main
# Original author: xrl2408


import random
import math

def getFitness(gx, gy, x, y):
    """Function that computes the fitness function

    Args:
        gx (float): x coordinate of the desired state
        gy (float): y coordinate of the desired state
        x (float): x coordinate of the agent
        y (float): y coordinate of the agent

    Returns:
        float: value of fitness
    """
    dis = math.sqrt((gx - x) * (gx - x) + (gy - y) * (gy - y))
    sq = math.sqrt(dis)

    return math.sqrt(sq)

# update Gbest Pbest
def update_GbestPbest(r, Gx, Gy, Gbest, goal_list_x, goal_list_y):
    """Updates the personal best of an agent and the global best across all agents, 
    based on current fitness compared to stored values

    Args:
        r (Agent): instance of Agent
        Gx (float): x coordinate of the global best
        Gy (float): y coorindate of the global best
        Gbest (float): value of the global best
        goal_list_x (list): list containing the x coordinates of all goals
        goal_list_y (_type_): list containing the y coordinates of all goals

    Returns:
        float: x coordinate of the global best
        float: y coorindate of the global best
        float: value of the global best
    """   
    gx = Gx
    gy = Gy
    gbest = Gbest
    f = 0
    for ttt in range(len(goal_list_x)):
        tmp_f = getFitness(goal_list_x[ttt], goal_list_y[ttt], r.x, r.y)
        f += tmp_f

    if f <= r.pbest:
        r.pbest = f
        r.px = r.x
        r.py = r.y
    if f <= gbest:
        gbest = f
        gx = r.px
        gy = r.py

    return gbest, gx, gy

# limit max velocity
def Limit_maxVelocity(v_x, v_y, v_limit):
    """Function that checks if the current velocity exceeds the limit and clips it if it does

    Args:
        v_x (float): Current x velocity
        v_y (float): Current y velocity
        v_limit (float): maximum value the velocity may have. 

    Returns:
        float: checked velocity for x
        float: checked velocity for y
    """
    if abs(v_x) > v_limit / 2 or abs(v_y) > v_limit / 2:
        k = v_x * v_x + v_y * v_y
        k = math.sqrt(k)
        v_x = v_x * v_limit / k
        v_y = v_y * v_limit / k
    return v_x, v_y

def Unit_speed(r, gx, gy):
    """Compute normalized directional vectors toward the agent's personal best
    and the global best from its current position.

    Args:
        r (Agent): instance of Agent
        gx (float): x coordinate of the global best
        gy (float): y cooridnate of the global best

    Returns:
        tuple: Unit vectors toward personal best (xp, yp) and global best (xg, yg).
  
    """
    xp = r.px - r.x
    xg = gx - r.x
    yp = r.py - r.y
    yg = gy - r.y
    p_len = math.sqrt(xp ** 2 + yp ** 2)
    if p_len > 0:
        xp = xp / p_len
        yp = yp / p_len
    g_len = math.sqrt(xg ** 2 + yg ** 2)
    if g_len > 0:
        xg = xg / g_len
        yg = yg / g_len
    return xp, yp, xg, yg

def is_collision(grid, x, y, radius):
    """Check if a point (x,y) with given radius collides with any obstacle

    Args:
        grid (Grid): instance of Grid
        x (float): x coordinate of the agent
        y (float): y coordinate of the agent
        radius (int): sensing range of the agent

    Returns:
        Bool: True if there is a collision, false otherwise
    """    # Check bounds
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

def is_collision_agents(agent_self, grid, x, y, radius):
    """Check if a point (x,y) with given radius collides with any agent

    Args:
        agent_self (Agent): instacne of class Agent
        grid (Grid): instance of a grid
        x (float): x coordinate of the agent
        y (float): y coordinate of the agent
        radius (int): sensing range of the agent

    Returns:
        Bool: True if there is a collision, false otherwise
    """    # Check bounds
    if x - radius < 0 or x + radius > grid.width or y - radius < 0 or y + radius > grid.height:
        return True

    # Check obstacles
    for agent in grid.agents: 
        if agent != agent_self:
            ag_x = agent.x
            ag_y = agent.y
            ag_w = agent.half_size 
            ag_h = agent.half_size  
            
            # Calculate closest point on rectangle to circle center
            closest_x = max(ag_x - ag_w, min(x, ag_x + ag_w))
            closest_y = max(ag_y - ag_h, min(y, ag_y + ag_h))
            
            # Calculate distance between closest point and circle center
            distance = math.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)
            
            # Check collision
            if distance < radius:
                return True
        
    return False

# Avoid obstacle
def avoidObstacle(r, v_x, v_y, V_LIMIT, grid):
    """Function that computes how to avoid the obstacle

    Args:
        r (Agent): instance of Agent
        v_x (float): x velocity
        v_y (float): y velocity
        V_LIMIT (float): maximum velocity
        grid (Grid): instance of a grid

    Returns:
        float: New computed values to avoid obstacles
    """
    # x,y direction
    angle_list = [10, -10, 20, -20, 30, -30, 40, -40, 50, -50, 60, -60, 70, -70, 80, -80, 90, -90, 100, -100,
                    110, -110, 120, -120, 130, -130, 140, -140, 150, -150, 160, -160, 170, -170, 180, -180]
    
    for magnitude in [V_LIMIT * f for f in [1.0, 0.8, 0.6, 0.4, 0.2]]:
        for angle in angle_list:
            theta = math.atan2(v_y, v_x) + math.radians(angle)
            ndx = math.cos(theta) * magnitude
            ndy = math.sin(theta) * magnitude

            new_x = r.x + ndx
            new_y = r.y + ndy

            if not (is_collision(grid, new_x, new_y, r.radius) or is_collision_agents(r, grid, new_x, new_y, r.radius)):
                return new_x, new_y, ndx, ndy
    return r.x, r.y, 0.0, 0.0

def init_RDPSO(step_size,grid):
    """Init function for the DPSO algorithm

    Args:
        step_size (float): Maximum velocity
        grid (Grid): instance of Grid

    Returns:
            list: List of agent objects after initialization.
            list: List of x-coordinates of goal positions.
            list: List of y-coordinates of goal positions.
            float: x-coordinate of the global best position.
            float: y-coordinate of the global best position.
            float: Best (lowest) fitness value found globally.
    """

    for agent in grid.agents:
        agent.setVelocity(random.uniform(0, step_size), random.uniform(-step_size, step_size))

    goal_list_x = []
    goal_list_y = []
    for x, y, _ in grid.targets:
        goal_list_x.append(x)
        goal_list_y.append(y)

    gbest = 200000
    gx = 0
    gy = 0

    for agent in grid.agents:
        f = 0
        for ttt in range(len(goal_list_x)):
            tmp_f = getFitness(goal_list_x[ttt], goal_list_y[ttt], agent.x, agent.y)
            f += tmp_f

        if f < agent.pbest:
            agent.px = agent.x
            agent.py = agent.y
            agent.pbest = f
            if f < gbest:
                gbest = f
                gx = agent.px
                gy = agent.py

    return grid.agents,goal_list_x,goal_list_y, gx, gy, gbest

