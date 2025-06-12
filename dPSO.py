import numpy as np
import math
import random
from Continuous_grid import visualize_grid, Grid
import dPSO_util

class Particle:
    def __init__(self, x, y, p_radius):
        """This is the initialization function of the Particle class

        Args:
            x (float): coordinate X
            y (float): coordinate Y
            p_radius (int): radius if the particle
        """
        self.x = x
        self.y = y
        self.vx = 0
        self.vy = 0
        self.px = x
        self.py = y
        self.pbest = float('inf')
        self.radius = p_radius
    
    def setcoords(self, x, y):
        """Setting the corodinates of the particle

        Args:
            x (float): New x coordinate
            y (float): New y coordinate
        """
        self.x = x
        self.y = y

    def setVelocity(self, vx, vy):
        """Function for setting the velocity of the particle

        Args:
            vx (float): new x velocity
            vy (float): new y coordinate
        """
        self.vx = vx
        self.vy = vy

class Swarm:
    def __init__(self, agent, grid, num_particles=10, sigma=3):
        """Initialization function of the Swarm class

        Args:
            agent (Agent): Agent objects from class Agent in Continuous_grid.py
            grid (Grid): Instance of the grid
            num_particles (int): number of particles. Defaults to 10.
            sigma (int): representen the standard deviation for the nromal distribution. Defaults to 3.
        """
        self.agent = agent
        self.particles = []
        self.gx = agent.x
        self.gy = agent.y
        self.gbest = np.inf
        self.sigma = sigma
        self.initialize_particles(grid, num_particles)

    def initialize_particles(self, grid, num_particles):
        """Function that initializes the particles

        Args:
            grid (Grid): Instance of the grid
            num_particles (int): number of particles_
        """
        #self.agent.grid = None  # will be set externally
        attempts = 0
        while len(self.particles) < num_particles and attempts < num_particles * 10:
            x = np.random.normal(self.agent.x, self.sigma)
            y = np.random.normal(self.agent.y, self.sigma)
            if not (dPSO_util.is_collision(grid, x, y, self.agent.radius) or
                    dPSO_util.is_collision_particles(self, None, grid, x, y, self.agent.radius)):
                self.particles.append(Particle(x, y, self.agent.radius))
            attempts += 1

    def update_particles(self, grid, V_LIMIT, goal_list_x, goal_list_y, c1, c2, c3=1, c4=1, w=0.5, max_dis = 8):
        """Function that updates the particle function on the grid

        Args:
            grid (Grid): instance of the grid
            V_LIMIT (int): Maximum velocity of the particle
            goal_list_x (list): list containing the x coordinates of the targets
            goal_list_y (list): list containing the y coordinates of the targets
            c1 (float): learning constant
            c2 (float): learning constant
            c3 (int, optional): learning constant. Defaults to 1.
            c4 (int, optional): learning constant. Defaults to 1.
            w (float, optional): inertia weight. Defaults to 0.5.
            max_dis (int, optional): Used for computing the distance between the agent and the particle. Defaults to 8.

        Returns:
            float: the coordinate of the personal best
        """
        for p in self.particles:    
            r1 = random.uniform(0, 1)
            r2 = random.uniform(0, 1)
            r3 = random.uniform(0, 1)
            r4 = random.uniform(0, 1)

            # Unit speed for x and y component
            xp, yp, xg, yg = dPSO_util.Unit_speed(p, self.gx, self.gy)

            v_x = w * p.vx + c1 * r1 * xp + c2 * r2 * xg
            v_y = w * p.vy + c1 * r1 * yp + c2 * r2 * yg

            dis = math.hypot(self.agent.x - p.x, self.agent.y - p.y)
            if dis > max_dis:
                v_x += c3 * r3 * (1 + dis - max_dis) * ((self.agent.x - p.x)/dis)
                v_y += c3 * r3 * (1 + dis - max_dis) * ((self.agent.y - p.y)/dis)

            v_x *= 100
            v_y *= 100
            v_x, v_y = dPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT*2)          

            # update position
            if not move(self, p, v_x, v_y, grid):
                new_x, new_y, v_x, v_y = dPSO_util.avoidObstacle(p, v_x, v_y, V_LIMIT*2, grid)
                p.setcoords(new_x, new_y)

            p.setVelocity(v_x, v_y)

            self.gbest, self.gx, self.gy = dPSO_util.update_GbestPbest(p, self.gx, self.gy, self.gbest, goal_list_x, goal_list_y)

        return self.gx, self.gy

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

def move(swarm, agent, dx, dy, grid):
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
    if not (dPSO_util.is_collision(grid, new_x, new_y, agent.radius) or 
            dPSO_util.is_collision_agents(agent, grid, new_x, new_y, agent.radius) or
            dPSO_util.is_collision_particles(swarm, agent, grid, new_x, new_y, agent.radius)):

        # Update agent position
        agent.setcoords(new_x, new_y)        
        return True

    return False

# --- Main DPSO Execution ---
def dPSO(grid, max_steps = 1000, step_size = 2):
    total_targets = len(grid.targets)
    targets_found = 0
    V_LIMIT = step_size

    c1 = 0.4
    c2 = 0.9

    w_upper = 0.9
    w_lower = 0.4
    
    # initial
    agents, goal_list_x, goal_list_y, gx, gy, gbest = dPSO_util.init_dPSO(step_size,grid)
    swarms = [Swarm(agent, grid) for agent in agents]

    for steps in range(max_steps):

        w = w_upper - (steps / max_steps) * (w_upper - w_lower)
                                    
        # Traverse all robots
        for swarm in swarms:

            r1 = random.uniform(0, 1)
            r2 = random.uniform(0, 1)
                                                                                                            
            # Unit speed for x and y component
            xp, yp, xg, yg = dPSO_util.Unit_speed(swarm.agent, gx, gy)

            # update velocity , using RDPSO                                                                       
            v_x = w * swarm.agent.Vx + r1 * c1 * xp + c2 * r2 * xg
            v_y = w * swarm.agent.Vy + r1 * c1 * yp + c2 * r2 * yg 
                                                                                                                                
            # limit max velocity
            v_x *= 100
            v_y *= 100
            v_x, v_y = dPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT)          

            # update position
            if not move(swarm, swarm.agent, v_x, v_y, grid):
                new_x, new_y, v_x, v_y = dPSO_util.avoidObstacle(swarm.agent, v_x, v_y, V_LIMIT, grid)
                swarm.agent.setcoords(new_x, new_y)

            swarm.agent.setVelocity(v_x, v_y)


            spx, spy = swarm.update_particles(grid, swarms, V_LIMIT, goal_list_x, goal_list_y, c1, c2, w = w)

            f = 0
            for ttt in range(len(goal_list_x)):
                tmp_f = dPSO_util.getFitness(goal_list_x[ttt], goal_list_y[ttt], spx, spy)
                f += tmp_f

            if f <= swarm.agent.pbest:
                swarm.agent.pbest = f
                swarm.agent.px = spx
                swarm.agent.py = spy
            if f <= gbest:
                gbest = f
                gx = swarm.agent.px
                gy = swarm.agent.py    
            # update Pbest & Gbest
            gbest, gx, gy = dPSO_util.update_GbestPbest(swarm.agent, gx, gy, gbest, goal_list_x, goal_list_y)


        grid.pos_change()
        if check_target_detection(grid):
            targets_found += 1

        # Optional: Visualize grid state periodically
        if steps % 10 == 0:
            visualize_grid(grid, steps, name='dPSO')

        if targets_found == total_targets:
            break
        else:
            a, b, c = zip(*[(ai, bi, ci) for ai, bi, ci in zip(grid.targets, goal_list_x, goal_list_y) if ai is not None])
            grid.targets = list(a)
            goal_list_x = list(b)
            goal_list_y = list(c)

    total_distance = grid.total_distance_covered()
        
    visualize_grid(grid, steps, done = True, name='dPSO')
        
    return steps, targets_found, total_distance




