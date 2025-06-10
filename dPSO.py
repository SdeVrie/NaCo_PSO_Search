import numpy as np
import math
import random
from Continuous_grid import visualize_grid, Grid
import dPSO_util

class Particle:
    def __init__(self, x, y, p_radius):
        self.x = x
        self.y = y
        self.vx = 0
        self.vy = 0
        self.px = x
        self.py = y
        self.pbest = float('inf')
        self.radius = p_radius
    
    def setcoords(self, x, y):
        self.x = x
        self.y = y

    def setVelocity(self, vx, vy):
        self.vx = vx
        self.vy = vy

class Swarm:
    def __init__(self, agent, grid, num_particles=10, sigma=3):
        self.agent = agent
        self.particles = []
        self.gx = agent.x
        self.gy = agent.y
        self.gbest = np.inf
        self.sigma = sigma
        self.initialize_particles(grid, num_particles)

    def initialize_particles(self, grid, num_particles):
        #self.agent.grid = None  # will be set externally
        attempts = 0
        while len(self.particles) < num_particles and attempts < num_particles * 10:
            x = np.random.normal(self.agent.x, self.sigma)
            y = np.random.normal(self.agent.y, self.sigma)
            if not (dPSO_util.is_collision(grid, x, y, self.agent.radius) or
                    dPSO_util.is_collision_particles(self, None, grid, x, y, self.agent.radius)):
                self.particles.append(Particle(x, y, self.agent.radius))
            attempts += 1

    def update_particles(self, grid, all_swarms, V_LIMIT, goal_list_x, goal_list_y, c1, c2, c3=1, c4=1, w=0.5, max_dis = 8):
        for p in self.particles:
#             distances = [
#     math.sqrt((p.x - tx)**2 + (p.y - ty)**2)
#     for target in grid.targets if target is not None
#     for tx, ty, _ in [target]
# ]
#             fitness = min(distances) if distances else np.inf

#             if fitness < p.fitness:
#                 p.pbest_x, p.pbest_y = p.x, p.y
#                 p.fitness = fitness
#                 if fitness < self.gbest_value:
#                     self.gbest_x, self.gbest_y = p.x, p.y
#                     self.gbest_value = fitness
            
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

            # for other in self.particles:
            #     if other == p: continue
            #     d = math.hypot(p.x - other.x, p.y - other.y)
            #     if d == 0: continue
            #     rep = 1 / (1 + (d / 1.5)**2)
            #     v_x += c3 * r3 * (p.x - other.x) * rep
            #     v_y += c3 * r3 * (p.y - other.y) * rep

            # for swarm in all_swarms:
            #     if swarm == self: continue
            #     d = math.hypot(p.x - swarm.agent.x, p.y - swarm.agent.y)
            #     rep = 1 / (1 + (d / 2.0)**2)
            #     v_x += c4 * r4 * (p.x - swarm.agent.x) * rep
            #     v_y += c4 * r4 * (p.y - swarm.agent.y) * rep

            # Apply and check for collisions
            v_x *= 100
            v_y *= 100
            v_x, v_y = dPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT*2)          

            # update position
            if not move(self, p, v_x, v_y, grid):
                new_x, new_y, v_x, v_y = dPSO_util.avoidObstacle(p, v_x, v_y, V_LIMIT*2, grid)
                p.setcoords(new_x, new_y)

            p.setVelocity(v_x, v_y)

            self.gbest, self.gx, self.gy = dPSO_util.update_GbestPbest(p, self.gx, self.gy, self.gbest, goal_list_x, goal_list_y)
            # new_x = p.x + vel_x
            # new_y = p.y + vel_y
            # if not dPSO_util.is_collision(grid, new_x, new_y, grid.agent_radius):
            #     p.vx, p.vy = vel_x, vel_y
            #     p.x, p.y = new_x, new_y
            # else:
            #     # Small random movement to escape stagnation
            #     angle = random.uniform(0, 2 * math.pi)
            #     p.vx = math.cos(angle) * 0.5
            #     p.vy = math.sin(angle) * 0.5
            #     alt_x = p.x + p.vx
            #     alt_y = p.y + p.vy
            #     if not dPSO_util.is_collision(grid, alt_x, alt_y, grid.agent_radius):
            #         p.x, p.y = alt_x, alt_y
            #     else:
            #         p.vx = p.vy = 0
        
        return self.gx, self.gy

    def update_agent_position(self, V_LIMIT):
        avg_x = np.mean([p.x for p in self.particles])
        avg_y = np.mean([p.y for p in self.particles])
        print(avg_x, avg_y)
        print(self.agent.x, self.agent.y)
        v_x = avg_x - self.agent.x
        v_y = avg_y - self.agent.y
        print(v_x, v_y)
        v_x, v_y = dPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT)
        print(v_x, v_y)
        print()
        if not move(self, self.agent, v_x, v_y, grid):
            new_x, new_y, v_x, v_y = dPSO_util.avoidObstacle(self.agent, v_x, v_y, V_LIMIT, grid)
            self.agent.setcoords(new_x, new_y)


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

def move(swarm, agent, dx, dy, grid):
    """Move agent by (dx,dy) if no collision"""
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

            # # deal root out of map
            # if (agent.x < 0 or agent.y < 0 or agent.x > SIDE or agent.y > SIDE):
            #     agent.x = agent.x - v_x
            #     agent.y = agent.y - v_y
            #     agent.Vx = -agent.Vx
            #     agent.Vy = -agent.Vy

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

    # for steps in range(max_steps):

    #     w = w_upper - (steps / max_steps) * (w_upper - w_lower)

    #     for swarm in swarms:
    #         swarm.update_particles(grid, swarms, V_LIMIT, goal_list_x, goal_list_y, c1, c2, w = w)
    #         swarm.update_agent_position(V_LIMIT)
    #         grid.pos_change(swarm.particles)

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




