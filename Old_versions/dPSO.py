import numpy as np
import math
import random
from Continuous_grid import visualize_grid

class Particle:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.vx = 0
        self.vy = 0
        self.pbest_x = x
        self.pbest_y = y
        self.fitness = np.inf

class Swarm:
    def __init__(self, agent, grid, num_particles=30, sigma=3):
        self.agent = agent
        self.agent.grid = grid  # set the grid here
        self.particles = []
        self.gbest_x = agent.x
        self.gbest_y = agent.y
        self.gbest_value = np.inf
        self.sigma = sigma
        self.initialize_particles(num_particles)

    def initialize_particles(self, num_particles):
        #self.agent.grid = None  # will be set externally
        attempts = 0
        while len(self.particles) < num_particles and attempts < num_particles * 10:
            x = np.random.normal(self.agent.x, self.sigma)
            y = np.random.normal(self.agent.y, self.sigma)
            if not is_collision(self.agent.grid, x, y, self.agent.radius):
                self.particles.append(Particle(x, y))
            attempts += 1

    def update_particles(self, grid, all_swarms, c1=2, c2=2, c3=1, c4=1, w=0.5):
        for p in self.particles:
            distances = [
    math.sqrt((p.x - tx)**2 + (p.y - ty)**2)
    for target in grid.targets if target is not None
    for tx, ty, _ in [target]
]
            fitness = min(distances) if distances else np.inf

            if fitness < p.fitness:
                p.pbest_x, p.pbest_y = p.x, p.y
                p.fitness = fitness
                if fitness < self.gbest_value:
                    self.gbest_x, self.gbest_y = p.x, p.y
                    self.gbest_value = fitness

        for p in self.particles:
            r1, r2, r3, r4 = np.random.rand(4)
            vel_x = w * p.vx + c1 * r1 * (p.pbest_x - p.x) + c2 * r2 * (self.gbest_x - p.x)
            vel_y = w * p.vy + c1 * r1 * (p.pbest_y - p.y) + c2 * r2 * (self.gbest_y - p.y)

            for other in self.particles:
                if other == p: continue
                d = math.hypot(p.x - other.x, p.y - other.y)
                if d == 0: continue
                rep = 1 / (1 + (d / 1.5)**2)
                vel_x += c3 * r3 * (p.x - other.x) * rep
                vel_y += c3 * r3 * (p.y - other.y) * rep

            for swarm in all_swarms:
                if swarm == self: continue
                d = math.hypot(p.x - swarm.agent.x, p.y - swarm.agent.y)
                rep = 1 / (1 + (d / 2.0)**2)
                vel_x += c4 * r4 * (p.x - swarm.agent.x) * rep
                vel_y += c4 * r4 * (p.y - swarm.agent.y) * rep

            # Apply and check for collisions
            new_x = p.x + vel_x
            new_y = p.y + vel_y
            if not is_collision(grid, new_x, new_y, grid.agent_radius):
                p.vx, p.vy = vel_x, vel_y
                p.x, p.y = new_x, new_y
            else:
                # Small random movement to escape stagnation
                angle = random.uniform(0, 2 * math.pi)
                p.vx = math.cos(angle) * 0.5
                p.vy = math.sin(angle) * 0.5
                alt_x = p.x + p.vx
                alt_y = p.y + p.vy
                if not is_collision(grid, alt_x, alt_y, grid.agent_radius):
                    p.x, p.y = alt_x, alt_y
                else:
                    p.vx = p.vy = 0

    def update_agent_position(self):
        avg_x = np.mean([p.x for p in self.particles])
        avg_y = np.mean([p.y for p in self.particles])
        self.agent.setcoords(avg_x, avg_y)


# --- Collision Detection Function ---
def is_collision(grid, x, y, radius):
    if x - radius < 0 or x + radius > grid.width or y - radius < 0 or y + radius > grid.height:
        return True
    for obs_x, obs_y, obs_w, obs_h in grid.obstacles:
        closest_x = max(obs_x - obs_w, min(x, obs_x + obs_w))
        closest_y = max(obs_y - obs_h, min(y, obs_y + obs_h))
        distance = math.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)
        if distance < radius:
            return True
    return False

# --- Target Detection Function ---
def check_target_detection(grid):
    for agent in grid.agents:
        for i, target in enumerate(grid.targets):
            if target is None:
                continue
            target_x, target_y, target_radius = target
            distance = math.sqrt((agent.x - target_x) ** 2 + (agent.y - target_y) ** 2)
            if distance < agent.radius + target_radius:
                agent.targets_found += 1
                grid.remove_target(target_x, target_y, target_radius)
                grid.targets[i] = None
                return True
    return False

# --- Main DPSO Execution ---
def run_dpso(grid, iterations, step_size):
    total_targets = len(grid.targets)
    targets = 0
    swarms = [Swarm(agent, grid) for agent in grid.agents]

    total_distance = 0
    previous_positions = [(agent.x, agent.y) for agent in grid.agents]

    for t in range(iterations):
        for swarm in swarms:
            swarm.update_particles(grid, swarms)
            swarm.update_agent_position()

        grid.pos_change()

        if check_target_detection(grid):
            targets +=1

        if targets == total_targets:
            break

        # Calculate distance moved by all agents in this step
        for i, agent in enumerate(grid.agents):
            prev_x, prev_y = previous_positions[i]
            dist = math.sqrt((agent.x - prev_x)**2 + (agent.y - prev_y)**2)
            total_distance += dist
            previous_positions[i] = (agent.x, agent.y)

        if t % 20 == 0:
            visualize_grid(grid, t, done=True, name='dPSO')
    
    total_distance = grid.total_distance_covered()
    return t, targets, total_distance




